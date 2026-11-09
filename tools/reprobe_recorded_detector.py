"""Offline one-pass instrumentation of the unchanged upstream hand detector.

Reads recorded RGB arrays only. No cameras, robot SDKs, or source edits. The
default retains the exact upstream graph. Explicit --max-hands 2 replaces only
that graph's candidate count, marking an experimental variant. The process tap
returns the exact original MediaPipe result to the original detect() method;
no second inference or alternative handedness selection is performed.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import zipfile
import zlib


DETECTOR_BLOB = '4867ad7d2e5ea3984002c1730b937d06b1fe97ee'
ARCHIVE_ERRORS = (OSError, ValueError, KeyError, EOFError, zipfile.BadZipFile, zlib.error)


class ProcessTap:
    """Transparent process proxy: one original call, same input and return object."""
    def __init__(self, processor):
        self.processor = processor
        self.calls = 0
        self.last_result = None

    def process(self, image):
        self.calls += 1
        self.last_result = None
        result = self.processor.process(image)
        self.last_result = result
        return result

    def __getattr__(self, name):
        return getattr(self.processor, name)


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def detector_blob(data):
    return hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()


def serialize_raw_results(results):
    """Keep every returned landmark and classification, including filtered hands."""
    def landmarks(groups):
        if groups is None:
            return None
        return [[{key: float(getattr(point, key)) for key in ('x', 'y', 'z', 'visibility', 'presence')}
                 for point in group.landmark] for group in groups]

    handedness = results.multi_handedness
    return dict(
        image_landmarks=landmarks(results.multi_hand_landmarks),
        world_landmarks=landmarks(results.multi_hand_world_landmarks),
        handedness=None if handedness is None else [
            [dict(index=int(item.index), label=item.label, score=float(item.score))
             for item in group.classification] for group in handedness])


def selection_reason(raw, expected_label, selected_count):
    found = len(raw['image_landmarks'] or [])
    labels = [items[0]['label'] if items else None for items in raw['handedness'] or []]
    if selected_count:
        return 'upstream_selected' if found and expected_label in labels else 'selection_inconsistent'
    if not found:
        return 'mediapipe_no_hand'
    return 'handedness_filtered' if expected_label not in labels else 'selection_inconsistent'


def capture_raw_after_error(result):
    """A failed diagnostic serializer must not hide the original failed attempt."""
    try:
        raw = serialize_raw_results(result)
        json.dumps(raw, allow_nan=False)
    except Exception as error:
        return dict(raw_capture_error=dict(type=type(error).__name__, message=str(error)))
    return dict(raw_mediapipe=raw)


def compare_probe(saved, replayed):
    """Report exact equality and numerical differences; no acceptance threshold."""
    import numpy as np
    result = dict(selected_match=bool(saved['detected']) == bool(replayed['detected']),
                  exact_probe_match=saved == replayed)
    if saved['detected'] and replayed['detected']:
        for field, unit in (('pixels', 'px'), ('local_points_mano', 'm'),
                            ('model_wrist_rotation', 'unitless')):
            delta = np.asarray(replayed[field], dtype=float)-np.asarray(saved[field], dtype=float)
            result[field] = dict(max_abs=float(np.max(np.abs(delta))),
                                 rms=float(np.sqrt(np.mean(delta**2))), unit=unit)
        before = np.asarray(saved['model_wrist_rotation'])
        after = np.asarray(replayed['model_wrist_rotation'])
        result['rotation_delta_rad'] = float(np.arccos(np.clip((np.trace(before.T@after)-1)/2, -1, 1)))
    return result


def distribution(values):
    values = sorted(values)
    if not values:
        return dict(count=0, p50=None, p95=None, max=None)
    def quantile(p):
        index = (len(values)-1)*p
        lo, hi = math.floor(index), math.ceil(index)
        return values[lo]+(values[hi]-values[lo])*(index-lo)
    return dict(count=len(values), p50=quantile(.5), p95=quantile(.95), max=values[-1])


def package_versions():
    versions = {}
    for name in ('mediapipe', 'numpy', 'opencv-python', 'opencv-contrib-python',
                 'opencv-python-headless', 'opencv-contrib-python-headless', 'protobuf'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def create_detector(module, hand, max_hands):
    """Keep the default upstream path; candidate-count experiments are explicit."""
    if max_hands not in (1, 2):
        raise ValueError('max_hands must be 1 or 2')
    detector = module.SingleHandDetector(hand_type=hand, selfie=False)
    if max_hands == 2:
        # Close the original graph before processing any images. Only the new
        # isolated graph receives this recording; original detect() and its
        # handedness selection / all 21 keypoints are untouched. These settings
        # are identical to the pinned source except max_num_hands.
        detector.hand_detector.close()
        detector.hand_detector = module.mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2,
            min_detection_confidence=0.8, min_tracking_confidence=0.8)
    return detector


def run(recording, out, detector_repo, *, max_hands=1):
    recording, out, detector_repo = map(Path, (recording, out, detector_repo))
    source_path = recording/'frames.jsonl'
    rows = [json.loads(line) for line in source_path.read_text().splitlines()]
    # Reject malformed/nonfinite input before creating a graph or output file.
    for row in rows:
        json.dumps(row, allow_nan=False)
    sequences = [row.get('sequence', row.get('observation', {}).get('sequence')) for row in rows]
    if (not sequences or sequences[0] != 0 or any(type(n) is not int for n in sequences)
            or any(b != a+1 for a, b in zip(sequences, sequences[1:]))):
        raise ValueError('Source sequence must begin at zero and be complete and consecutive; cannot reproduce detector state')
    source_frames = [row for row in rows if 'error' not in row]
    hands = {row['geometry']['selected_hand'] for row in source_frames}
    blobs = {row['geometry']['detector_blob'] for row in source_frames}
    if len(hands) != 1 or not hands <= {'Left', 'Right'} or blobs != {DETECTOR_BLOB}:
        raise ValueError('Recording must use one selected hand and the unchanged audited detector')
    hand = next(iter(hands))
    detector_path = detector_repo/'example/vector_retargeting/single_hand_detector.py'
    detector_source = detector_path.read_bytes()
    if detector_blob(detector_source) != DETECTOR_BLOB:
        raise ValueError('Upstream detector differs from the recorded audited blob')
    for path in (out, out.with_suffix('.summary.json'), out.with_suffix('.inputs.json')):
        if path.exists():
            raise FileExistsError(path)
    out.parent.mkdir(parents=True, exist_ok=True)

    # Import expensive inference libraries only for an explicitly invoked run.
    import cv2
    import numpy as np
    spec = importlib.util.spec_from_file_location('unchanged_upstream_reprobe', detector_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Identical constructor arguments to vision_rgbd.py; upstream defaults retain
    # confidence thresholds, temporal tracking and max_num_hands=1.
    detector = create_detector(module, hand, max_hands)
    tap = ProcessTap(detector.hand_detector)
    detector.hand_detector = tap
    counts = Counter({key: 0 for key in (
        'attempts', 'recorded_selected', 'capture_errors', 'archive_errors',
        'reprocessed', 'not_reprocessed_after_gap', 'inference_or_instrumentation_errors',
        'upstream_selected', 'mediapipe_no_hand', 'handedness_filtered',
        'selection_inconsistent', 'selected_match', 'exact_probe_match')})
    comparisons, durations, mismatches, inputs = [], [], [], []
    abort_reason = None
    started = time.monotonic()
    try:
        with out.open('x') as destination:
            for sequence, source in zip(sequences, rows):
                counts['attempts'] += 1
                saved = source.get('probe')
                if saved is not None:
                    counts['recorded_selected'] += int(saved['detected'])
                result = dict(sequence=sequence, recorded_probe=saved)
                serialized_result = None
                if 'error' in source:
                    counts['capture_errors'] += 1
                    result.update(status='capture_error_no_original_inference', capture_error=source['error'])
                elif abort_reason is not None:
                    counts['not_reprocessed_after_gap'] += 1
                    result.update(status='not_reprocessed_after_gap', cause=abort_reason)
                else:
                    archive = recording/f'{sequence:06d}.npz'
                    archive_hash = None
                    try:
                        archive_hash = file_sha256(archive)
                        with np.load(archive) as arrays:
                            image_bgr = arrays['color_bgr']
                        if (image_bgr.dtype != np.uint8 or image_bgr.shape !=
                                (saved['image_height'], saved['image_width'], 3)):
                            raise ValueError('Recorded RGB array does not match the original probe image')
                    except ARCHIVE_ERRORS as error:
                        counts['archive_errors'] += 1
                        abort_reason = dict(sequence=sequence, type=type(error).__name__, message=str(error))
                        result.update(status='archive_unreadable', error=abort_reason,
                                      recorded_observation=source['observation'])
                        inputs.append(dict(path=archive.name, sha256=archive_hash, error=abort_reason))
                    else:
                        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
                        inputs.append(dict(path=archive.name, sha256=archive_hash,
                                           rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()))
                        result.update(input_archive_sha256=archive_hash,
                                      rgb_sha256=inputs[-1]['rgb_sha256'])
                        before = tap.calls
                        inference_started = time.monotonic()
                        try:
                            n, local, points, rotation = detector.detect(rgb)
                            elapsed = time.monotonic()-inference_started
                            if tap.calls-before != 1 or tap.last_result is None:
                                raise RuntimeError('Expected exactly one upstream process call/result')
                            raw = serialize_raw_results(tap.last_result)
                            uv = None if not n else detector.parse_keypoint_2d(points, image_bgr.shape[:2])
                            replayed = dict(schema='upstream.probe.v2', sequence=sequence,
                                detected=bool(n), image_width=image_bgr.shape[1], image_height=image_bgr.shape[0],
                                local_points_mano=None if not n else local.tolist(),
                                model_wrist_rotation=None if not n else rotation.tolist(),
                                operator2mano=detector.operator2mano.tolist(), pixels=None if not n else uv.tolist())
                            comparison = compare_probe(saved, replayed)
                            reason = selection_reason(raw, detector.detected_hand_type, n)
                            selected_index = next((index for index, items in enumerate(raw['handedness'] or [])
                                if items and items[0]['label'] == detector.detected_hand_type), None) if n else None
                            candidate = dict(result, status='reprocessed', raw_mediapipe=raw,
                                upstream_return_count=int(n), selected_hand_count=int(bool(n)),
                                raw_candidate_count=len(raw['image_landmarks'] or []),
                                selected_candidate_index=selected_index,
                                expected_raw_handedness=detector.detected_hand_type,
                                selection_reason=reason, replayed_probe=replayed,
                                comparison=comparison, detect_elapsed_s=elapsed,
                                process_calls=tap.calls-before)
                            # Validate before counting success. A nonfinite model
                            # result is an explicit instrumentation failure, not
                            # a missing JSONL row or an implicit temporal skip.
                            serialized_result = json.dumps(candidate, allow_nan=False)
                            result = candidate
                            counts['reprocessed'] += 1
                            counts[reason] += 1
                            counts['selected_match'] += int(comparison['selected_match'])
                            counts['exact_probe_match'] += int(comparison['exact_probe_match'])
                            if not comparison['selected_match']:
                                mismatches.append(sequence)
                            comparisons.append(comparison)
                            durations.append(elapsed)
                        except Exception as error:
                            counts['inference_or_instrumentation_errors'] += 1
                            abort_reason = dict(sequence=sequence, type=type(error).__name__, message=str(error))
                            result.update(status='inference_or_instrumentation_error', error=abort_reason,
                                          process_calls=tap.calls-before)
                            if tap.last_result is not None:
                                result.update(capture_raw_after_error(tap.last_result))
                destination.write((serialized_result or json.dumps(result, allow_nan=False))+'\n')
    finally:
        tap.close()
    source_unchanged = detector_path.read_bytes() == detector_source
    manifest = dict(recording=str(recording.resolve()), frames_jsonl_sha256=file_sha256(source_path),
        detector_path=str(detector_path), detector_git_blob=DETECTOR_BLOB,
        detector_sha256=hashlib.sha256(detector_source).hexdigest(), detector_source_unchanged=source_unchanged,
        script_sha256=file_sha256(Path(__file__)), images=inputs,
        python=sys.version, packages=package_versions(),
        imported_versions=dict(numpy=np.__version__, opencv=cv2.__version__),
        variant='upstream_baseline' if max_hands == 1 else 'experimental_max_num_hands_2',
        graph_settings=dict(static_image_mode=False, max_num_hands=max_hands,
            min_detection_confidence=0.8, min_tracking_confidence=0.8),
        original_detect_and_handedness_selection_unchanged=True)
    manifest['candidate_index_semantics'] = 'Frame-local ordering, not persistent identity; original detect selects first matching handedness label.'
    manifest_path = out.with_suffix('.inputs.json')
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    summary = dict(schema='teleop.raw_detector_reprobe.v1',
        variant=manifest['variant'], graph_settings=manifest['graph_settings'],
        finished_utc=datetime.now(timezone.utc).isoformat(), counts=dict(counts),
        total_process_calls=tap.calls, selected_mismatch_sequences=mismatches,
        detect_elapsed_s=distribution(durations), wall_elapsed_s=time.monotonic()-started,
        comparison_scope=('Exact baseline reproduction' if max_hands == 1 else 'Experimental candidate-count variant versus recorded one-hand baseline') +
            '; original numeric selected-count was not logged. Compare selected boolean and all 21 selected landmarks/rotation without threshold changes.',
        differences={field: distribution([r[field]['max_abs'] for r in comparisons if field in r])
            for field in ('pixels', 'local_points_mano', 'model_wrist_rotation')},
        continuity=('stopped_after_input_or_inference_gap' if counts['not_reprocessed_after_gap'] else
                    'terminal_inference_or_instrumentation_error' if counts['inference_or_instrumentation_errors'] else
                    'terminal_unreadable_archive' if counts['archive_errors'] else 'complete_recording'),
        abort_reason=abort_reason, detector_source_unchanged=source_unchanged,
        input_manifest=str(manifest_path.resolve()), input_manifest_sha256=file_sha256(manifest_path),
        note='One process call per readable acquired RGB frame, chronological order, fresh graph with declared graph_settings, no second inference. Capture failures had no original inference. Wrong-handedness observations retain all raw landmarks; original detect and handedness selection remain unchanged. Candidate index is frame-local, not stable identity; multiple matching labels are selected by the unchanged first-match rule. upstream_return_count is total raw boxes on selection, otherwise zero; selected_hand_count is zero or one. Experimental differences are variant effects, not baseline nondeterminism or inferred original hidden results.')
    out.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2, allow_nan=False))
    if not source_unchanged or counts['inference_or_instrumentation_errors'] or counts['not_reprocessed_after_gap']:
        raise RuntimeError('Reprobe was not complete; inspect retained attempts and summary')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--detector-repo', type=Path, default=Path('/home/bx/repos/dex-retargeting'))
    parser.add_argument('--max-hands', type=int, choices=(1, 2), default=1,
        help='Default 1 is unchanged upstream. Explicit 2 is an experimental candidate-count graph only.')
    args = parser.parse_args()
    run(args.recording, args.out, args.detector_repo, max_hands=args.max_hands)
