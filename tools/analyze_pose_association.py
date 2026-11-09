"""Offline body-pose proximity audit of saved hand candidates; never selects control input."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reprobe_recorded_detector import ARCHIVE_ERRORS, file_sha256, distribution


POSE_SETTINGS = dict(static_image_mode=False, model_complexity=1, smooth_landmarks=True,
    enable_segmentation=False, min_detection_confidence=.8, min_tracking_confidence=.8)
BODY_INDICES = dict(left_elbow=13, right_elbow=14, left_wrist=15, right_wrist=16,
                    left_pinky=17, right_pinky=18, left_index=19, right_index=20)


def serialize_landmarks(group):
    if group is None:
        return None
    if len(group.landmark) != 33:
        raise ValueError('Expected all 33 body landmarks')
    return [dict({key: float(getattr(point, key)) for key in ('x', 'y', 'z', 'visibility')},
                 presence=float(point.presence) if point.HasField('presence') else None)
            for point in group.landmark]


def distance_to_polyline(point, chain):
    """Pixel proximity to a whole digit; no claim of exact pose/hand joint correspondence."""
    start, delta = chain[:-1], chain[1:]-chain[:-1]
    length2 = np.sum(delta*delta, axis=1)
    along = np.divide(np.sum((point-start)*delta, axis=1), length2,
                      out=np.zeros_like(length2), where=length2 != 0)
    closest = start+np.clip(along, 0, 1)[:, None]*delta
    return float(np.min(np.linalg.norm(closest-point, axis=1)))


def associate(pose, raw, width, height):
    """Expose measurements/rankings only; no confidence gate or identity decision."""
    hands = raw['image_landmarks'] or []
    result = dict(raw_candidate_count=len(hands), body_available=pose is not None,
                  raw_handedness=raw['handedness'],
                  candidates=[], right_wrist_nearest_indices=[], right_digits_nearest_indices=[])
    if pose is None:
        return result
    body = np.array([[point['x']*width, point['y']*height] for point in pose])
    priors = {name: dict(pixel=body[index].tolist(), visibility=pose[index]['visibility'])
              for name, index in BODY_INDICES.items()}
    right_length = float(np.linalg.norm(body[16]-body[14]))
    result.update(body_priors=priors, right_forearm_projected_length_px=right_length)
    for index, points in enumerate(hands):
        if len(points) != 21:
            raise ValueError('Expected all 21 hand landmarks')
        hand = np.array([[point['x']*width, point['y']*height] for point in points])
        right = float(np.linalg.norm(hand[0]-body[16]))
        left = float(np.linalg.norm(hand[0]-body[15]))
        index_distance = distance_to_polyline(body[20], hand[5:9])
        pinky_distance = distance_to_polyline(body[18], hand[17:21])
        result['candidates'].append(dict(candidate_index=index,
            handedness=raw['handedness'][index], wrist_pixel=hand[0].tolist(),
            right_wrist_distance_px=right, left_wrist_distance_px=left,
            right_wrist_distance_forearm_units=None if right_length == 0 else right/right_length,
            right_vs_left_distance_advantage_px=left-right,
            nearer_body_wrist='right' if right < left else 'left' if left < right else 'equal',
            right_index_to_index_chain_px=index_distance,
            right_pinky_to_pinky_chain_px=pinky_distance,
            mean_right_digit_chain_distance_px=(index_distance+pinky_distance)/2))
    if hands:
        for output, field in [('right_wrist_nearest_indices', 'right_wrist_distance_px'),
                              ('right_digits_nearest_indices', 'mean_right_digit_chain_distance_px')]:
            minimum = min(row[field] for row in result['candidates'])
            result[output] = [row['candidate_index'] for row in result['candidates'] if row[field] == minimum]
        sorted_distances = sorted(row['right_wrist_distance_px'] for row in result['candidates'])
        result['right_wrist_ranking_margin_px'] = None if len(hands) < 2 else sorted_distances[1]-sorted_distances[0]
        result['wrist_digit_ranking_disagreement'] = not bool(
            set(result['right_wrist_nearest_indices']) & set(result['right_digits_nearest_indices']))
    return result


def read_rows(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    sequences = [row.get('sequence', row.get('observation', {}).get('sequence')) for row in rows]
    if sequences != list(range(len(rows))) or not rows:
        raise ValueError('Recording must include complete consecutive attempts beginning at zero')
    return rows


def summarize(rows):
    counts = Counter(row['status'] for row in rows)
    processed = [row for row in rows if row['status'] == 'processed']
    counts['attempts'] = len(rows)
    counts['body_available'] = sum(row['pose_landmarks'] is not None for row in processed)
    counts['body_unavailable'] = len(processed)-counts['body_available']
    result = dict(counts=dict(counts), inference_elapsed_s=distribution([r['inference_elapsed_s'] for r in processed]),
        right_wrist_visibility=distribution([r['pose_landmarks'][16]['visibility'] for r in processed if r['pose_landmarks']]))
    for method in ('baseline', 'max2'):
        available = [row['association'][method] for row in processed]
        result[method] = dict(candidate_unavailable=sum(not item['raw_candidate_count'] for item in available),
            body_and_candidate_available=sum(item['body_available'] and item['raw_candidate_count'] > 0 for item in available),
            multiple_candidates=sum(item['raw_candidate_count'] > 1 for item in available),
            wrist_digit_ranking_disagreement=sum(item.get('wrist_digit_ranking_disagreement', False) for item in available),
            exact_wrist_ranking_ties=sum(len(item['right_wrist_nearest_indices']) > 1 for item in available),
            candidate_nearer_wrist=dict(Counter(candidate['nearer_body_wrist'] for item in available for candidate in item['candidates'])))
    return result


def run(recording, baseline, max2, out):
    recording, baseline, max2, out = map(Path, (recording, baseline, max2, out))
    source_path = recording/'frames.jsonl'
    source, one, two = map(read_rows, (source_path, baseline, max2))
    if not len(source) == len(one) == len(two):
        raise ValueError('Candidate recordings must retain every source attempt')
    for path in (out, out.with_suffix('.summary.json'), out.with_suffix('.inputs.json')):
        if path.exists():
            raise FileExistsError(path)
    source_hash = file_sha256(source_path)
    for name, path, expected_max in [('baseline', baseline, 1), ('max2', max2, 2)]:
        manifest = json.loads(path.with_suffix('.inputs.json').read_text())
        if manifest['frames_jsonl_sha256'] != source_hash:
            raise ValueError('Candidate source hash does not match original recording')
        settings = manifest.get('graph_settings')
        if settings is not None and settings['max_num_hands'] != expected_max:
            raise ValueError(f'{name} graph candidate count differs from declared input')
        if name == 'max2' and settings is None:
            raise ValueError('Experimental max2 candidate manifest must declare graph settings')
        if name == 'baseline' and settings is None:
            # Original audited one-hand artifact predates variant metadata.
            # Require all its saved probes to reproduce the original exactly,
            # and no result to contain more than one raw candidate.
            if any(row['status']=='reprocessed' and (not row['comparison']['exact_probe_match'] or
                    len(row['raw_mediapipe']['image_landmarks'] or []) > 1) for row in one):
                raise ValueError('Legacy baseline artifact does not reproduce one-hand source')
    for sequence, (original, a, b) in enumerate(zip(source, one, two)):
        if 'error' not in original and (a['recorded_probe'] != original['probe'] or b['recorded_probe'] != original['probe']):
            raise ValueError(f'Original probe differs at sequence {sequence}')
    out.parent.mkdir(parents=True, exist_ok=True)
    import cv2
    import mediapipe as mp
    package = Path(mp.__file__).parent
    model = package/'modules/pose_landmark/pose_landmark_full.tflite'
    if not model.is_file():
        raise FileNotFoundError('Installed model1 asset required; no downloads allowed')
    pose = mp.solutions.pose.Pose(**POSE_SETTINGS)
    outputs, image_inputs, timings = [], [], []
    abort = None
    calls = 0
    started = time.monotonic()
    try:
        with out.open('x') as destination:
            for sequence, original in enumerate(source):
                row = dict(sequence=sequence, timestamp_s=original.get('observation', {}).get('timestamp_s'))
                if 'error' in original:
                    row.update(status='capture_error_no_original_inference', error=original['error'])
                elif abort is not None:
                    row.update(status='not_processed_after_gap', error=abort)
                else:
                    archive = recording/f'{sequence:06d}.npz'
                    archive_hash = None
                    try:
                        archive_hash = file_sha256(archive)
                        with np.load(archive) as arrays:
                            bgr = arrays['color_bgr']
                        saved = original['probe']
                        if bgr.dtype != np.uint8 or bgr.shape != (saved['image_height'], saved['image_width'], 3):
                            raise ValueError('Invalid recorded image dimensions/type')
                    except ARCHIVE_ERRORS as error:
                        abort = dict(sequence=sequence, type=type(error).__name__, message=str(error))
                        row.update(status='archive_unreadable', error=abort, recorded_observation=original['observation'])
                        image_inputs.append(dict(path=archive.name, sha256=archive_hash, error=abort))
                    else:
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        image_inputs.append(dict(path=archive.name, sha256=archive_hash,
                            rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()))
                        try:
                            if one[sequence]['status'] != 'reprocessed' or two[sequence]['status'] != 'reprocessed':
                                raise ValueError('Saved hand candidate result unavailable for readable frame')
                            for saved_candidates in (one[sequence], two[sequence]):
                                if (saved_candidates['rgb_sha256'] != image_inputs[-1]['rgb_sha256'] or
                                        saved_candidates['input_archive_sha256'] != archive_hash):
                                    raise ValueError('RGB/archive differs from saved hand candidate inference input')
                            before = time.monotonic()
                            calls += 1
                            detected = pose.process(rgb)
                            elapsed = time.monotonic()-before
                            normalized = serialize_landmarks(detected.pose_landmarks)
                            world = serialize_landmarks(detected.pose_world_landmarks)
                            candidate = dict(row, status='processed', pose_landmarks=normalized,
                                pose_world_landmarks=world, inference_elapsed_s=elapsed,
                                source_archive_sha256=archive_hash,
                                association={name: dict(associate(normalized, rows[sequence]['raw_mediapipe'],
                                    bgr.shape[1], bgr.shape[0]),
                                    original_upstream_selected=rows[sequence]['replayed_probe']['detected'],
                                    original_selected_candidate_index=rows[sequence].get('selected_candidate_index',
                                        0 if rows[sequence]['replayed_probe']['detected'] else None))
                                    for name, rows in [('baseline', one), ('max2', two)]})
                            json.dumps(candidate, allow_nan=False)
                            row = candidate
                            timings.append(elapsed)
                        except Exception as error:
                            abort = dict(sequence=sequence, type=type(error).__name__, message=str(error))
                            row.update(status='inference_or_analysis_error', error=abort)
                destination.write(json.dumps(row, allow_nan=False)+'\n')
                outputs.append(row)
    finally:
        pose.close()
    files = [Path(__file__), package/'python/solutions/pose.py',
             package/'modules/pose_landmark/pose_landmark_cpu.binarypb', model,
             package/'modules/pose_detection/pose_detection.tflite']
    manifest = dict(recording=str(recording.resolve()), frames_jsonl_sha256=source_hash,
        baseline=str(baseline.resolve()), baseline_sha256=file_sha256(baseline),
        max2=str(max2.resolve()), max2_sha256=file_sha256(max2), settings=POSE_SETTINGS,
        files=[dict(path=str(path.resolve()), sha256=file_sha256(path)) for path in files],
        images=image_inputs, mediapipe=mp.__version__, numpy=np.__version__, opencv=cv2.__version__,
        note='No hand inference, modification or confidence change. Body world is model-relative to hips, not measured camera-global hand position. Legacy Pose processes current images with its default pose smoothing and simulated internal 30Hz timestamps; recorded capture timestamps are preserved separately.')
    out.with_suffix('.inputs.json').write_text(json.dumps(manifest, indent=2)+'\n')
    summary = summarize(outputs)
    summary.update(process_calls=calls, finished_utc=datetime.now(timezone.utc).isoformat(),
        wall_elapsed_s=time.monotonic()-started, abort_reason=abort,
        continuity='stopped_after_gap' if any(row['status']=='not_processed_after_gap' for row in outputs) else
            'terminal_unreadable_archive' if abort and outputs[-1]['status']=='archive_unreadable' else
            'terminal_analysis_error' if abort else 'complete_recording',
        blocks={name: summarize(outputs[start:end+1]) for name,start,end in [
            ('distractor',9,164),('palm_down_label_flip',2435,2468),('long_no_hand',1550,1744)]},
        note='Continuous diagnostic proximities/rankings only. No tuned thresholds, inferred recovery count, hand relabeling, runtime selection, metric-depth substitution, or identity guarantee.')
    out.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2, allow_nan=False))
    if any(row['status'] in ('not_processed_after_gap','inference_or_analysis_error') for row in outputs):
        raise RuntimeError('Incomplete analysis; retained failures in output')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--max2', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.recording, args.baseline, args.max2, args.out)
