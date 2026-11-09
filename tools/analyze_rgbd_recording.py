"""Replay all saved RGB-D attempts through selected position methods; no devices."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time
import zipfile
import zlib

DEFAULT_METHODS = ('model_translation', 'surface_wrist')
POSITION_METHODS = (*DEFAULT_METHODS, 'wrist_surface_fit')
OBSERVED_POSITION_EVIDENCE = {'rgbd_fused', 'stereo'}


def distribution(values):
    """Dependency-free audit summary; empty distributions remain null."""
    ordered = sorted(values)
    if not ordered:
        return dict(count=0, min=None, p50=None, p95=None, p99=None, max=None, mean=None)

    def quantile(fraction):
        index = (len(ordered)-1)*fraction
        low, high = math.floor(index), math.ceil(index)
        return ordered[low]+(ordered[high]-ordered[low])*(index-low)

    return dict(count=len(ordered), min=ordered[0], p50=quantile(.5),
                p95=quantile(.95), p99=quantile(.99), max=ordered[-1],
                mean=math.fsum(ordered)/len(ordered))


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_recorded_depth(path):
    """Preserve corrupt/missing archive attempts without fabricating depth."""
    import numpy as np
    digest = None
    try:
        digest = file_sha256(path)
        with np.load(path) as frame:
            depth = frame['depth_m']
        if depth.ndim != 2:
            raise ValueError('Recorded depth is not a 2D image')
    except (OSError, ValueError, KeyError, EOFError, zipfile.BadZipFile, zlib.error) as error:
        return None, digest, dict(type=type(error).__name__, message=str(error))
    return depth, digest, None


def model_orientation_pair(previous, current):
    """Match the supervisor's angular jump guard on consecutive selected frames."""
    if previous is None or current is None:
        return None
    if current['sequence'] != previous['sequence']+1:
        return None
    if any(current[key] != previous[key] for key in ('track_id', 'frame_id', 'geometry_sha256')):
        return None
    delta = current['timestamp_s']-previous['timestamp_s']
    if not math.isfinite(delta) or delta <= 0:
        return None
    # trace(R_previous.T @ R_current), with camera-model orientation unchanged.
    trace = math.fsum(a*b for ar, br in zip(previous['rotation'], current['rotation'])
                      for a, b in zip(ar, br))
    angle = math.acos(max(-1., min(1., (trace-1.)/2)))
    limit = .12+8.*delta
    return dict(previous_sequence=previous['sequence'], dt_s=delta,
                angle_rad=angle, guard_limit_rad=limit, guard_exceeded=angle > limit)


class MethodSummary:
    """Pure summary logic independent of camera SDKs and numerical environments."""

    def __init__(self):
        self.counts = Counter()
        self.attempts = self.detected = self.available = self.pose_present = 0
        self.position_evidence = Counter()
        self.timings = []
        self.run_state = None
        self.run_frames = 0
        self.run_time_start = self.last_timestamp = None
        self.longest_frames = {True: 0, False: 0}
        self.longest_spans = {True: 0., False: 0.}
        self.missing_timestamps = self.nonincreasing_timestamps = 0
        self.previous = None
        self.steps, self.speeds, self.intervals = [], [], []
        self.pair_rejections = Counter()
        self.observed_orientation_pairs = []

    def add(self, *, sequence, timestamp_s, detected, observation=None,
            diagnostics=None, elapsed_s=None, geometry_sha256=None, model_orientation=None):
        diagnostics = diagnostics or {}
        self.attempts += 1
        self.detected += int(detected)
        self.counts[diagnostics.get('reason', 'capture_error')] += 1
        pose = None if observation is None else observation.get('pose')
        evidence = 'unavailable' if observation is None else observation['position_evidence']
        self.position_evidence[evidence] += 1
        available = bool(detected and pose is not None and evidence in OBSERVED_POSITION_EVIDENCE)
        self.pose_present += int(pose is not None)
        self.available += int(available)
        if elapsed_s is not None:
            self.timings.append(elapsed_s)
        timestamp_known = timestamp_s is not None and math.isfinite(timestamp_s)
        if not timestamp_known:
            self.missing_timestamps += 1
        increasing = (timestamp_known and self.last_timestamp is not None
                      and timestamp_s > self.last_timestamp)
        if timestamp_known and self.last_timestamp is not None and not increasing:
            self.nonincreasing_timestamps += 1
        if available == self.run_state:
            self.run_frames += 1
        else:
            self.run_state = available
            self.run_frames = 1
            self.run_time_start = None
        self.longest_frames[available] = max(self.longest_frames[available], self.run_frames)
        if not timestamp_known:
            self.run_time_start = None
        else:
            if self.run_time_start is None or not increasing:
                self.run_time_start = timestamp_s
            self.longest_spans[available] = max(
                self.longest_spans[available], timestamp_s-self.run_time_start)
        self.last_timestamp = timestamp_s if timestamp_known else None

        current = dict(sequence=sequence, timestamp_s=timestamp_s, available=available,
                       observation=observation, reference=diagnostics.get('position_reference'),
                       geometry_sha256=geometry_sha256)
        pair = None
        if self.previous is not None:
            previous = self.previous
            old = previous['observation']
            reason = None
            if not available or not previous['available']:
                reason = 'unavailable_position'
            elif sequence != previous['sequence']+1:
                reason = 'nonconsecutive_recorded_sequence'
            elif any(observation[k] != old[k] for k in ('track_id', 'frame_id')):
                reason = 'identity_or_frame_change'
            elif (geometry_sha256 != previous['geometry_sha256']
                  or current['reference'] != previous['reference']):
                reason = 'geometry_or_reference_change'
            elif not increasing:
                reason = 'missing_or_nonincreasing_timestamp'
            if reason is None:
                delta = timestamp_s-previous['timestamp_s']
                step = math.dist(pose['position_m'], old['pose']['position_m'])
                pair = dict(previous_sequence=previous['sequence'], dt_s=delta,
                            step_m=step, speed_m_s=step/delta)
                self.steps.append(step)
                self.speeds.append(step/delta)
                self.intervals.append(delta)
                if model_orientation is not None:
                    self.observed_orientation_pairs.append(model_orientation)
            else:
                self.pair_rejections[reason] += 1
        self.previous = current
        return pair

    def as_dict(self):
        return dict(
            counts=dict(self.counts), attempts=self.attempts,
            selected_hand_frames=self.detected, observed_position_frames=self.available,
            pose_present_frames=self.pose_present,
            position_evidence_counts=dict(self.position_evidence),
            availability=dict(
                all_attempts=dict(numerator=self.available, denominator=self.attempts,
                                  fraction=self.available/self.attempts if self.attempts else None),
                selected_hand=dict(numerator=self.available, denominator=self.detected,
                                   fraction=self.available/self.detected if self.detected else None)),
            longest_valid_frame_streak=self.longest_frames[True],
            longest_invalid_frame_streak=self.longest_frames[False],
            longest_valid_sample_span_s=self.longest_spans[True],
            longest_invalid_sample_span_s=self.longest_spans[False],
            missing_timestamps=self.missing_timestamps,
            nonincreasing_timestamps=self.nonincreasing_timestamps,
            fusion_elapsed_s=distribution(self.timings),
            model_orientation_on_observed_pairs=dict(
                pairs=len(self.observed_orientation_pairs),
                guard_exceeded_pairs=sum(pair['guard_exceeded'] for pair in self.observed_orientation_pairs),
                angle_rad=distribution([pair['angle_rad'] for pair in self.observed_orientation_pairs])),
            adjacent_observed_motion=dict(
                dt_s=distribution(self.intervals), step_m=distribution(self.steps),
                speed_m_s=distribution(self.speeds),
                steps_above_60mm=sum(step > .06 for step in self.steps),
                steps_above_100mm=sum(step > .1 for step in self.steps),
                rejected_pairs=dict(self.pair_rejections)))


def run(recording, out, methods=DEFAULT_METHODS):
    if not methods or len(set(methods)) != len(methods) or any(m not in POSITION_METHODS for m in methods):
        raise ValueError('Choose distinct supported position methods')
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    from anyteleop_so101.runtime.rgbd import fuse_frame
    from anyteleop_so101.surface_position import wrist_surface_support_mask

    recording, out = Path(recording), Path(out)
    input_path = recording/'frames.jsonl'
    input_hash = file_sha256(input_path)
    stats = {method: MethodSummary() for method in methods}
    image_inputs, detector_blobs = [], set()
    probe_digest = hashlib.sha256()
    attempts = detected = 0
    previous_orientation = None
    orientation_pairs = []
    with out.open('x') as output, input_path.open() as source:
        for line in source:
            row = json.loads(line)
            attempts += 1
            sequence = row.get('sequence', row.get('observation', {}).get('sequence'))
            result = dict(sequence=sequence, methods={})
            if 'error' in row:
                result['capture_error'] = row['error']
                previous_orientation = None
                for method in stats:
                    stats[method].add(sequence=sequence, timestamp_s=None, detected=False)
            else:
                probe, g = row['probe'], row['geometry']
                detected += int(probe['detected'])
                timestamp = row['observation']['timestamp_s']
                geometry_hash = row.get('geometry_sha256')
                frame_path = recording/f'{sequence:06d}.npz'
                depth, frame_hash, archive_error = read_recorded_depth(frame_path)
                image_inputs.append(dict(path=frame_path.name, sha256=frame_hash,
                                         archive_error=archive_error))
                probe_bytes = json.dumps(probe, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
                probe_digest.update(str(sequence).encode()+b'\n'+probe_bytes+b'\n')
                detector_blobs.add(g.get('detector_blob', 'not_recorded'))
                result.update(timestamp_s=timestamp, detected=bool(probe['detected']),
                    source_frame_number=row.get('source_frame_number'), source_frame_gap=row.get('source_frame_gap'),
                    geometry_sha256=geometry_hash, recorded_detector_git_blob=g.get('detector_blob'),
                    input_frame_sha256=frame_hash, probe_sha256=hashlib.sha256(probe_bytes).hexdigest())
                current_orientation = (dict(sequence=sequence, timestamp_s=timestamp,
                    track_id=row['observation']['track_id'], frame_id=row['observation']['frame_id'],
                    geometry_sha256=geometry_hash, rotation=probe['model_wrist_rotation'])
                    if probe['detected'] else None)
                orientation_pair = model_orientation_pair(previous_orientation, current_orientation)
                result['model_orientation_from_previous'] = orientation_pair
                if orientation_pair is not None:
                    orientation_pairs.append(orientation_pair)
                previous_orientation = current_orientation
                if archive_error is not None:
                    result.update(replay_error=archive_error, recorded_observation=row['observation'],
                                  recorded_diagnostics=row.get('diagnostics'))
                    for method in stats:
                        diagnostics = dict(reason='recording_depth_unreadable', archive=frame_path.name)
                        stats[method].add(sequence=sequence, timestamp_s=timestamp,
                            detected=bool(probe['detected']), diagnostics=diagnostics,
                            geometry_sha256=geometry_hash, model_orientation=orientation_pair)
                        result['methods'][method] = dict(observation=None, diagnostics=diagnostics,
                            fusion_elapsed_s=None, motion_from_previous=None)
                    output.write(json.dumps(result, allow_nan=False)+'\n')
                    continue
                mask = np.zeros(depth.shape, dtype=np.uint8)
                wrist_mask = np.zeros(depth.shape, dtype=bool)
                if probe['detected']:
                    uv = np.asarray(probe['pixels'])
                    cv2.fillConvexPoly(mask, cv2.convexHull(uv[[0,5,9,13,17]].astype(np.int32)), 1)
                    mask = cv2.dilate(mask, np.ones((7,7), np.uint8))
                    wrist_mask = wrist_surface_support_mask(uv, depth.shape)
                intrinsics = rs.intrinsics()
                for field in ('width','height','fx','fy','ppx','ppy','coeffs'):
                    setattr(intrinsics, field, g[field])
                intrinsics.model = getattr(rs.distortion, g['distortion'].split('.')[-1])
                for method in stats:
                    started = time.perf_counter()
                    observation, diagnostics = fuse_frame(probe, depth, mask.astype(bool),
                        lambda pixel,z: rs.rs2_deproject_pixel_to_point(intrinsics,pixel,z),
                        timestamp_s=timestamp, track_id=row['observation']['track_id'],
                        frame_id=row['observation']['frame_id'],
                        aperture_calibration=g['aperture_calibration'], position_method=method,
                        wrist_support_mask=wrist_mask if method == 'wrist_surface_fit' else None)
                    elapsed = time.perf_counter()-started
                    encoded = observation.as_dict()
                    pair = stats[method].add(sequence=sequence, timestamp_s=timestamp,
                        detected=bool(probe['detected']), observation=encoded,
                        diagnostics=diagnostics, elapsed_s=elapsed, geometry_sha256=geometry_hash,
                        model_orientation=orientation_pair)
                    result['methods'][method] = dict(observation=encoded, diagnostics=diagnostics,
                                                     fusion_elapsed_s=elapsed, motion_from_previous=pair)
            output.write(json.dumps(result, allow_nan=False)+'\n')

    # Detector blob labels describe capture metadata, not a fresh source audit.
    manifest = dict(recording=str(recording.resolve()), frames_jsonl_sha256=input_hash,
                    image_files=image_inputs, probe_stream_sha256=probe_digest.hexdigest(),
                    recorded_detector_git_blobs=sorted(detector_blobs))
    manifest_path = out.with_suffix('.inputs.json')
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    method_summaries = {method: value.as_dict() for method, value in stats.items()}
    summary = dict(schema='teleop.rgbd_analysis.v2', attempts=attempts, selected_hand_frames=detected,
        unreadable_depth_archives=sum(item['archive_error'] is not None for item in image_inputs),
        methods=method_summaries,
        source_model_orientation=dict(
            eligible_adjacent_selected_pairs=len(orientation_pairs),
            angle_rad=distribution([pair['angle_rad'] for pair in orientation_pairs]),
            dt_s=distribution([pair['dt_s'] for pair in orientation_pairs]),
            guard_exceeded_pairs=sum(pair['guard_exceeded'] for pair in orientation_pairs),
            guard_formula='angle_rad > 0.12 + 8.0 * actual_dt_s',
            note='Adjacent selected same-track/frame/geometry model orientations. This is a diagnostic potential guard rejection, not an executed control replay; actual motion is included.'),
        # Preserve original summary fields for existing readers.
        counts={method: value['counts'] for method, value in method_summaries.items()},
        longest_valid_frame_streak={method: value['longest_valid_frame_streak'] for method, value in method_summaries.items()},
        provenance=dict(recording=str(recording.resolve()), frames_jsonl_sha256=input_hash,
                        input_manifest=str(manifest_path.resolve()),
                        input_manifest_sha256=file_sha256(manifest_path),
                        hashed_image_files=sum(item['sha256'] is not None for item in image_inputs),
                        probe_stream_sha256=probe_digest.hexdigest(),
                        recorded_detector_git_blobs=sorted(detector_blobs)),
        definitions=dict(
            observed_position='Selected detector frame with a pose and rgbd_fused/stereo evidence; predictions and monocular model output excluded.',
            run_duration='Elapsed timestamp span from first to last sample in an uninterrupted availability state. Single samples span zero; missing/nonincreasing timestamps break timed spans. These are sampled lower bounds, not inferred outage durations.',
            motion='Consecutive logged sequences with observed positions, identical track/frame/geometry/reference and increasing capture timestamps. Actual dt is used; source-camera gaps remain recorded.',
            timing='Wall time around fuse_frame only; includes no image read/hash, detector inference or control latency.',
            provenance='SHA-256 covers recorded files/probes; detector Git blob IDs are copied from capture metadata, not independently reverified.'),
        note='All attempts retained. Motion steps contain actual hand motion and capture gaps; they are not precision or jitter errors. Surface reference is not anatomical wrist or proven metric accuracy.')
    out.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2, allow_nan=False))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--methods', choices=POSITION_METHODS, nargs='+', default=DEFAULT_METHODS)
    args = parser.parse_args()
    run(args.recording, args.out, args.methods)
