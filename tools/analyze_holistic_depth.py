"""Offline depth A/B using saved Holistic probes and unchanged current-depth fusion.

No detector inference, camera pipeline, robot interface or runtime configuration.
The override is explicit: the Holistic probe is a separate body-right model
output, fused with the original recorded registered depth and SDK intrinsics.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import time

from analyze_pose_association import read_rows
from analyze_rgbd_recording import MethodSummary, OBSERVED_POSITION_EVIDENCE, read_recorded_depth
from reprobe_recorded_detector import DETECTOR_BLOB, file_sha256, distribution


VARIANTS = ('baseline', 'holistic')


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def observed(observation):
    return (observation is not None and observation.get('pose') is not None
            and observation.get('position_evidence') in OBSERVED_POSITION_EVIDENCE)


def comparison_kind(before, after):
    return ('both_valid' if before else 'gained_position') if after else (
        'lost_position' if before else 'neither_valid')


def availability_spans(rows, variant):
    """Retain every valid/invalid run; no invented duration for unknown timestamps."""
    spans = []
    for row in rows:
        available = observed(row['methods'][variant]['observation'])
        timestamp = row['timestamp_s']
        if not spans or spans[-1]['available'] != available:
            spans.append(dict(available=available, first_sequence=row['sequence'],
                last_sequence=row['sequence'], frames=0, first_timestamp_s=timestamp,
                last_timestamp_s=timestamp, missing_timestamps=0, sample_span_s=None))
        span = spans[-1]
        span.update(last_sequence=row['sequence'], last_timestamp_s=timestamp)
        span['frames'] += 1
        span['missing_timestamps'] += timestamp is None
        if not span['missing_timestamps']:
            span['sample_span_s'] = timestamp-span['first_timestamp_s']
    return spans


def summarize(rows):
    counts = Counter(row['status'] for row in rows)
    compared = [row for row in rows if row['status'] == 'processed']
    counts.update(row['comparison_kind'] for row in compared)
    counts.update(attempts=len(rows), compared_readable=len(compared),
        recorded_observed_all_attempts=sum(observed(row['recorded_observation']) for row in rows),
        recorded_observed_readable=sum(observed(row['recorded_observation']) for row in compared),
        baseline_replayed_observed=sum(observed(row['methods']['baseline']['observation']) for row in compared),
        holistic_observed=sum(observed(row['methods']['holistic']['observation']) for row in compared))
    methods = {}
    for variant in VARIANTS:
        stats = MethodSummary()
        guard_pairs = []
        for row in rows:
            item = row['methods'][variant]
            stats.add(sequence=row['sequence'], timestamp_s=row['timestamp_s'],
                detected=item['detected'], observation=item['observation'],
                diagnostics=item['diagnostics'], elapsed_s=item['fusion_elapsed_s'],
                geometry_sha256=row.get('source_geometry_sha256'))
            if (item['motion_from_previous'] is not None
                    and item['motion_from_previous']['previous_sequence'] >= rows[0]['sequence']):
                guard_pairs.append(dict(sequence=row['sequence'], **item['motion_from_previous']))
        methods[variant] = dict(stats.as_dict(), availability_spans=availability_spans(rows, variant),
            adjacent_position_guard=dict(pairs=len(guard_pairs),
                exceeded_pairs=sum(pair['guard_exceeded'] for pair in guard_pairs),
                exceeded=[pair for pair in guard_pairs if pair['guard_exceeded']],
                formula='step_m > 0.015 + 1.5 * actual_dt_s'))
    return dict(counts=dict(counts), methods=methods,
        changed_sequences={kind:[r['sequence'] for r in compared if r['comparison_kind'] == kind]
                           for kind in ('gained_position','lost_position')},
        baseline_replay_availability_mismatch=[r['sequence'] for r in compared
            if observed(r['recorded_observation']) != observed(r['methods']['baseline']['observation'])],
        baseline_replay_position_max_abs_m=distribution([r['baseline_replay_position_max_abs_m']
            for r in compared if r['baseline_replay_position_max_abs_m'] is not None]),
        both_valid_position_difference_m=distribution([r['both_valid_position_difference_m']
            for r in compared if r['both_valid_position_difference_m'] is not None]))


def run(recording, holistic, out):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    import anyteleop_so101.runtime.rgbd as rgbd
    import anyteleop_so101.surface_position as surface
    import anyteleop_so101.perception as perception
    import anyteleop_so101.observation_adapter as observation_adapter

    recording, holistic, out = map(Path, (recording, holistic, out))
    source_path = recording/'frames.jsonl'
    source, alternatives = map(read_rows, (source_path, holistic))
    manifest_path = holistic.with_suffix('.inputs.json')
    previous_manifest = json.loads(manifest_path.read_text())
    source_hash, holistic_hash = file_sha256(source_path), file_sha256(holistic)
    if previous_manifest['frames_jsonl_sha256'] != source_hash or len(source) != len(alternatives):
        raise ValueError('Holistic probes must retain the complete original recording')
    for path in (out, out.with_suffix('.summary.json'), out.with_suffix('.inputs.json')):
        if path.exists():
            raise FileExistsError(path)
    files = [Path(__file__), Path(rgbd.__file__), Path(surface.__file__),
             Path(perception.__file__), Path(observation_adapter.__file__),
             Path(__file__).with_name('analyze_rgbd_recording.py')]
    code_hashes = {str(path.resolve()): file_sha256(path) for path in files}
    surface_hash = file_sha256(Path(surface.__file__))
    for sequence, (original, alternate) in enumerate(zip(source, alternatives)):
        if alternate['recorded_probe'] != original.get('probe'):
            raise ValueError(f'Holistic source probe differs at {sequence}')
        if alternate['timestamp_s'] != original.get('observation', {}).get('timestamp_s'):
            raise ValueError(f'Holistic source timestamp differs at {sequence}')
        if 'error' not in original:
            geometry = original['geometry']
            if (geometry['detector_blob'] != DETECTOR_BLOB or geometry['selected_hand'] != 'Right'
                    or geometry['position_method'] != 'wrist_surface_fit'
                    or geometry['surface_fit_version'] != surface.SURFACE_FIT_VERSION
                    or geometry['surface_estimator_sha256'] != surface_hash):
                raise ValueError('Recorded estimator/baseline differs; refusing changed depth method')
    image_manifest = {image['path']: image for image in previous_manifest['images']}
    outputs, image_inputs = [], []
    stats = {name: MethodSummary() for name in VARIANTS}
    last = {name: None for name in VARIANTS}
    started = time.monotonic()
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as destination:
        for sequence, (original, alternate) in enumerate(zip(source, alternatives)):
            observation = original.get('observation')
            row = dict(sequence=sequence,
                timestamp_s=None if observation is None else observation['timestamp_s'],
                source_frame_number=original.get('source_frame_number'),
                source_frame_gap=original.get('source_frame_gap'),
                source_geometry_sha256=original.get('geometry_sha256'),
                recorded_observation=observation, recorded_diagnostics=original.get('diagnostics'),
                recorded_probe_sha256=None if 'probe' not in original else digest_json(original['probe']),
                holistic_probe_sha256=None if 'probe' not in alternate else digest_json(alternate['probe']),
                holistic_selection_status=alternate['status'], methods={})
            depth = None
            if 'error' in original:
                row.update(status='capture_error_no_original_inference', error=original['error'])
            elif alternate['status'] != 'processed':
                archive = recording/f'{sequence:06d}.npz'
                actual_hash = file_sha256(archive)
                if actual_hash != image_manifest[archive.name]['sha256']:
                    raise ValueError(f'Unprocessed source archive changed at {sequence}')
                row.update(status='holistic_input_unreadable', error=alternate.get('error'),
                           input_archive_sha256=actual_hash)
                image_inputs.append(dict(path=archive.name, sha256=actual_hash, status=row['status']))
                if sequence != len(source)-1 or alternate['status'] != 'archive_unreadable':
                    raise ValueError('Holistic input contains an interior gap; complete comparison unavailable')
            else:
                archive = recording/f'{sequence:06d}.npz'
                depth, archive_hash, error = read_recorded_depth(archive)
                if error is not None or archive_hash != alternate['archive_sha256']:
                    raise ValueError(f'Recorded depth archive unavailable/changed at {sequence}: {error}')
                row.update(status='processed', input_archive_sha256=archive_hash,
                           original_rgb_sha256=alternate['rgb_sha256'])
                image_inputs.append(dict(path=archive.name, sha256=archive_hash,
                                         rgb_sha256=alternate['rgb_sha256'], status='processed'))
                g = original['geometry']
                intrinsics = rs.intrinsics()
                for field in ('width','height','fx','fy','ppx','ppy','coeffs'):
                    setattr(intrinsics, field, g[field])
                intrinsics.model = getattr(rs.distortion, g['distortion'].split('.')[-1])
                for variant, probe in [('baseline',original['probe']),('holistic',alternate['probe'])]:
                    palm_mask = np.zeros(depth.shape, dtype=np.uint8)
                    wrist_mask = np.zeros(depth.shape, dtype=bool)
                    if probe['detected']:
                        uv = np.asarray(probe['pixels'])
                        cv2.fillConvexPoly(palm_mask,cv2.convexHull(uv[[0,5,9,13,17]].astype(np.int32)),1)
                        palm_mask = cv2.dilate(palm_mask,np.ones((7,7),np.uint8))
                        wrist_mask = surface.wrist_surface_support_mask(uv,depth.shape)
                    before = time.perf_counter()
                    fused, diagnostics = rgbd.fuse_frame(probe,depth,palm_mask.astype(bool),
                        lambda pixel,z: rs.rs2_deproject_pixel_to_point(intrinsics,pixel,z),
                        timestamp_s=row['timestamp_s'],frame_id=observation['frame_id'],
                        track_id=observation['track_id'] if variant=='baseline' else
                            'offline_holistic_body_right_unverified_identity',
                        aperture_calibration=g['aperture_calibration'],
                        position_method='wrist_surface_fit',wrist_support_mask=wrist_mask)
                    row['methods'][variant] = dict(detected=probe['detected'],observation=fused.as_dict(),
                        diagnostics=diagnostics,fusion_elapsed_s=time.perf_counter()-before)
            for variant in VARIANTS:
                if variant not in row['methods']:
                    row['methods'][variant] = dict(detected=False,observation=None,
                        diagnostics=dict(reason=row['status']),fusion_elapsed_s=None)
                item = row['methods'][variant]
                pair = stats[variant].add(sequence=sequence,timestamp_s=row['timestamp_s'],
                    detected=item['detected'],observation=item['observation'],diagnostics=item['diagnostics'],
                    elapsed_s=item['fusion_elapsed_s'],geometry_sha256=row['source_geometry_sha256'])
                if pair is not None:
                    pair.update(guard_limit_m=.015+1.5*pair['dt_s'],
                        guard_exceeded=pair['step_m']>.015+1.5*pair['dt_s'],
                        delta_xyz_m=(np.asarray(item['observation']['pose']['position_m'])-
                                     np.asarray(last[variant]['pose']['position_m'])).tolist())
                item['motion_from_previous'] = pair
                last[variant] = item['observation']
            if row['status'] == 'processed':
                baseline_obs = row['methods']['baseline']['observation']
                holistic_obs = row['methods']['holistic']['observation']
                row['comparison_kind'] = comparison_kind(observed(observation),observed(holistic_obs))
                row['baseline_replay_position_max_abs_m'] = None
                if observed(observation) and observed(baseline_obs):
                    row['baseline_replay_position_max_abs_m'] = float(np.max(np.abs(
                        np.asarray(observation['pose']['position_m'])-baseline_obs['pose']['position_m'])))
                row['both_valid_position_difference_m'] = None
                if observed(observation) and observed(holistic_obs):
                    row['both_valid_position_difference_m'] = math.dist(
                        observation['pose']['position_m'],holistic_obs['pose']['position_m'])
            destination.write(json.dumps(row,allow_nan=False)+'\n')
            outputs.append(row)
    if any(file_sha256(Path(path)) != sha for path,sha in code_hashes.items()):
        raise RuntimeError('Fusion code changed during the run')
    manifest = dict(schema='teleop.offline_holistic_depth_inputs.v1',
        recording=str(recording.resolve()),frames_jsonl_sha256=source_hash,
        holistic=str(holistic.resolve()),holistic_sha256=holistic_hash,
        holistic_manifest_sha256=file_sha256(manifest_path),code_sha256=code_hashes,
        numpy=np.__version__,opencv=cv2.__version__,surface_fit_version=surface.SURFACE_FIT_VERSION,
        pyrealsense2=importlib.metadata.version('pyrealsense2'),
        images=image_inputs,position_method='wrist_surface_fit',explicit_wrist_support_mask=True,
        note='Explicit saved Holistic probe override. No new inference or camera source. '
             'Original registered depth and SDK distortion/intrinsics, thresholds and estimator unchanged. '
             'Model orientation remains RGB-model evidence; observed position requires current depth support. '
             'Synthetic track name identifies an unverified body-right branch, not persistent person identity.')
    out.with_suffix('.inputs.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    summary = summarize(outputs)
    summary.update(schema='teleop.offline_holistic_depth_comparison.v1',
        finished_utc=datetime.now(timezone.utc).isoformat(),wall_elapsed_s=time.monotonic()-started,
        blocks={name:summarize(outputs[start:end+1]) for name,start,end in [
            ('distractor',9,164),('palm_down_label_flip',2435,2468),('long_no_hand',1550,1744)]},
        note='All attempts retained. Comparisons exclude initial capture errors and terminal unreadable RGB. '
             'Position changes and guard inputs include actual hand motion; they are not accuracy/jitter errors. '
             'This does not run a controller or validate physical following/identity/surface association.')
    out.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:summary[key] for key in ('counts','changed_sequences','baseline_replay_availability_mismatch',
        'baseline_replay_position_max_abs_m','both_valid_position_difference_m','wall_elapsed_s')},indent=2))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--holistic',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    run(args.recording,args.holistic,args.out)
