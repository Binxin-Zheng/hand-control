"""Offline body-associated Holistic comparison; no runtime or depth/control changes."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from analyze_pose_association import read_rows
from reprobe_recorded_detector import ARCHIVE_ERRORS, compare_probe, distribution, file_sha256


def summarize(rows):
    counts = Counter(row['status'] for row in rows)
    counts['attempts'] = len(rows)
    processed = [row for row in rows if row['status']=='processed']
    counts['recorded_selected_all_attempts'] = sum((row.get('recorded_probe') or {}).get('detected', False) for row in rows)
    counts['baseline_selected_readable'] = sum(row['recorded_probe']['detected'] for row in processed)
    counts['holistic_selected'] = sum(row['probe']['detected'] for row in processed)
    for name in ('both_selected', 'gained_selection', 'lost_selection', 'neither_selected'):
        counts[name] = sum(row['comparison_kind']==name for row in processed)
    counts['baseline_pose_present_readable'] = sum(row['recorded_pose_present'] for row in processed)
    counts['lost_selection_on_recorded_pose_present'] = sum(
        row['recorded_pose_present'] and not row['probe']['detected'] for row in processed)
    return dict(counts=dict(counts),
        selected_difference_sequences={name:[row['sequence'] for row in processed if row['comparison_kind']==name]
            for name in ('gained_selection', 'lost_selection')},
        probe_elapsed_s=distribution([row['probe_elapsed_s'] for row in processed]),
        inference_elapsed_s=distribution([row['diagnostics']['inference_elapsed_s'] for row in processed
            if 'inference_elapsed_s' in row['diagnostics']]),
        coordinate_differences={field: distribution([row['comparison'][field]['max_abs'] for row in processed
            if field in row['comparison']]) for field in ('pixels','local_points_mano','model_wrist_rotation')})


def run(recording, baseline, out, detector_repo):
    recording, baseline, out, detector_repo = map(Path, (recording, baseline, out, detector_repo))
    source_path = recording/'frames.jsonl'
    source, previous = map(read_rows, (source_path, baseline))
    if len(source) != len(previous):
        raise ValueError('Baseline must retain every recorded attempt')
    for path in (out,out.with_suffix('.summary.json'),out.with_suffix('.inputs.json')):
        if path.exists():
            raise FileExistsError(path)
    source_hash = file_sha256(source_path)
    original_manifest = json.loads(baseline.with_suffix('.inputs.json').read_text())
    if original_manifest['frames_jsonl_sha256'] != source_hash:
        raise ValueError('Baseline refers to another source recording')
    hands = {row['geometry']['selected_hand'] for row in source if 'error' not in row}
    if hands != {'Right'}:
        raise ValueError('This explicit experiment requires the recorded body-right hand')
    for sequence,(original,prior) in enumerate(zip(source,previous)):
        if 'error' not in original and prior['recorded_probe'] != original['probe']:
            raise ValueError(f'Baseline original probe differs at {sequence}')
    out.parent.mkdir(parents=True, exist_ok=True)
    import cv2
    from holistic_probe_adapter import HolisticProbe
    probe = HolisticProbe(detector_repo, hand='Right')
    outputs, images = [], []
    abort = None
    process_calls = 0
    started = time.monotonic()
    try:
        with out.open('x') as destination:
            for sequence,original in enumerate(source):
                observation = original.get('observation', {})
                row = dict(sequence=sequence, timestamp_s=observation.get('timestamp_s'),
                    recorded_probe=original.get('probe'), recorded_pose_present=observation.get('pose') is not None,
                    recorded_position_evidence=observation.get('position_evidence'))
                if 'error' in original:
                    row.update(status='capture_error_no_original_inference', error=original['error'])
                elif abort is not None:
                    row.update(status='not_processed_after_gap', error=abort)
                else:
                    path = recording/f'{sequence:06d}.npz'
                    archive_hash = None
                    try:
                        archive_hash = file_sha256(path)
                        with np.load(path) as archive:
                            bgr = archive['color_bgr']
                        saved = original['probe']
                        if bgr.dtype != np.uint8 or bgr.shape != (saved['image_height'],saved['image_width'],3):
                            raise ValueError('Image dimensions/type differ from original detector input')
                    except ARCHIVE_ERRORS as error:
                        abort = dict(sequence=sequence,type=type(error).__name__,message=str(error))
                        row.update(status='archive_unreadable',error=abort,recorded_observation=observation)
                        images.append(dict(path=path.name,sha256=archive_hash,error=abort))
                    else:
                        rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
                        rgb_hash = hashlib.sha256(rgb.tobytes()).hexdigest()
                        images.append(dict(path=path.name,sha256=archive_hash,rgb_sha256=rgb_hash))
                        try:
                            prior = previous[sequence]
                            if prior['status'] != 'reprocessed':
                                raise ValueError('Readable frame has no completed baseline inference')
                            if archive_hash != prior['input_archive_sha256'] or rgb_hash != prior['rgb_sha256']:
                                raise ValueError('RGB/archive differs from baseline inference input')
                            before = time.monotonic()
                            process_calls += 1
                            result = probe.process(rgb,sequence)
                            elapsed = time.monotonic()-before
                            comparison = compare_probe(original['probe'],result['probe'])
                            old,new = original['probe']['detected'],result['probe']['detected']
                            kind = ('both_selected' if old else 'gained_selection') if new else (
                                'lost_selection' if old else 'neither_selected')
                            candidate = dict(row,**result,status='processed',comparison=comparison,
                                comparison_kind=kind,probe_elapsed_s=elapsed,
                                archive_sha256=archive_hash,rgb_sha256=rgb_hash)
                            json.dumps(candidate,allow_nan=False)
                            row = candidate
                        except Exception as error:
                            abort = dict(sequence=sequence,type=type(error).__name__,message=str(error))
                            row.update(status='inference_or_analysis_error',error=abort)
                destination.write(json.dumps(row,allow_nan=False)+'\n')
                outputs.append(row)
    finally:
        probe.close()
    manifest = dict(recording=str(recording.resolve()),frames_jsonl_sha256=source_hash,
        baseline=str(baseline.resolve()),baseline_sha256=file_sha256(baseline),
        script_sha256=file_sha256(Path(__file__)),adapter=probe.manifest,images=images,
        note='Explicit offline body-right identity variant, not original handedness selection. Original RGB, original21-point orientation/MANO math, bothhand0.8 gates. No depth fusion, hand-pose prediction counted as global observation, runtime modification, devices, or installs.')
    out.with_suffix('.inputs.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    summary = summarize(outputs)
    summary.update(schema='teleop.offline_holistic_comparison.v1',process_calls=process_calls,
        finished_utc=datetime.now(timezone.utc).isoformat(),wall_elapsed_s=time.monotonic()-started,
        abort_reason=abort,continuity='stopped_after_gap' if any(r['status']=='not_processed_after_gap' for r in outputs) else
            'terminal_unreadable_archive' if abort and outputs[-1]['status']=='archive_unreadable' else
            'terminal_analysis_error' if abort else 'complete_recording',
        blocks={name:summarize(outputs[start:end+1]) for name,start,end in [
            ('distractor',9,164),('palm_down_label_flip',2435,2468),('long_no_hand',1550,1744)]},
        note='Selection means current body-associated21image+world output, not observedglobalXYZ. All baselinevalid losses retained. No threshold tuning or runtime promotion.')
    out.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2,allow_nan=False))
    if any(r['status'] in ('not_processed_after_gap','inference_or_analysis_error') for r in outputs):
        raise RuntimeError('Incomplete offline comparison; retained failures in output')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--detector-repo',type=Path,default=Path('/home/bx/repos/dex-retargeting'))
    args=parser.parse_args()
    run(args.recording,args.baseline,args.out,args.detector_repo)
