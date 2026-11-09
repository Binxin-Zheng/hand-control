"""Pure summary tests; no SDK, inference, device or file replay."""
import importlib.util
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
spec=importlib.util.spec_from_file_location('holistic_depth_test',
    Path(__file__).parents[1]/'tools/analyze_holistic_depth.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def observation(sequence, valid):
    return dict(sequence=sequence,pose=dict(position_m=[sequence*.01,0.,0.]) if valid else None,
        position_evidence='rgbd_fused' if valid else 'unavailable',track_id='test',frame_id='camera')


def row(sequence,before,after,timestamp,status='processed'):
    record=dict(sequence=sequence,timestamp_s=timestamp,status=status,
        recorded_observation=observation(sequence,before),methods={},source_geometry_sha256='geometry',
        comparison_kind=audit.comparison_kind(before,after),baseline_replay_position_max_abs_m=0. if before else None,
        both_valid_position_difference_m=0. if before and after else None)
    for name,valid in [('baseline',before),('holistic',after)]:
        record['methods'][name]=dict(detected=valid,observation=observation(sequence,valid),
            diagnostics=dict(reason='observed' if valid else 'unavailable',position_reference='surface'),
            fusion_elapsed_s=.001,motion_from_previous=None)
    return record


def test_changed_position_counts_are_not_detector_selection_counts():
    rows=[row(0,True,True,0.),row(1,False,True,.1),row(2,True,False,.2),row(3,False,False,.3)]
    summary=audit.summarize(rows)
    assert summary['counts']['holistic_observed']==2
    assert summary['counts']['recorded_observed_readable']==2
    assert summary['changed_sequences']==dict(gained_position=[1],lost_position=[2])
    assert summary['baseline_replay_availability_mismatch']==[]


def test_unknown_times_and_terminal_failure_remain_in_full_spans_and_denominator():
    rows=[row(0,False,False,None,'capture_error_no_original_inference'),row(1,False,False,.1),
          row(2,True,True,.2),row(3,True,True,.3),row(4,True,False,.4,'holistic_input_unreadable')]
    rows[-1]['methods']['baseline']['observation']=None
    summary=audit.summarize(rows)
    assert summary['counts']['attempts']==5 and summary['counts']['compared_readable']==3
    assert summary['counts']['recorded_observed_all_attempts']==3
    assert summary['counts']['recorded_observed_readable']==2
    spans=summary['methods']['holistic']['availability_spans']
    assert [span['frames'] for span in spans]==[2,2,1]
    assert spans[0]['sample_span_s'] is None and spans[0]['missing_timestamps']==1
    assert spans[1]['sample_span_s']==pytest.approx(.1)
    assert spans[2]['first_sequence']==spans[2]['last_sequence']==4
    assert spans[2]['sample_span_s']==0


def test_predictions_never_count_as_observed_position():
    sample=observation(0,True);sample['position_evidence']='predicted'
    assert not audit.observed(sample)
    assert not audit.observed(None)


def test_block_summary_does_not_count_pair_from_outside_block():
    first=row(9,True,True,1.)
    first['methods']['holistic']['motion_from_previous']=dict(previous_sequence=8,dt_s=.2,
        step_m=.4,speed_m_s=2.,guard_limit_m=.315,guard_exceeded=True,delta_xyz_m=[.4,0.,0.])
    assert audit.summarize([first])['methods']['holistic']['adjacent_position_guard']['pairs']==0


def test_hash_identifies_explicit_probe_override_canonically():
    assert audit.digest_json(dict(a=1,b=2))==audit.digest_json(dict(b=2,a=1))
    assert audit.digest_json(dict(a=1))!=audit.digest_json(dict(a=2))
