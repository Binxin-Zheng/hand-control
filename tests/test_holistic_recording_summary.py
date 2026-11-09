import importlib.util
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
spec=importlib.util.spec_from_file_location('holistic_recording_summary_test',
    Path(__file__).parents[1]/'tools/analyze_holistic_recording.py')
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_all_attempts_detector_losses_and_prior_pose_losses_are_separate():
    rows=[dict(sequence=0,status='capture_error_no_original_inference',recorded_probe=None)]
    for sequence,old,new,kind,pose_present in [
        (1,True,True,'both_selected',True),(2,False,True,'gained_selection',False),
        (3,True,False,'lost_selection',True),(4,False,False,'neither_selected',False)]:
        rows.append(dict(sequence=sequence,status='processed',recorded_probe=dict(detected=old),
            probe=dict(detected=new),comparison_kind=kind,recorded_pose_present=pose_present,
            probe_elapsed_s=.02,diagnostics=dict(inference_elapsed_s=.019),comparison={}))
    rows.append(dict(sequence=5,status='archive_unreadable',recorded_probe=dict(detected=True)))
    result=audit.summarize(rows)
    counts=result['counts']
    assert counts['attempts']==6 and counts['processed']==4
    assert counts['recorded_selected_all_attempts']==3
    assert counts['baseline_selected_readable']==counts['holistic_selected']==2
    assert counts['lost_selection']==counts['gained_selection']==1
    assert counts['baseline_pose_present_readable']==2
    assert counts['lost_selection_on_recorded_pose_present']==1
    assert result['selected_difference_sequences']==dict(gained_selection=[2],lost_selection=[3])
    assert result['inference_elapsed_s']['count']==4


def test_no_processed_frame_never_claims_a_successful_selection():
    result=audit.summarize([dict(sequence=0,status='capture_error_no_original_inference',recorded_probe=None)])
    assert result['counts']['attempts']==1
    assert result['counts']['holistic_selected']==0
    assert result['inference_elapsed_s']['count']==0
    assert result['inference_elapsed_s']['p50']is None
