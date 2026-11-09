"""Research-only, instrumented legacy Holistic; no camera or robot interfaces.

This is a separate body-right selection experiment, not the original Hands
detector. It reuses only the pinned detector's all-21 parsing/SVD/MANO math.
World landmarks are model-relative coordinates, never observed camera XYZ.
Imports do not construct an inference graph. Explicit construction starts one
Holistic graph; process submits each original RGB image exactly once.
"""
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reprobe_recorded_detector import DETECTOR_BLOB, detector_blob, file_sha256


MEDIAPIPE_VERSION = '0.10.14'
BINARY_GRAPH = 'mediapipe/modules/holistic_landmark/holistic_landmark_cpu.binarypb'
GRAPH_SHA256 = 'c1da3ea92cd0dfaeb35a1b3863a3a34202c6f4beffe410eff4a337569c808fbd'
SIDE_INPUTS = dict(model_complexity=1, smooth_landmarks=True,
    enable_segmentation=False, smooth_segmentation=True,
    refine_face_landmarks=False, use_prev_landmarks=True)
CALCULATOR_PARAMS = {
    'poselandmarkcpu__posedetectioncpu__TensorsToDetectionsCalculator.min_score_thresh': .8,
    'poselandmarkcpu__poselandmarkbyroicpu__tensorstoposelandmarksandsegmentation__ThresholdingCalculator.threshold': .8,
}
HAND_STREAMS = {}
for _side, _index in [('left', 1), ('right', 2)]:
    _prefix = f'handlandmarksleftandrightcpu__handlandmarksfromposecpu_{_index}__'
    _hand = _prefix+'handlandmarkcpu__'
    CALCULATOR_PARAMS[_hand+'ThresholdingCalculator.threshold'] = .8
    HAND_STREAMS[_side] = dict(
        image_landmarks=f'{_side}_hand_landmarks',
        world_landmarks=_hand+'hand_world_landmarks',
        presence_score=_hand+'hand_presence_score',
        presence=_hand+'hand_presence',
        handedness=_hand+'handedness',
        tracking_roi=f'handlandmarksleftandrightcpu__{_side}_hand_tracking_roi',
        pose_wrist_visibility=_prefix+'handvisibilityfromhandlandmarksfrompose__wrist_visibility_score',
        pose_hand_visible=_prefix+'hand_visibility')
OUTPUTS = ['pose_landmarks', 'pose_world_landmarks', 'pose_landmarks_roi']+[
    stream for streams in HAND_STREAMS.values() for stream in streams.values()]
MODEL_PATHS = [
    'modules/pose_landmark/pose_landmark_full.tflite',
    'modules/pose_detection/pose_detection.tflite',
    'modules/hand_landmark/hand_landmark_full.tflite',
    'modules/holistic_landmark/hand_recrop.tflite',
    'modules/face_landmark/face_landmark.tflite',
]


def graph_options():
    """Four explicit .8 gates; the pose-wrist visibility ROI gate stays at .1."""
    return dict(binary_graph_path=BINARY_GRAPH, side_inputs=dict(SIDE_INPUTS),
                calculator_params=dict(CALCULATOR_PARAMS), outputs=list(OUTPUTS))


def _optional_float(message, field):
    return float(getattr(message, field)) if message.HasField(field) else None


def serialize_landmarks(group, count):
    if group is None:
        return None
    if len(group.landmark) != count:
        raise ValueError(f'Expected all {count} landmarks')
    return [dict(x=float(p.x), y=float(p.y), z=float(p.z),
                 visibility=_optional_float(p, 'visibility'),
                 presence=_optional_float(p, 'presence')) for p in group.landmark]


def serialize_roi(roi):
    if roi is None:
        return None
    return {field: _optional_float(roi, field)
            for field in ('x_center', 'y_center', 'width', 'height', 'rotation')}


def capture_raw(results):
    """No caching: absent packets from this process() remain explicitly absent."""
    raw = dict(pose_landmarks=serialize_landmarks(results.pose_landmarks, 33),
               pose_world_landmarks=serialize_landmarks(results.pose_world_landmarks, 33),
               pose_roi=serialize_roi(results.pose_landmarks_roi), hands={})
    for side, streams in HAND_STREAMS.items():
        packets = {key: getattr(results, stream) for key, stream in streams.items()}
        handedness = packets['handedness']
        raw['hands'][side] = dict(
            image_landmarks=serialize_landmarks(packets['image_landmarks'], 21),
            world_landmarks=serialize_landmarks(packets['world_landmarks'], 21),
            handedness=None if handedness is None else [
                dict(index=int(c.index), label=c.label, score=float(c.score))
                for c in handedness.classification],
            tracking_roi=serialize_roi(packets['tracking_roi']),
            **{key: None if packets[key] is None else float(packets[key])
               for key in ('presence_score', 'pose_wrist_visibility')},
            **{key: None if packets[key] is None else bool(packets[key])
               for key in ('presence', 'pose_hand_visible')})
    json.dumps(raw, allow_nan=False)
    return raw


def adapt_body_right(results, raw, upstream, image_shape, sequence):
    """Use current body-right outputs; do not apply standalone raw-Left inversion."""
    hand = raw['hands']['right']
    if raw['pose_landmarks'] is None:
        reason = 'current_body_unavailable'
    elif hand['presence'] is not True:
        reason = 'current_right_hand_presence_unavailable_or_rejected'
    elif hand['image_landmarks'] is None or hand['world_landmarks'] is None:
        reason = 'current_right_hand_landmarks_unavailable'
    else:
        reason = 'current_body_right_selected'
    selected = reason == 'current_body_right_selected'
    probe = dict(schema='upstream.probe.v2', sequence=sequence, detected=selected,
        image_width=int(image_shape[1]), image_height=int(image_shape[0]),
        local_points_mano=None, model_wrist_rotation=None,
        operator2mano=upstream.OPERATOR2MANO_RIGHT.tolist(), pixels=None)
    if selected:
        if hand['presence_score'] is None or hand['presence_score'] < .8:
            raise ValueError('Presence flag contradicts the configured .8 hand gate')
        detector = upstream.SingleHandDetector
        world = getattr(results, HAND_STREAMS['right']['world_landmarks'])
        image = getattr(results, HAND_STREAMS['right']['image_landmarks'])
        centered = detector.parse_keypoint_3d(world)
        centered = centered-centered[0:1, :]
        rotation = detector.estimate_frame_from_hand_points(centered)
        local = centered @ rotation @ upstream.OPERATOR2MANO_RIGHT
        probe.update(local_points_mano=local.tolist(), model_wrist_rotation=rotation.tolist(),
                     pixels=detector.parse_keypoint_2d(image, image_shape[:2]).tolist())
    diagnostics = dict(selection_reason=reason, selected_body_side='right' if selected else None,
        selection_policy='current pose plus current right-hand image/world packets and accepted presence',
        raw_handedness_used_for_selection=False, persistent_person_identity=False,
        global_translation_available=False, hand_world_provenance='rgb_model_relative',
        pose_world_provenance='rgb_model_hip_relative', original_detector_detect_called=False)
    output = dict(probe=probe, raw=raw, diagnostics=diagnostics)
    json.dumps(output, allow_nan=False)
    return output


class HolisticProbe:
    """Explicit offline graph owner; constructor never creates a Hands instance."""
    def __init__(self, detector_repo, hand='Right'):
        if hand != 'Right':
            raise ValueError('This isolated experiment explicitly selects anatomical body Right')
        detector_path = Path(detector_repo)/'example/vector_retargeting/single_hand_detector.py'
        if detector_blob(detector_path.read_bytes()) != DETECTOR_BLOB:
            raise ValueError('Upstream detector differs from the pinned audited blob')
        import mediapipe as mp
        # Register the original calculator option extensions; do not instantiate
        # Holistic(), whose constructor includes a model-download helper.
        from mediapipe.python.solutions import holistic
        from mediapipe.python.solution_base import SolutionBase
        if mp.__version__ != MEDIAPIPE_VERSION:
            raise ValueError('Instrumented stream names require MediaPipe '+MEDIAPIPE_VERSION)
        package = Path(mp.__file__).parent
        graph = package/'modules/holistic_landmark/holistic_landmark_cpu.binarypb'
        if file_sha256(graph) != GRAPH_SHA256:
            raise ValueError('Installed Holistic graph differs from the audited graph')
        files = [Path(__file__), detector_path, Path(holistic.__file__),
                 package/'python/solution_base.py', graph]+[package/p for p in MODEL_PATHS]
        for path in files:
            if not path.is_file():
                raise FileNotFoundError(f'Required installed asset missing; no downloads: {path}')
        spec = importlib.util.spec_from_file_location('pinned_holistic_math', detector_path)
        self._upstream = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self._upstream)
        self.manifest = dict(mediapipe=mp.__version__, numpy=np.__version__,
            graph_options=graph_options(), detector_blob=DETECTOR_BLOB,
            files=[dict(path=str(path.resolve()), sha256=file_sha256(path)) for path in files],
            anatomical_body_side='right', pose_wrist_visibility_gate_unchanged=.1,
            raw_streams=HAND_STREAMS, simulated_process_interval_us=33333,
            note='Separate Holistic ROI/selection experiment; original upstream Hands baseline unchanged. '
                 'Original all-21 parsing, SVD and right MANO math reused without detect() or Hands inference. '
                 'Original unmirrored RGB required. Pose smoothing and previous ROI tracking remain enabled. '
                 'Presence fields are retained instead of cleared by the public Holistic wrapper. '
                 'Body side is not persistent person identity; model world is not measured global XYZ.')
        self._graph = SolutionBase(**graph_options())
        self.calls = 0

    def process(self, rgb, sequence):
        if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8
                or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) <= 0):
            raise ValueError('Expected original uint8 HxWx3 RGB image')
        if type(sequence) is not int or sequence < 0:
            raise ValueError('Expected a nonnegative source sequence')
        started = time.monotonic()
        self.calls += 1
        results = self._graph.process(input_data={'image': rgb})
        inference_elapsed = time.monotonic()-started
        raw = capture_raw(results)
        output = adapt_body_right(results, raw, self._upstream, rgb.shape, sequence)
        output['diagnostics']['inference_elapsed_s'] = inference_elapsed
        output['diagnostics']['process_calls'] = self.calls
        return output

    def close(self):
        self._graph.close()
