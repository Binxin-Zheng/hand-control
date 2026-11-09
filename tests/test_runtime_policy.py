import importlib.util
from pathlib import Path
import numpy as np
import pytest
from anyteleop_so101.runtime.driver_policy import ARM_NAMES, Calibration, CommandLease
from anyteleop_so101.runtime.control import GuardedController
from anyteleop_so101.runtime.model import validate_model
from conftest import obs, state


def calibration():
    return {n:dict(id=i+1,zero_raw=2048 if n!='gripper' else 1000,
                   raw_per_unit=-500 if i==1 else 500,lower=-1 if n!='gripper' else 0,
                   upper=1,raw_min=0,raw_max=4095) for i,n in enumerate((*ARM_NAMES,'gripper'))}


def packet(seq, now, mode='active', **kwargs):
    return dict(sequence=seq,deadline_s=now+.08,mode=mode,q=kwargs.get('q',[0.]*5),aperture=kwargs.get('aperture',.5))


def test_calibration_units_sign_and_endpoints():
    c=Calibration(calibration())
    for n in (*ARM_NAMES,'gripper'):
        assert c.from_raw(n,c.to_raw(n,.4)) == pytest.approx(.4)
    assert c.to_raw('shoulder_lift',.2) < c.to_raw('shoulder_lift',0)
    with pytest.raises(ValueError): c.to_raw('shoulder_pan',2)
    with pytest.raises(ValueError): c.from_raw('shoulder_pan',float('nan'))


@pytest.mark.parametrize('field,value',[('raw_per_unit',0),('zero_raw',None),('raw_max',5000),('id',True)])
def test_bad_calibration_fails_before_connect(field,value):
    c=calibration(); c['shoulder_pan'][field]=value
    with pytest.raises(ValueError): Calibration(c)


def test_expiry_requires_explicit_release_not_new_commands():
    lease=CommandLease(); measured=dict(q=[0.]*5,aperture=.5)
    with pytest.raises(ValueError): lease.accept(packet(0,1),1,measured)
    lease.accept(packet(1,1,'release'),1,measured)
    assert lease.accept(packet(2,1),1,measured) is not None
    assert lease.poll(1.081)
    with pytest.raises(ValueError): lease.accept(packet(3,1.09),1.09,measured)
    lease.accept(packet(4,1.1,'release'),1.1,measured)
    assert lease.accept(packet(5,1.11),1.11,measured)


def test_arrival_does_not_hide_expired_lease():
    lease=CommandLease(); measured=dict(q=[0.]*5,aperture=.5)
    lease.accept(packet(0,1,'release'),1,measured); lease.accept(packet(1,1),1,measured)
    with pytest.raises(ValueError): lease.accept(packet(2,1.2),1.2,measured)


def test_estop_cannot_be_cleared_by_release():
    lease=CommandLease(); measured=dict(q=[0.]*5,aperture=.5)
    lease.accept(packet(0,1,'estop'),1,measured)
    with pytest.raises(ValueError): lease.accept(packet(1,1,'release'),1,measured)
    lease.accept(packet(2,1,'reset'),1,measured)
    with pytest.raises(ValueError): lease.accept(packet(3,1),1,measured)
    lease.accept(packet(4,1,'release'),1,measured)
    assert lease.accept(packet(5,1),1,measured)


@pytest.mark.parametrize('change',[{'deadline_s':.9},{'deadline_s':10},{'q':[1.]*5},{'aperture':1.}, {'sequence':False}])
def test_driver_rejects_unsafe_envelopes(change):
    lease=CommandLease(); measured=dict(q=[0.]*5,aperture=.5)
    lease.accept(packet(0,1,'release'),1,measured)
    bad=packet(1,1); bad.update(change)
    with pytest.raises(ValueError): lease.accept(bad,1,measured)


def test_collision_stop_latches_and_requires_release(session,home_q):
    class Guard:
        clear=False
        def check_segment(self,*args): return self.clear
    guard=Guard(); control=GuardedController(session,guard)
    d=control.tick(obs(),state(home_q),now_s=1.,dt=.01,clutch=True)
    assert d.reason=='collision_requires_reengage' and d.q_command is None
    guard.clear=True
    d=control.tick(obs(1.01,2),state(home_q,1.01),now_s=1.01,dt=.01,clutch=True)
    assert d.q_command is None
    control.tick(obs(1.02,3),state(home_q,1.02),now_s=1.02,dt=.01,clutch=False)
    d=control.tick(obs(1.03,4),state(home_q,1.03),now_s=1.03,dt=.01,clutch=True)
    np.testing.assert_array_equal(d.q_command,home_q)


def test_kinematic_extract_cannot_be_used_for_collision():
    with pytest.raises(ValueError): validate_model(Path(__file__).parents[1]/'assets/so101_kinematics_only.urdf')


def test_hardware_stop_holds_current_arm_and_never_opens_gripper():
    path=Path(__file__).parents[1]/'tools/robot_worker.py'
    spec=importlib.util.spec_from_file_location('worker_test',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    port=module.RawFeetechPort({'joints':calibration()})
    class Bus:
        def sync_read(self,name,**kwargs):
            assert name=='Present_Position' and set(kwargs['motors'])==set(ARM_NAMES)
            return {n:2048 for n in ARM_NAMES}
        def sync_write(self,name,values,**kwargs):
            assert name=='Goal_Position' and kwargs['normalize'] is False
            assert values=={n:2048 for n in ARM_NAMES}
    port.bus=Bus(); port.stop()
    with pytest.raises(ValueError): module.validate_acceptance({'joints':calibration()})


def test_lab_demo_accepts_software_stop_without_physical_estop():
    path=Path(__file__).parents[1]/'tools/robot_worker.py'
    spec=importlib.util.spec_from_file_location('worker_demo_policy',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    acceptance=dict(joint_calibration=True,
                    tcp=True, collision_scene=True, command_watchdog=True,
                    software_stop=True, hold_stop=True, camera_geometry=True,
                    serial_timeout=True)
    joints=calibration()
    expected={name:dict(id=item['id'],drive_mode=0,homing_offset=0,
                        range_min=item['raw_min'],range_max=item['raw_max'])
              for name,item in joints.items()}
    config=dict(port='/dev/test_not_opened', joints=joints, acceptance=acceptance,
                expected_servo_calibration=expected,
                gripper_model_calibration=dict(zero_raw=1000,raw_per_rad=500,angle_range_rad=[0,1]))
    module.validate_acceptance(config)  # Validation only; never opens a port.
    acceptance['software_stop']=False
    acceptance['physical_estop']=True
    with pytest.raises(ValueError): module.validate_acceptance(config)
