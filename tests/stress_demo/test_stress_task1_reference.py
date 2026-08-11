import numpy as np
from uav_sway.demo.stress_runner import reference

def test_stress_task1_reference_window():
    p=np.array([0.,0.,0.]); target=p+np.array([2.,1.7,4.5])
    assert np.allclose(reference(p,target,3).position_world,p)
    assert np.allclose(reference(p,target,23).position_world,target)

