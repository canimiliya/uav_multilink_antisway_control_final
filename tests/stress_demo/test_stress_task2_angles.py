import numpy as np

def test_stress_task2_angles():
    assert np.allclose(np.deg2rad([20,-16,12,-8,4]), [0.34906585,-0.27925268,0.20943951,-0.13962634,0.06981317])

