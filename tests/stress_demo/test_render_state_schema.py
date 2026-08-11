from pathlib import Path
import numpy as np

def test_render_state_schema():
    p=Path("outputs/meeting_demo_stress_v2/T1/full_lqr_048/render_states.npz")
    if p.exists():
        x=np.load(p); assert set(x.files)=={"time","qpos"}; assert x["qpos"].shape[1]==12

