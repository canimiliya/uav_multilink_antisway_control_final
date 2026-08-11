from pathlib import Path
import numpy as np

def test_render_state_schema():
    paths = [Path("outputs/meeting_demo_stress_v2/T1/full_lqr_048/render_states.npz"), Path("outputs/meeting_demo_extreme_v3/T1/full_lqr_048/render_states.npz")]
    for p in paths:
        if p.exists():
            x=np.load(p); assert set(x.files)=={"time","qpos"}; assert x["qpos"].shape[1]==12
