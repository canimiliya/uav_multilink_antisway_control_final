"""Run the P3-R1F T1/T2 boundary search and final showcase freeze."""
from uav_sway.demo.boundary_runner import run_all


if __name__ == "__main__":
    import json
    print(json.dumps(run_all(), indent=2, default=str))
