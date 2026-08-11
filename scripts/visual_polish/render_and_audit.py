"""Run the sequential P3-R1G visual polish and controller lineup audit."""

from uav_sway.demo.visual_polish import run_all


if __name__ == "__main__":
    import json

    print(json.dumps(run_all(), indent=2))
