from pathlib import Path
import csv
import json


def test_attitude_logging_audit_declares_fixed_when_artifact_exists():
    path = Path(__file__).resolve().parents[2] / "artifacts/meeting_demo_extreme_v3/attitude_logging_audit.json"
    if not path.exists():
        return
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["ATTITUDE_LOGGING_FIXED"] is True


def test_csv_schema_includes_attitude_and_wrench_fields_when_artifacts_exist():
    root = Path(__file__).resolve().parents[2] / "outputs/meeting_demo_extreme_v3"
    csvs = list(root.glob("**/run.csv"))
    if not csvs:
        return
    with csvs[0].open(encoding="utf-8", newline="") as handle:
        fields = next(csv.reader(handle))
    for name in ("uav_roll_deg", "uav_pitch_deg", "uav_yaw_deg", "requested_thrust_N", "applied_thrust_N", "thrust_saturated"):
        assert name in fields
