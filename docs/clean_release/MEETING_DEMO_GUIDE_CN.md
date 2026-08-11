# 三任务会议演示指南

运行 `python scripts/meeting_demo/run_all.py` 会在冻结五连杆模型上执行三项功能演示，并为 Full-LQR `full_lqr_048` 与 SATC `satc_b_027` 生成 CSV、metrics、PNG 和 GIF。演示只使用 Development 语义，不打开 Holdout，也不调参。

- T1：剪钳初始位置保持 1 s，随后按固定目标 `(+0.30,+0.20,+0.15) m` 做 4 s minimum-jerk 移动，再保持。
- T2：XYZ 目标保持初始目标，初始关节角固定为 `(+5,-4,+3,-2,+1)°`，观察初始摆动衰减。
- T3：初始目标保持，`t=4 s` 起施加正式分布式世界坐标 `+3.0 m/s X` 风，观察恢复。

会议重点看 `meeting_summary.png`、`MEETING_METRICS.md` 与三项任务目录下的轨迹图和 GIF。Heading 当前状态是 `NOT_VERIFIED`：Udaan 的 `desired_att` 接受目标姿态三元组，但其 `compute()` 未消费第 0 项旋转，因此本次不把 Heading 写入主线结论。

只重画已有 CSV：

```text
python scripts/meeting_demo/run_all.py --plots-only
```
