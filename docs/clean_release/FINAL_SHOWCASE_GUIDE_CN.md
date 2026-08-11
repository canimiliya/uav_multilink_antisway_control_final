# Final showcase guide (CN)

P3-R1D 是 failure-boundary evidence：4 s 大转场、初始 20° 摆动和 10 m/s 持续风超出当前任务控制能力。

P3-R1E 是 controlled stress showcase：固定模型和固定控制器，6 s quintic 转场，T2 在转场中加入 5 m/s 世界 +X 风，T3 扫描 3--10 m/s 世界 +X 风。

## 会议解读

- T1 分类：LQR `CONTROLLED_SHOWCASE`，SATC `CONTROLLED_SHOWCASE`。
- T2 分类：LQR `NOT_RECOVERED`，SATC `NOT_RECOVERED`。
- 风包络：LQR 最大可恢复风速 `3.0` m/s；SATC 最大可恢复风速 `5.0` m/s。
- 所有数字都属于 `DEMO CAPABILITY ENVELOPE ONLY`，不是 Holdout 或新的论文 claim。

Y/XY30 历史结果仍保留，但由于五个 y-axis hinge 的平面链模型，不作为核心多连杆消摆结论。
