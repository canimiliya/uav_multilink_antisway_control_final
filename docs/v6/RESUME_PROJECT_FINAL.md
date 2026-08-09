# Resume Project Final

## 中文简历版

构建并冻结五连杆无人机吊载抗摆 MuJoCo 基准，统一 PID、Full-State LQR、Task-LQR 与自研 SATC-OFMPC 的三维加速度权限；在 96 个未见 Holdout 样本上，自研方法保持 100% safety，并将相对 Full-LQR 的位置 RMSE 降低 29.10%，10,000 次配对 bootstrap 置信区间严格大于零。进一步预注册并适配两种 2026 论文控制方法，完整保留其 Development 负结果且未打开不合格方法的 Holdout。

## English resume version

Built a reproducible five-link UAV suspended-tool MuJoCo benchmark and a fair acceleration-level comparison across PID, two LQR baselines, and a self-developed SATC-OFMPC controller. Achieved a 29.10% position-RMSE reduction versus Full-LQR on a 96-sample unseen Holdout with 100% safety and a strictly positive 10,000-resample paired-bootstrap interval; preregistered and transparently reported two recent-paper adaptation failures without test-set tuning.

## Do not claim

Do not describe the result as real flight, a Strict all-metric win, or superiority over the original published Paper systems.
