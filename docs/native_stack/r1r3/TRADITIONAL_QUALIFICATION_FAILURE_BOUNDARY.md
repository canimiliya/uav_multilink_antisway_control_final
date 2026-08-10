# Traditional Qualification Failure Boundary

## 已证实

在冻结的 native-stack benchmark v1.2-governance 上，48 个 PID 配置与 48 个 Full-LQR/LQI 配置经过两轮 42-case 预筛，并各有 12 个候选完成 authoritative 200-case Development。最佳 PID 与最佳 LQI 均保持 100% all-case Safety，但 nominal success 分别止于 47% 与 52%。

最稳定的结构性缺口是 moderate wind：最佳 PID 为 14.7%，最佳 LQI 为 5.88%，均远低于 50% 子门槛。提高反馈/积分权限会降低 calm/stochastic 或 Safety，未形成同时满足所有子门槛的 classical servo。

## 没有证实

- 没有证据表明 benchmark、mission、wind、reference 或 success definition 有 bug。
- 没有证据表明 Governance v2 应再次修改。
- 没有执行 Task-LQR、SATC、Paper 或 Holdout，不能对这些未执行阶段作性能结论。
- 没有把旧 R1R1/R1R2 结果追认为 PASS，也没有覆盖其历史状态。

## 科研边界

本轮只证明：在预冻结预算与已授权 classical PID/LQI architecture 内，没有获得 Governance-v2 competent Traditional。它不证明所有可能的传统控制器在该 plant 上都不可能合格，也不授权后续自动扩展预算或修改平台。

