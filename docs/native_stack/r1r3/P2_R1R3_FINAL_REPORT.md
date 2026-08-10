# P2-R1R3 Governance-v2 Baseline Qualification Final Report

## 结论

本轮完成了授权范围内的 Native PID 与 Native Full-LQR/LQI 定向资格搜索，但没有得到任何 Governance-v2 competent Traditional。因此正式结果是：

`P2_GOVERNANCE_V2_TRADITIONAL_QUALIFICATION_FAILED`

项目保持 **84%**，没有推进到 SATC，也没有打开 Recent Paper 阶段。

这不是平台 BLOCK。Authoritative Runner、Governance v2、benchmark 语义和物理 plant 均未发现真实性缺陷；失败属于控制器在冻结资格门槛下的普通性能失败。

## 冻结边界

- Source tag：`native-stack-benchmark-v1.2-governance`
- Source head：`88c3aef081fabab44d174bc8afd234ae634af3da`
- Research branch：`research/p2-baselines-governance-v2`
- Protocol freeze：`ab289b2dda4813bf2f8cfc76019f82bfd801d2d0`
- Local-recovery freeze：`4118c4d57235ba646c585f1cd01e4f11f5e48c02`
- Traditional failure evidence freeze：`fd1530b9005646b1da6134673d91396e5327d003`

Governance v2 的所有数字保持不变。Development 仍是原 200 resolved cases；Holdout 仍是原 140 cases，未执行、未读取性能字段、authoritative run 为 0。

## 搜索执行

### Native PID

- 先验来源：R1R1 `native_pid_001` 与 R1R2 failure audit
- 初始 targeted configs：24
- local recovery configs：24
- 新配置总数：48，未超过总预算 64
- Stage A：两轮均使用同一冻结 42-case manifest
- full-200：初始 6 个 + local 6 个
- 最终保留诊断：`r1r3_pid_local_000`

`r1r3_pid_local_000` 精确复现了旧 `native_pid_001` 的控制参数和 Development 指标，说明 prior 导入与新 authoritative runner 路径一致；旧 R1R1 FAIL 结论没有被改写。

### Native Full-LQR/LQI

- architecture：causal 9-state translational CARE-LQI servo + frozen SO(3) physical-wrench layer
- 初始 targeted configs：24
- conservative local recovery configs：24
- Stage A：两轮均使用冻结 42-case manifest
- full-200：初始 6 个 + local 6 个
- 最终保留诊断：`r1r3_lqi_local_000`

初始 CARE 权限过高造成 Safety 下降；local recovery 将权重降至安全低权限邻域，恢复到 100% Safety，但 moderate-wind 与总体 nominal success 仍不足。

### Task-LQR

Task-LQR 是任务卡定义的可选第三支线。本轮 PID 与 Full-LQR/LQI 均未得到 competent method，最多再增加一个 Task-LQR 也无法满足 `COMPETENT_TRADITIONAL_COUNT >= 2`。因此没有消耗额外 24-config 可选预算，正式记录为 `NOT_RUN_OPTIONAL_AFTER_DECISIVE_FAILURE`。

## 最佳完整 200-case 结果

| 指标 | PID `r1r3_pid_local_000` | Full-LQR/LQI `r1r3_lqi_local_000` | Governance-v2 gate |
|---|---:|---:|---:|
| All safety | 1.000 | 1.000 | >= 0.98 |
| Catastrophic | 0 | 0 | <= 4 |
| Deadline miss | 0.000100 | 0.0000986 | <= 0.01 |
| Nominal success | 0.470 | 0.520 | >= 0.70 |
| Calm success | 0.735 | 0.912 | >= 0.50 |
| Moderate success | 0.147 | 0.0588 | >= 0.50 |
| Stochastic success | 0.531 | 0.594 | >= 0.50 |
| Nominal setpoint success | 0.420 | 0.520 | >= 0.60 |
| Nominal trajectory success | 0.520 | 0.520 | >= 0.60 |
| Nominal setpoint RMSE (m) | 0.755 | 0.758 | <= 1.25 |
| Nominal trajectory RMSE (m) | 0.336 | 0.345 | <= 1.50 |
| Challenge success | 0.080 | 0.110 | report only |
| Challenge RMSE (m) | 1.319 | 1.388 | report only |
| Strong P90 (m) | 2.344 | 2.527 | report only |

两种最佳方法都通过了全部安全、灾难、deadline 和 RMSE 门槛，但同时未通过 nominal overall、moderate wind 以及两类任务 success 门槛。失败不能通过选择另一条汇总口径消除。

## Gate 后动作

- `COMPETENT_TRADITIONAL_COUNT = 0`
- Primary Traditional：未冻结
- Traditional Envelope：未冻结
- SATC search：未执行（前置门槛不成立）
- SATC Advanced qualification：未执行
- Paper search / selection / implementation / performance：全部 `false`
- Holdout：未执行，未受损
- PASS annotated tag：未创建

根据任务卡第 24 节，本轮到此停止；是否结束 Native extension 或另行授权新路线，由 owner 决定。

## 项目回归

按测试目录隔离执行后，`tests/native_stack`、`tests/release` 与 `tests/v2` 至 `tests/v10` 共 `271 passed, 0 failed`。仅有 OSQP deprecation / pending-deprecation warnings。

一次单体 `pytest -q` 调用在无失败输出的情况下长期无 I/O，已停止该精确测试进程并保留为 harness limitation；相同测试集合随后按目录全部通过。该编排层停滞不改变 Traditional 性能失败结论，也不构成 benchmark truth bug。
