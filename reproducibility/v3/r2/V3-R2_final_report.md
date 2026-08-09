# V3-R2 Self-Advanced 最终报告

## 结论

V3-R2 已在冻结的 75 样本 Development benchmark 上完成。`self_a_034` 被冻结为 3D-DR-TSRMPC Self controller，并达到 `STRICT_ALL_METRIC_WIN_DEVELOPMENT`。该结论只适用于 Development；V3 Holdout 与 Paper-Advanced 均未启动。

## 冻结方法

- 方法：3D Dynamic-Residual Task-Space Residual MPC
- 状态：冻结的 20D V3 state
- 控制输入：世界坐标系 `[ax, ay, az]`
- backbone：冻结的 `task_lqr_009`
- horizon：8 个 outer updates
- residual estimator：`beta=0.3`、component limit `0.2`
- task position weight：`40.0`
- 物理约束：各轴加速度 `±2 m/s²`，各轴 slew `0.25 m/s²/update`
- Self 冻结提交：`58b6544ab6a123f08d1cf04f3b9a7b38a40647e5`

## Development 结果

| 指标 | Primary Traditional `full_lqr_048` | Self `self_a_034` | 相对变化 |
|---|---:|---:|---:|
| Safety | 100% | 100% | 不劣化 |
| Success | 72% | 100% | +28 percentage points |
| 3D position RMSE | 0.0882019 m | 0.0605289 m | 改善 31.37% |
| Acquisition median | 4.655 s | 2.265 s | 改善 51.34% |
| Ramp peak | 0.325972 m | 0.221678 m | 改善 31.99% |
| Ramp steady | 0.150375 m | 0.0276913 m | 改善 81.59% |
| Orientation RMSE | 0.615148 deg | 0.362535 deg | 改善 41.06% |
| Acceleration effort | 0.622482 | 0.538631 | 改善 13.47% |
| Solve P95 | — | 1.1014 ms | `< 50 ms` PASS |

所有冻结数值 gate 均通过；75 个 sample_id 与 Primary Traditional 完成精确配对。Development 上未执行正式 10,000 次 bootstrap，正式统计保留给未来冻结 Holdout。

## 冻结后消融

| 结构 | Safety | Success | Position RMSE | Acquisition median |
|---|---:|---:|---:|---:|
| backbone only | 100% | 58.67% | 0.100161 m | 2.170 s |
| predictive only | 100% | 52.00% | 0.110334 m | 2.255 s |
| residual only | 100% | 33.33% | 0.507847 m | 2.145 s |
| full method | 100% | 100% | 0.0605289 m | 2.265 s |

消融表明，单独的 predictive 或 residual 分支都不能复制完整性能；冻结的 full method 依赖预测优化与动态残差稳态补偿协同工作。`residual only` 的 limiter mismatch 很大，只作为负向机制证据，不构成可部署控制器。

## 边界审计

- 三个 Traditional controllers 未修改。
- MuJoCo plant、A3/B3、C_task_v3、physics、wind physics、Development/Holdout banks、门槛与公共控制约束未修改。
- 无 future wind truth、future target leakage、fallback 或 Traditional override。
- Advanced-Paper 未启动。
- Holdout 未读取、未执行。
- 代表性 Development 证据包含 4 份原始 trace CSV、1 张汇总曲线和 SHA-256 visual manifest。

## 最终状态

`V3_SELF_ADVANCED_FROZEN`

下一阶段只能由项目负责人另行授权；本轮不进入 Paper-Advanced 或 Holdout。
