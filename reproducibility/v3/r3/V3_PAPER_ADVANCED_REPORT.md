# V3 Paper-Advanced Report

## 结论

V3-R3 已按冻结协议完成，但没有 Paper candidate 同时通过全部 Development 门槛。路线以 `CLOSED_WITH_NO_V3_PAPER_ADVANCED_WIN` 关闭；没有换论文、追加搜索、修改 Traditional/Self，也没有访问 Holdout。

## 论文与适配

- 论文：*Modeling and Control for UAV with Off-center Slung Load*（Lv et al., 2026, arXiv preprint）
- 适配名：`LV2026-SPACC-ADAPTED-3D`
- 声明：adapted，非 exact reproduction
- Protocol freeze：`39a01592429897df943f306ec61c70deedabad3e`
- 核心保留：以悬挂点三维加速度调节 alpha/beta 摆动；五连杆用 cutter-tip 到 UAV 的等效摆向量表示。
- 冻结接口：输出 `[ax, ay, az]`，统一经过 `±2 m/s²` 幅值和 `0.25 m/s²/update` slew，再进入同一 geometric inner loop。

## 搜索

- Round A：48 个唯一候选 × 14 Development core = 672 cases。
- Round B：Top-6 原参数 × 75 Development = 450 cases。
- Refinement：0；GPU：未使用；Holdout：未执行。

## 最佳 Near-Miss

`paper_a_001`：75/75 safe，55/75 success（0.7333），position RMSE `0.084838475 m`，acquisition `5.230 s`，ramp peak/steady `0.302746414/0.140320052 m`，runtime P95 `0.2430 ms`。

它通过 safety、success、ramp、runtime，但 position 比硬门槛高 `0.001046680 m`，acquisition 比硬门槛慢 `0.34225 s`，因此 `WIN_LEVEL = NO_WIN`。

相对 Primary Full-LQR，position 改善 `3.813%`，ramp peak 改善 `7.125%`，ramp steady 改善 `6.687%`；但 acquisition 恶化 `12.352%`。相对冻结 Self，Paper 在 success、position、acquisition 和 ramp 上均更弱。

## 消融与 Holdout

没有 candidate 通过 Development gate，因此没有 Paper freeze；按“先冻结、后消融”的合同，消融未执行。Holdout manifest、3000-3019 seeds、2.0/3.5 m/s 风和 3.5 m/s ramp 均未访问。
