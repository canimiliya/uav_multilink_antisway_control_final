# Traditional competence gate audit

## Origin of 70%

Competence v1 was correctly frozen before P2-R1R1 performance. Its rationale cites the 2 m mission geometry, 0.5 m endpoint tolerance, SafetyV2, and real-time periods. However, neither the contract nor its introducing commit contains an equation, engineering acceptance requirement, or comparison that uniquely derives 70% instead of 50%, 60%, or 80%.

The formal classification is therefore:

- `PHYSICALLY_DERIVED_THRESHOLD=false`
- `ENGINEERING_REQUIREMENT=false`
- `HEURISTIC_RESEARCH_GATE=true`

Preregistration protects v1 history from post-hoc editing, but preregistration alone does not establish that the gate matches Traditional's scientific role.

## Structural mismatch

Competence v1 aggregates 100 nominal and 100 challenge cases into one success rate. A hypothetical method with exactly 70% nominal success and no challenge success would score only 35% overall and be rejected. Traditional eligibility is therefore mathematically coupled to conquering the challenge set.

The existing `native_pid_001` illustrates, but did not define, this issue: all-case safety is 100%, catastrophic count is zero, and its v1 RMSE bounds pass, while success is 47% nominal and 8% challenge, producing 27.5% overall. It is not invalid merely because challenge success is low; nevertheless it also remains insufficiently reliable under moderate nominal wind.

The root cause is a combination: governance coupling is primary, while controller class and implementation limits contribute. The mission envelope itself is not the structural root cause.
