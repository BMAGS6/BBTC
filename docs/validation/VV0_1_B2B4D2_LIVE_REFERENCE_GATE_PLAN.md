# VV0.1-B2b.4d.2 — Integrated live reference-gate test plan

| Field | Value |
| --- | --- |
| Record class | Developer-only verification plan; no owner acceptance |
| Target base | `a58b13a27c327ca5315d78bbfbc789e9354fffae` |
| Affected source | `tests/validation/test_virial_reference_matrix.c` only |
| Test audience | GNU/Clang x86-64 Linux, isolated POSIX `fork()` children |
| Production modifications | **None** |
| Numerical and scientific qualification | **Not granted** |

## 1. Identified integration gap

The previous runner checked a C/native profile and live x87/MXCSR control
before executing the 36 required comparisons. A distinct B2b.4b suite altered
six real hardware-control states and verified the *screening* function's
rejection. But that campaign did not run the reference runner under the
perturbed states, and a separate runner self-test passed an artificial false
Boolean for a rejected profile.

This increment connects the hardware observation to the *existing* reference
runner entry path. A shared `screen_reference_environment()` helper screens
both C/native metadata and actual x87/MXCSR controls. The existing required
matrix and the new adversarial cases use precisely this helper. No artificial
"unsupported" Boolean is supplied by the new adversarial cases.

## 2. Predeclared challenges and assertions

Two complementary hardware states are examined. The other four individual
states have already been tested in `test_virial_live_fp_rejection.c` and are
not reimplemented here.

| Live challenge | Changed control | Mandatory rejection |
| --- | --- | --- |
| x87 upward rounding | Only the x87 RC field is changed to upward | `X87_ROUND` issue |
| SSE FTZ | Set MXCSR FTZ; all masks and other controls retained | `MXCSR_FTZ` issue |

For each challenge, run a child process that saves real x87 and MXCSR control
words, applies the change, and invokes `screen_reference_environment()`.
Require `UNSUPPORTED`, the specific issue bit, and effective modification.
Then submit the unchanged, valid M-02 reference fixture to *all three native*
`run_reference_*` functions with the **actual gate result**.

For each native family, require accepted reference metadata, an unsupported
execution profile, no producer invocation, no valid numerical observation,
no B1 comparison attempt, and no numerical PASS or FAIL finding. The test
must not count a missing field or case as a mathematical PASS. Restore both
control words before communicating child results. Parent verifies complete
pipe evidence, normal child exit, restored state, and its own unchanged
floating-point controls. Baseline unsupported conditions fail, never skip.

## 3. Scope boundaries

- The two children execute no admitted mathematical reference comparisons.
  Passing is **rejection evidence**, not one of the 36 required B1 findings.
- The reference matrix positive case remains intact, retaining both before-
  and after-evaluation control observations and its 36 required EXACT checks.
- This is a bounded sampled-preflight demonstration, not continuous control-
  state monitoring inside the production evaluator and not a proof of the
  entire call graph or of concurrent changes in other threads.
- Real hardware challenges are compiled only with GNU/Clang on Linux x86-64.
  Other targets retain the original reference suite without pretending that
  live-gate tests executed; audit scope must record that restriction.
- Does not establish physical virial EOS accuracy, ammunition/firearm safety,
  arbitrary-input numerical exactness, compiler qualification, or B2b owner
  acceptance. Q1/Q2 and final decision remain deferred.

## 4. Test and review protocol

1. Apply the two-file patch to the exact base; preserve the unrelated
   `VV0_1_B2B_FINAL_IMPLEMENTATION_AUDIT_DRAFT.md` as intent-to-add only.
2. Use the existing strict GCC/Clang and Clang ASan/UBSan profiles in fresh
   out-of-tree builds with ordinary VVUQ tests on, Python oracles off.
3. Focus on `bbtc.vv.virial_reference_matrix` and
   `bbtc.vv.virial_live_fp_rejection`, then run the entire CTest suite.
4. Expect two *additional cmocka cases* in the existing reference executable:
   12 rather than 10 on supported hosts; **CTest count remains 33**.
5. Preserve build/test logs, `compile_commands.json`, exact revision and
   source hashes, and review the test implementation for false-green paths.
6. Do not mark the audit complete or commit an acceptance decision until the
   new results, residual clamp-mutant question, and the contract mapping
   have received separate review.
