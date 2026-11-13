# VV0.1-B2b.4b Stage 1 — Controlled producer-mutation campaign

| Field | Value |
| --- | --- |
| Phase | `VV0.1-B2b.4b`, Stage 1 |
| Status | **REVIEW DRAFT — NOT B2b ACCEPTANCE** |
| Intended source baseline | Stage 2 reference-neighbor commit `1bc61c0497abc76522ccc09f2cda94594632f987` |
| Fixture authority | Accepted B2a M-01–M-04 reference matrix and Stage 1 rational oracle |
| Fault model | Bounded, intentionally incorrect edits to one *native evaluator* at a time |
| Tool | `tools/bbtc_vv/virial_mutation_campaign.py` |
| Dependencies | Python 3.12+, Git, CMake, Ninja, C23 GCC/Clang, pkg-config and cmocka |
| Production changes | **None** |
| Approval state | Owner Q1 deferred; owner Q2 requires strengthened assurance; Q3 proposed, Q4 unapproved |

## 1. Audit question

**Can the already committed, immutable independent mathematical reference test detect representative arithmetic defects in the production first-order virial temperature-law evaluator, separately for `float`, `double`, and `long double`?**

A normal successful execution tests the correct implementation. This campaign deliberately changes the implementation while keeping the analytical reference, B1 comparator, adapter, and test fixture definitions **unchanged**. Only when a bad *compiled* implementation reaches the verifier and its designated `test_complete_reference_matrix` assertion fails has a mutant been **DETECTED**. Compilation failure, timeout, crash, or an unrelated test failure does not count as detection.

These are intentionally selected fault hypotheses, not a statistically valid sample of all possible C bugs. A high detection fraction here cannot establish universal mutation adequacy, arbitrary-input exactness, or hardware qualification.

## 2. Fault catalogue and predeclared outcomes

The campaign mutates each of these expressions in exactly one native-family evaluator at a time, giving 18 experiments: six edits × three families.

| Mutant identifier | Deliberately introduced error | Predeclared outcome | Frozen-case sensitivity |
| --- | --- | --- | --- |
| `normalization_offset` | Replace the normalized-coordinate `−1` offset by `+1` | **DETECTED** | Nonconstant linear/quadratic fixtures |
| `temperature_scale` | Replace the affine derivative scale `2/range` with `1/range` | **DETECTED** | M-02, M-03, M-04 nonzero derivatives |
| `reconstruction_sign` | Change the `−b₂` reconstruction term to `+b₂` | **DETECTED** | M-03, M-04 virial coefficient |
| `second_derivative_factor` | Replace the factor `2` multiplying `b′₁` with `1` | **DETECTED** | M-03, M-04 second derivatives |
| `first_derivative_guard` | Invert the first-derivative nonzero guard | **DETECTED** | M-02 and M-04 first derivatives |
| `inactive_left_clamp_control` | Change only the already-validated coordinate clamp from `x < −1` to `x < −2` | **SURVIVED** (boundary control) | No frozen reference uses the left clamp |

The intentionally surviving control makes a limitation visible: M-01–M-04 do not test coordinate-clamp roundoff near the domain boundary. It must not be misreported as a detected mutant or automatically treated as a production defect. Other tests or a separate boundary campaign would be required to challenge that behavior.

## 3. Isolation and reproducibility

1. Verify the full expected committed HEAD hash before doing anything. Refuse unexpected revisions. Do not require a clean working tree: the uncommitted B2b acceptance-audit draft is intentionally excluded.
2. Retrieve the exact producer blob from `git show HEAD:path` and independently verify it against a `git archive HEAD` snapshot. `git archive` copies **committed files only**, and does not alter Git worktree administrative state.
3. Create an automatically removed temporary source tree and an out-of-tree Ninja build. Build only the required mathematical reference runner; do not build unrelated test targets.
4. Before mutating anything, run the pristine reference-matrix executable and require success. This is necessary to detect a broken baseline rather than attribute infrastructure trouble to an intentional mutation.
5. For each pair `(family, mutation)`, start from the identical pristine source, replace exactly one expression in the named evaluator, save the explicit unified diff and source SHA-256, force recompilation of the production object, and execute the same unchanged reference runner.
6. Record build logs, test logs, executable SHA-256, exact return code, expected verdict, and actual verdict in persistent JSON. The temporary binaries and source snapshot are deleted automatically. Evidence is kept outside Git and should eventually be archived and hashed.
7. Fail the campaign on an unexpected survivor, failure to compile, missing compiled object, missing designated test, or any result other than the predeclared expectation. Do **not** adjust a fixture, EXACT tolerance, or original source during this process.

A source-edit preflight (`--plan`) verifies all 18 lexical anchors before any build. A separate `--self-test` validates pattern/replacement integrity without requiring the repo.

## 4. Initial execution on Arch Linux

From the local BBTC checkout, using a new evidence path outside the repo:

```sh
python3 tools/bbtc_vv/virial_mutation_campaign.py --self-test

python3 tools/bbtc_vv/virial_mutation_campaign.py \
    --expected-head 1bc61c0497abc76522ccc09f2cda94594632f987 \
    --plan

python3 tools/bbtc_vv/virial_mutation_campaign.py \
    --expected-head 1bc61c0497abc76522ccc09f2cda94594632f987 \
    --compiler gcc \
    --output "$HOME/Projects/BBTC-build/vv0_1b2b4b_stage1_1bc61c0/gcc_strict"
```

Run the first campaign with GCC. If it passes after local review, repeat with `--compiler clang` and a distinct output path; **do not** overwrite the GCC evidence. Strict flags are `-O1 -g -frounding-math -ffp-contract=off -fno-fast-math`, matching the previous verification baseline. The output location must not already exist.

## 5. Evidence acceptance conditions

- Exactly **15 of 15 required detection mutants** (five per family) must be reported `DETECTED` from an identifiable cmocka `test_complete_reference_matrix` failure after a **successful compile/link**.
- Exactly **3 of 3 boundary-control mutants** must be reported `SURVIVED` (one per family) in the current four-case matrix. This establishes that the campaign honestly reports a deliberately untested path.
- The unmutated baseline must pass, and the runner must report the designated test name.
- The mutation tool must never write to the live checkout, and the staged Git diff must contain only the developer-only script and this plan.
- Results from a fresh second compiler are supplementary evidence; neither profile grants formal compiler/ABI/fenv qualification. Sanitizers and real hardware-state rejection are planned in a separate increment.
- A mutant *not* detected as expected is an **audit finding**, never something to relabel as success. Preserve its logs before investigating whether the mutation is equivalent, the test is insufficient, or the environment was invalid.

## 6. Follow-up work and limitations

A passing campaign demonstrates fault-detection capability against *these specifically chosen wrong expressions*, not the correctness of all possible coefficient orders or thermal states. The four fixed dyadic cases do not exercise branch conditions near temperature-domain boundaries, high-degree coefficients, catastrophic cancellation, or extreme exponents. Those require their own independently justified fixtures and acceptance policies, not silent changes to the frozen M-01–M-04 references.

Next stages should include live, isolated x87/MXCSR rejection tests; full artifact-linked compile-command and native ABI review; and eventually general-input high-precision error analysis under mathematically justified tolerances. **Owner Q1/Q2 remain deferred and no B2b acceptance tag is authorized by this document or the campaign.**
