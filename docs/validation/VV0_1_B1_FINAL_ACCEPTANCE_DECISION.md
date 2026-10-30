# VV0.1-B1 — Numerical Comparison Core: Final Acceptance Decision

**Document class:** project-owner acceptance decision and evidence manifest
**Status:** **ACCEPTED — SCOPED NUMERICAL COMPARISON CORE**
**Review date (UTC):** 2026-10-30
**Reviewed code commit:** `21f804ba0fc2ecb32242662f1c55d9cc8ed74396`
**Repository:** `BMAGS6/BBTC`
**Reviewed branch:** `rewrite/vv0_1b1b3_comparison_api_v1`
**Approval authority / reviewer:** Brandon Magoni (BMAGS), project owner
**Approval date and explicit decision:** 2026-10-30 (UTC) — **ACCEPT** within the bounded scope below

> **Decision boundary:** Project owner Brandon Magoni (BMAGS) explicitly
> approved **scoped acceptance** on 2026-10-30. All A01–A14 technical
> dispositions are adopted as **PASS within the recorded qualified environment**,
> with source-level arguments and a clean-commit qualification campaign as
> evidence. This is a project-owner decision, **not** a third-party certification,
> formal machine-checked proof, or a validation of BBTC physical models.
> Repository publication requires a byte-identical evidence copy with the
> expected SHA-256; do not tag or merge an incomplete evidence record.

## 1. Decision and scope

**Formal owner decision:** **ACCEPT** the *developer-only VV0.1-B1 numerical comparison
core* at the exact reviewed **code commit** identified above, **only** on the
specified primary x86-64 Arch GNU/Linux environment with the verified compiler
options, native formats, runtime behavior, and limitations in sections 1, 2, and 5.

The B1 core implements three typed native-family entry points:
`bbtc_vv_compare_float`, `bbtc_vv_compare_double`, and
`bbtc_vv_compare_long_double`. For admissible finite represented operands and
policies, it delivers a mathematical `PASS`/`FAIL` determination for Exact,
Absolute, or Reference-relative comparison, plus a separately classified,
nearest-even scalar diagnostic for the latter two methods. The comparison
decision operates on the exact mathematical values of the **represented
floating-point inputs**, not a rounded subtraction, division, or displayed
metric. The outcome is a *numerical comparison*, **not** a scientific judgment
about the truth, relevance, or independence of the reference value.

**Not accepted by this decision:** production BBTC physics, external-ballistics
or internal-ballistics model accuracy; measured/reference data provenance;
measurement uncertainties or certified error enclosures; dimensional
consistency; numerical behavior on other architectures; unmasked exceptions;
or safety or suitability of any physical configuration. This record neither
changes nor accepts any installed production BBTC API/ABI.

The code commit is an immutable **reviewed implementation baseline**. The
subsequent documentation/evidence commit is *not* a replacement numerical
baseline. A release/acceptance tag on the final documentation commit must
state that the actual tested numerical source is the above code commit, not
claim a new code qualification. The tag itself is a separate Git operation.

## 2. Source, host, and verification identity

### 2.1 Fixed implementation baseline

- Git commit: `21f804ba0fc2ecb32242662f1c55d9cc8ed74396`.
- Last implementation-relevant increments include the exact-dyadic, metric,
  rounding, whole-API, capability-hardening, and oracle-integrity reviews.
- The audit documents in section 4 were reviewed against the committed branch.
- The final capture began with `PRE_RUN_CLEAN=1` and ended with
  `POST_RUN_CLEAN=1`; `git status --porcelain` was empty at both checks.
- Final evidence recording started **2026-10-30T04:50:07Z**.
- Host: Linux `7.2.11-arch1-1`, x86-64, Arch GNU/Linux.
- Toolchains: GCC `16.2.1 20260810`; Clang `23.1.4`; CMake `4.4.5`;
  Ninja `1.13.2`; Python `3.14.8`; cmocka `2.0.2`.
- Language / environment: C standard reported `202311`, `CHAR_BIT=8`,
  `FLT_RADIX=2`, `FLT_EVAL_METHOD=0`.

### 2.2 Qualified floating-point families

| C family | Storage | Significand precision | `MIN_EXP` | `MAX_EXP` | Verification |
| --- | ---: | ---: | ---: | ---: | --- |
| `float` | 32 bits | 24 bits | -125 | 128 | Matching binary32 profile |
| `double` | 64 bits | 53 bits | -1021 | 1024 | Matching binary64 profile |
| `long double` | 128-bit storage | 64 bits | -16381 | 16384 | Padded x87 binary80 extended precision |

The 128-bit `long double` **storage** size must not be misrepresented as
128-bit significand precision or IEEE binary128. Both GCC and Clang reported
`PRIMARY_FORMAT_PROFILE_MATCH=1` and `FENV_SMOKE_TEST_PASS=1`.

The verified build configurations use strict floating-point options:
`-frounding-math -ffp-contract=off -fno-fast-math`, with a warning-as-error
compiler configuration. No claim is extended to `-ffast-math`, trapping
exceptions, flush-to-zero behavior outside the admitted probe, noncanonical
extended representations, or an untested compiler/ABI.

### 2.3 Final clean-source acceptance capture

**Source transcript, on the qualified host:**
`$HOME/Downloads/junk/vv0_1b_final_clean_commit_acceptance.txt`.

**Designated immutable Git evidence copy:**
`docs/validation/evidence/VV0_1_B1_ACCEPTANCE_20261030.txt`.

**Expected SHA-256 (measured on the host by the owner):** `1da8d949434b1b9e2c8a83bec10b608ef63832aba5389e878de80bcbabaa2a88`.

The repository copy MUST be made from the original transcript **without any
byte changes** and MUST hash to the value above before it is staged or
committed. The designated archive path is a **publication instruction** until
the final documentation/evidence Git commit exists; this record by itself
does not claim that the evidence has already been pushed. The reviewed code
commit remains the capture's source identity regardless of the later archive
commit.

| Acceptance gate | Final observed result |
| --- | --- |
| Oracle integrity mutation/positive-control tests | 15/15 PASS |
| GCC strict CTest suites | 9/9 PASS |
| Clang strict CTest suites | 9/9 PASS |
| Clang Debug ASan/UBSan CTest suites | 9/9 PASS |
| Aggregate CTest suite executions | 27/27 PASS |
| Exact-dyadic Python rational oracle | 300,054 cases / configuration |
| Exact-metric Python rational oracle | 60,072 cases / configuration |
| Exact-rounding Python rational oracle | 30,054 cases / configuration |
| Whole-API Python rational oracle | 31,530 cases / configuration |
| All four oracle programs | 421,710 executions / configuration |
| Three configurations | 1,265,130 repeated-corpus oracle executions |
| Defined developer API symbols | All three present (GCC and Clang) |
| Production-only VVUQ targets | 0 |
| Production-only defined VVUQ symbols | 0 |
| Source working tree before / after | `PRE_RUN_CLEAN=1`; `POST_RUN_CLEAN=1` |
| Capture process exit status | `FINAL_ACCEPTANCE_CAPTURE_EXIT_STATUS=0` |

The oracle datasets use deterministic seeds, and the same corpus is executed
under each compiler configuration. **1,265,130 is not a count of distinct
mathematical test cases.** The capture script records the seeds, requested
counts, platform probe output, source-file SHA-256 values, test registration,
and tool versions. Reproduction requires preserving its exact transcript.

**Build distinction:** Ninja printed `no work to do` for the already-configured
build directories. This demonstrates a fully rerun test campaign against a
clean *source tree*; it does **not** claim a clean rebuild from an empty build
directory. A fresh-build demonstration is a useful additional release check,
but is not an explicitly required B1b.3d-2 acceptance gate.

## 3. Final A01–A14 contract disposition matrix

**Interpretation:** `PASS (SCOPED)` is the adopted project-owner disposition
for the documented, qualified native host/compiler/fenv environment. Each
entry relies on the source arguments and the successful reproducible campaign;
it does not purport to be an independent third-party certification or
machine-checked proof. Any reproducible contradiction reopens the affected
obligation and requires a new decision and qualification.

| ID | Required proposition | Technical disposition | Supporting evidence and bounded rationale |
| --- | --- | --- | --- |
| **A01** | Exact, Absolute, Reference-relative inclusive comparisons of represented real values | **PASS (SCOPED)** | `exact_dyadic.c`; `VV0_1_B1B3D3B1_DYADIC_SOURCE_AUDIT.md` §§1–3; exact rational dyadic and full-API oracles. Exact algebra reduces `\|o-r\| <= T` to two signed inequalities without native rounded subtraction. |
| **A02** | Every successful admissible initial comparison is definitive PASS or FAIL | **PASS (SCOPED)** | `comparison.c`, `comparison_api.c`; `VV0_1_B1B3D3C1_WHOLE_API_CONTRACT_AUDIT.md` §§1–2. The successful finding originates from a Boolean; `INDETERMINATE` is not published. Failures remain status failures, not fabricated findings. |
| **A03** | 1,036-limb exact dyadic grid bounds all admissible signed terms/products/carries | **PASS (SCOPED)** | `VV0_1_B1B3D3B1_DYADIC_SOURCE_AUDIT.md` §§2–4: 66,304 bit-position grid `[-33152,33152)`, conservative admissible support `[-33016,32770)`, and checked unsigned-limb carries. |
| **A04** | Exact rational metric and 520-limb storage are sufficient | **PASS (SCOPED)** | `exact_metric.c`; `VV0_1_B1B3D3B2_EXACT_METRIC_SOURCE_AUDIT.md` §§2–5. At most 32,893 numerator bits against 33,280 allocated; denominator is exact unsigned reference magnitude in relative mode. |
| **A05** | Rounding, ties-even, finite-range and subnormal state classification | **PASS (SCOPED)** | `exact_round.c`; `VV0_1_B1B3D3B3_EXACT_ROUND_SOURCE_AUDIT.md` §§2–5. Exact midpoint comparison has a 129-bit multiplier path; coefficient carry handles 64-bit x87 without storing `2^64`; range checked before rounding. |
| **A06** | Diagnostic never governs finding; Exact does not construct metric | **PASS (SCOPED)** | `comparison_api.c`; whole-API audit §§1–2, directed `2^p+1` and overrange tests. Finding is produced first and published independently of diagnostic representability. |
| **A07** | Precedence and canonical result fields on all unsuccessful returns | **PASS (SCOPED)** | `comparison.c`, `comparison_api.c`; whole-API audit §§1–2, 24-case cmocka API suite and full-API oracle. Structural errors precede numeric classification; NaN precedes infinity in supported environment; failure results are cleared **field-wise**, not padding-wise. |
| **A08** | Preserve native `float`/`double`/`long double` semantics | **PASS (SCOPED)** | Typed kernels and transport drivers, native-family regressions; whole-API audit §3. Long-double values retain their x87 precision and exponent range; `printf` float-to-double promotion is exact for protocol reporting, not a computational fallback. |
| **A09** | Preserve supported caller rounding modes and sticky flags on normal returns | **PASS (SCOPED)** | Nested `feholdexcept`/`fesetenv` source paths; whole-API audit §§1,3–4; `FENV_SMOKE_TEST_PASS=1` and directed caller-mode/flag matrix. Failed host `fesetenv` and unmasked-trap executions are not claimed to preserve state. |
| **A10** | Restrict certification to verified platform/format/environment | **PASS (SCOPED)** | Capability-gated source and independent host probe in all three configurations; matches binary32, binary64 and padded x87 under `FLT_EVAL_METHOD=0`. This is **host-qualified**, not portable-family certification. |
| **A11** | Exclude unsafe compiler/FP assumptions from certification | **PASS (SCOPED)** | `__FAST_MATH__`/finite-math guards, strict build flags and explicitly excluded unmasked traps/unsafe modes; whole-API audit §3. The implementation does **not** promise to detect every possible unsupported trap configuration at runtime. |
| **A12** | Isolation from installed BBTC production API and runtime dependencies | **PASS (SCOPED)** | CMake opt-in VVUQ configuration; acceptance output reports zero VVUQ targets and defined symbols in production-only Clang build; cmocka and Python remain developer-test-only. |
| **A13** | Arithmetic oracle is independent of C numerical implementation and validates contract fields | **PASS (SCOPED)** | `VV0_1_B1B3D3C2_ORACLE_INTEGRITY_AUDIT.md` §§1–3; Python `fractions.Fraction` and integer `divmod`; 15 oracle mutation/positive-control checks. Two Python rounding references share the same mathematical quotient/remainder rule and are **not** independent of each other as mathematical theories. |
| **A14** | Source-identified, reproducible campaign with no silently skipped required tests | **PASS (SCOPED)** | Clean-commit transcript at exact code SHA, explicit tool/host probe, nine CTest entries per configuration, four oracle registrations, self-test preflight, file SHA-256 inventory, checked production separation, clean pre/post status and process exit 0. Existing build outputs were reused; see §2.3. |

No A01–A14 obligation is marked FAIL or OPEN **within this limited
accepted scope**. The project owner adopts these fourteen source/evidence
assessments and accepts the B1 numerical core under the exclusions above.
This decision does not assert exhaustive empirical testing of every possible
value, independent third-party review, or proof on unqualified hosts. If a
material objection is discovered, amend the acceptance record explicitly;
do not silently reinterpret this decision.

## 4. Traceable proof and contract bibliography

These are **relative repository paths at the reviewed code commit**, not
links to mutable live branch content:

1. Normative scope:
   `docs/validation/VVUQ_DESIGN_CONTRACT.md`,
   `docs/validation/VVUQ_COMPARISON_CORE_CONTRACT.md` (v0.1.0),
   `docs/validation/VV0_1_B1B3_DIAGNOSTIC_DESIGN.md`.
2. Historical acceptance worksheet (preserve as *candidate*, do not rewrite):
   `docs/validation/VV0_1_B1B3D2_ACCEPTANCE_AUDIT.md`.
3. Exact decision / grid:
   `docs/validation/VV0_1_B1B3D3B1_DYADIC_SOURCE_AUDIT.md`.
4. Exact rational metric / bounds:
   `docs/validation/VV0_1_B1B3D3B2_EXACT_METRIC_SOURCE_AUDIT.md`.
5. Correct rounding:
   `docs/validation/VV0_1_B1B3D3B3_EXACT_ROUND_SOURCE_AUDIT.md`.
6. Whole-API behavior:
   `docs/validation/VV0_1_B1B3D3C1_WHOLE_API_CONTRACT_AUDIT.md`.
7. Oracle source independence and parser mutation:
   `docs/validation/VV0_1_B1B3D3C2_ORACLE_INTEGRITY_AUDIT.md`.
8. Executable capture and case registration:
   `tools/bbtc_vv/acceptance_capture.sh`,
   `tests/validation/CMakeLists.txt` and the four `test_*_oracle.py` scripts.
9. Qualified code: `tools/bbtc_vv/exact_dyadic.c`, `exact_metric.c`,
   `exact_round.c`, `comparison.c`, `comparison_api.c`, and `comparison.h`.

All code/document paths are scoped to reviewed commit
`21f804ba0fc2ecb32242662f1c55d9cc8ed74396`.

## 5. Evidence preservation, owner decision, and publication

### 5.1 Repository publication checklist

1. **Preserve the accepted numerical source.** Do not change C numerical
   kernels, Python oracles, driver behavior, CMake registration, compiler
   flags, or capability guards while claiming the existing capture still
   qualifies that changed code. A changed implementation requires new review.
2. **Copy the original complete transcript byte-for-byte** from the qualified
   host into `docs/validation/evidence/VV0_1_B1_ACCEPTANCE_20261030.txt`. Check that its SHA-256 is `1da8d949434b1b9e2c8a83bec10b608ef63832aba5389e878de80bcbabaa2a88` both before and
   after copying; also compare the files byte-for-byte. **If it differs, do not
   stage, commit, tag, or publish the supposed evidence as equivalent.**
3. **Stage only this decision and the validated evidence transcript**; inspect
   `git diff --cached --check`, `git diff --cached --name-status`, and the
   source tree's unchanged numerical files. The earlier candidate worksheet
   must remain intact as a historical record.
4. Commit the acceptance record and archived transcript together, then
   push the branch. The acceptance-record commit can be recovered with
   `git log -- docs/validation/VV0_1_B1_FINAL_ACCEPTANCE_DECISION.md`;
   embedding its own commit hash in the document would be self-referential.
5. If a versioned annotated acceptance tag is created, record its **target
   commit and scope** in the tag message. The documentation/record tag may
   point at the acceptance-record commit while identifying the distinct
   accepted numerical code baseline `21f804ba0fc2ecb32242662f1c55d9cc8ed74396`.

### 5.2 Final owner decision and evidence manifest

| Field | Recorded decision or source |
| --- | --- |
| Final evidence transcript SHA-256 | `1da8d949434b1b9e2c8a83bec10b608ef63832aba5389e878de80bcbabaa2a88` |
| Durable archive path, to be committed | `docs/validation/evidence/VV0_1_B1_ACCEPTANCE_20261030.txt` |
| Original transcript on qualified host | `$HOME/Downloads/junk/vv0_1b_final_clean_commit_acceptance.txt` |
| Reviewer / approving authority | **Brandon Magoni (BMAGS)** — project owner |
| A01–A14 dispositions | **14/14 PASS (SCOPED)**; no open obligations within the qualified scope |
| Reviewed numerical-code commit | `21f804ba0fc2ecb32242662f1c55d9cc8ed74396` |
| Acceptance-document/evidence commit | Resolved by Git history after publication; intentionally not self-embedded |
| Approval date (UTC) | **2026-10-30** |
| Formal decision | **ACCEPT VV0.1-B1 WITH STATED NUMERICAL AND PLATFORM LIMITATIONS** |
| Basis of owner approval | Explicit confirmation by project owner in project correspondence on 2026-10-30 |

**Project-owner attestation (record of express authorization):** Brandon
Magoni (BMAGS) has approved acceptance of the developer-only VV0.1-B1
numerical comparison core **within the limits in this decision record**, and
has authorized preserving the qualification transcript in the repository.
The A01–A14 determinations rely on the documented technical source reviews and
qualified campaign; this is not a claim that an independent third party
personally replicated every proof or test. The acceptance record documents
express owner approval, **not** a cryptographic signature.

**Final project decision:** **ACCEPTED (SCOPED)** for the reviewed numerical
code revision. **Evidence-publication condition:** complete the byte-identity
check and commit both this record and the unmodified full qualification
transcript before treating the repository as containing the archival evidence.
