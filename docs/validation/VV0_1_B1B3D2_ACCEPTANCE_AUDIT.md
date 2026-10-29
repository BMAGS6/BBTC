# VV0.1-B1b.3d-2 — Numerical Comparison Core Acceptance Audit

**Disposition:** **CANDIDATE — NOT YET ACCEPTED**. This document is an audit
worksheet and evidence-capture contract. It is not a declaration that every
proof obligation has been discharged.

**Scope:** the *developer-only* native-family VVUQ comparison API, not the
production BBTC physical models, reference-data provenance, measurement
uncertainty, or empirical safety validation.

**Governing contracts:**

- `docs/validation/VVUQ_DESIGN_CONTRACT.md` (VV0.1-A).
- `docs/validation/VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0 (VV0.1-B1a).
- `docs/validation/VV0_1_B1B3_DIAGNOSTIC_DESIGN.md` (B1b.3a).
- Earlier kernel construction/proof notes and B1b.3d-1 oracle documentation.

## 1. Purpose and decision rule

The goal is not to add more arithmetic, but to establish whether the completed
B1 implementation justifies the **specific guarantees** in the accepted B1a
contract on the **specified compiler, scalar-family and host environment**.
A passing test does not by itself close a universal mathematical proof. Nor may
a recognized C floating-point format be used as evidence that every target with
similar macros shares the same execution semantics.

An acceptance decision requires all of the following:

1. A recorded source commit and reproducible, complete execution transcript.
2. Correctly identified native binary32, binary64 and x87 extended
   `long double` characteristics, plus verified compiler/runtime assumptions.
3. Strict optimized GCC and Clang builds, and an instrumented Clang Debug build.
4. All **nine** developer VVUQ CTest entries passing for all three builds,
   including the four independent `fractions.Fraction` oracles.
5. A clean production-only configuration with **no developer VVUQ target or
   defined VVUQ symbol** in the production static library.
6. A separate source-level review of *all* proof obligations in section 3.
7. A documented decision for every identified limitation or unresolved issue.

Do **not** change this document to `ACCEPTED` merely because the capture script
prints a success marker. If a required obligation is unproved, mark the specific
claim **OPEN** and do not certify it.

## 2. Source and executed test baseline

Record the actual Git commit in the evidence transcript; do not substitute a
branch name or an assistant-proposed commit ID. The B1b.3d-1 increment adds:

- `tests/validation/comparison_api_oracle_driver.c`: native hexadecimal
  test transport, fenv observation and status reporting.
- `tests/validation/test_comparison_api_oracle.py`: independent full-result
  Python rational oracle.
- `docs/validation/VV0_1_B1B3D1_END_TO_END_ORACLE.md`: oracle assumptions.
- `tests/validation/CMakeLists.txt`: opt-in ninth CTest registration.

The pre-capture qualification reported the following from the original primary
Arch GNU/Linux environment. These are **previously reported execution results**,
not automatically reproduced by this document:

| Configuration | CTest | Full-API oracle | Reported result |
| --- | ---: | ---: | --- |
| GCC 16.2.1, strict optimized | 9/9 | 31,530 cases | Pass |
| Clang 23.1.4, strict optimized | 9/9 | 31,530 cases | Pass |
| Clang 23.1.4, ASan/UBSan Debug | 9/9 | 31,530 cases | Pass |

All four oracle tests together exercise 421,710 checks per configuration:
300,054 exact dyadic, 60,072 exact metric, 30,054 native rounding and 31,530
full-API comparisons. These are *executions*, not independent or exhaustive
proof of all possible native input tuples. Each configuration uses the same
seeded datasets for reproducibility.

The B1b.3d-2 capture script uses the same test counts and seeds. Any changed
CTest count or compiler-family support should trigger a separate review before
this worksheet is updated.

## 3. Normative contract-to-source audit matrix

Use **PASS / FAIL / OPEN / NOT APPLICABLE**, with a specific source location,
proof note and reproduced test citation for every completed requirement.
**No item below is automatically PASS simply because a test name exists.**

| ID | Review obligation | Relevant source and evidence | Initial disposition |
| --- | --- | --- | --- |
| A01 | Exact, Absolute and Reference-relative methods implement B1a's inclusive inequalities on the exact represented inputs | `exact_dyadic.c`, B1b.2 proof; component and full-API rational oracles | OPEN: source proof review |
| A02 | Every admissible supported comparison returns definitive PASS/FAIL, never an accidental `INDETERMINATE` | `exact_dyadic.c`, `comparison_api.c`; threshold adversaries | OPEN: check all branch paths |
| A03 | The 1,036-limb dyadic grid bounds every aligned signed term and relative product, including carries | B1b.2 integer-range proof, `exact_dyadic.c` | OPEN: range argument audit |
| A04 | The 520-limb exact-metric numerator bounds every finite absolute difference; rational denominator/exponents remain exact | `exact_metric.c`, `VV0_1_B1B3C1_EXACT_METRIC_PROOF.md` | OPEN: bounds and invariants audit |
| A05 | Nearest-even rounding decisions, ties, subnormal transitions and pre-round finite-max classification use exact integer comparisons | `exact_round.c`, `VV0_1_B1B3C2_EXACT_ROUNDING_PROOF.md` | OPEN: midpoint and overflow audit |
| A06 | Diagnostics never influence PASS/FAIL; EXACT never constructs a metric | `comparison_api.c`, B1b.3c-3 integration note | OPEN: control-flow inspection |
| A07 | All failure statuses clear result fields; NaN has precedence over infinity; invalid method/policy and zero reference follow B1a | `comparison.c`, `comparison_api.c`, full-API oracle | OPEN: precedence audit |
| A08 | `float`, `double`, and `long double` retain native-family semantics and are never reduced to `double` in calculations | Header/implementations, transport driver, platform probe | OPEN: cast/call audit |
| A09 | Supported rounding modes and preexisting sticky flags survive *every* supported normal return | All kernels and wrapper fenv guards; fenv tests/oracle | OPEN: early-return and restore audit |
| A10 | The initial certification is restricted to the actual primary binary32/binary64/x87 + compiler/build environment | Capability guards, independent host probe, compiler flags | OPEN: host transcript needed |
| A11 | No `-ffast-math`, unsafe finite-math assumption or unsupported trapping configuration is falsely certified | Compiler guards, build options, documented exclusions | OPEN: unsupported conditions review |
| A12 | No installed public BBTC API change or production dependency on VVUQ; ordinary production build requires no Python/cmocka | CMake, `nm`, production-only target listing | OPEN: capture output needed |
| A13 | Independent Python oracle obtains expected statuses, findings, states and native values without calling C numerical kernels | `test_comparison_api_oracle.py`, B1b.3d-1 notes | OPEN: oracle independence review |
| A14 | All evidence is reproducible, source-identified and free from unreported test skipping | Transcript, CMake registration, seeds, exit status | OPEN: capture output needed |

### Specific review questions

- **Exact decision:** verify the sign decomposition of the two inequalities
  `o-r-T <= 0` and `r-o-T <= 0`, the treatment of signed zeros, and whether
  any possible finite input or tolerance product escapes the integer grid.
- **Exact metric:** check normalization of subnormals, carry and borrow, both
  extreme-sign cases, and exact use of the reference as relative denominator.
- **Rounding:** verify the coefficient search interval, midpoint construction,
  tie-even parity, `p=64` carry behavior, and values mathematically above
  maximum finite that might otherwise round back to finite maximum.
- **Environment:** inspect the path *before* `feholdexcept`, all nested guards,
  `fesetenv` failures and result commits. Unsupported unmasked traps are
  explicitly **outside** the accepted initial domain; this worksheet must
  not imply that the program dynamically recognizes or safely handles them.
- **Format gating:** verify `FLT_RADIX`, all native precision/exponent macros,
  compiler options, `FLT_EVAL_METHOD`, supported subnormal behavior, and
  extended-precision behavior. Similar `sizeof` values are insufficient.
- **Oracle independence:** review exact parsing of `%a`/`%La`, Python Fraction
  derivations, invalid-argument precedence, exact status codes and diagnostics.
  The transported null-output placeholder must not be mistaken for a real
  write through a null pointer.

## 4. Reproduce and capture evidence

On the primary Arch Linux worktree, after B1b.3d-1 has been committed and
pushed, run:

```bash
cd ~/Projects/BBTC_VVUQ_B1b || exit 1
mkdir -p "$HOME/Downloads/junk"
set -o pipefail
bash tools/bbtc_vv/acceptance_capture.sh 2>&1 | tee \
    "$HOME/Downloads/junk/vv0_1b3d2_acceptance_evidence.txt"
```

The script compiles `platform_capability_probe.c` **independently** with each
compiler, reruns the existing complete nine-suite CTest campaign, confirms
the developer archive *defines* all three API symbols, and checks the Clang
production-only build for unexpected VVUQ targets and symbols. It prints
source hashes to associate numerical code with the captured evidence.

On the previously qualified target, expect the final marker:

```
=== EVIDENCE CAPTURE COMPLETE; MANUAL ACCEPTANCE REVIEW STILL REQUIRED ===
```

A missing marker, failed command, missing symbol, changed test registration,
or unsupported format profile is a **failed capture gate**. Save the entire
transcript, not only the final lines.

## 5. Acceptance disposition and exclusions

**Current decision: OPEN.** We have extensive positive testing, but the
manual source/proof audit in section 3 still needs an explicit conclusion.
Do not relabel a proof note's historical "candidate" statement as a proof by
merely editing its heading.

The accepted *numerical* subsystem will answer whether two **represented
floating-point scalars** satisfy a chosen mathematical comparison. It does
not validate a ballistic solver, confirm units, establish sensor accuracy,
justify a reference dataset, calculate physical uncertainty, or certify any
firearm/ammunition condition. Those are separate future VVUQ deliverables.

At the conclusion of the review, write a **separate** final decision record
containing the reviewed commit SHA, verified host profile, compiler flags,
all test totals and seeds, explicit PASS/OPEN/FAIL results for A01–A14,
known limitations, reviewer identity/date and an unambiguous acceptance or
nonacceptance decision. Preserve this candidate audit worksheet as history.
