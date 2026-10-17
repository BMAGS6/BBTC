# BBTC VVUQ Comparison Core — Implementation Contract

| Metadata | Value |
| --- | --- |
| Contract version | **0.1.0** |
| Project phase | **VV0.1-B1a** |
| Status | **Accepted — implementation pending** |
| Acceptance date | **2026-10-16** |
| Review revision | **2 — four numerical/fenv decisions incorporated and accepted** |
| Target branch | `rewrite/vv0_1b1_vvuq_comparison_core_v1` |
| Baseline | `07400744ad0cdf63a95ff128d765257c5dd5901d` |
| Governing VVUQ contract | `docs/validation/VVUQ_DESIGN_CONTRACT.md` v0.1.0 |
| Governing production contract | `docs/design/BBTC_DESIGN_CONTRACT.md` v0.1.24 |

### B1a review decisions incorporated

| Decision | Contracted choice |
| --- | --- |
| 1 — Decision fidelity | Mathematically exact `PASS`/`FAIL` on supported admissible inputs |
| 2 — Initial formats | Verified binary implementations first; unsupported configurations rejected explicitly |
| 3 — Error diagnostics | Exact findings and faithfully classified scalar metrics now; certified lower/upper enclosures in a later checkpoint |
| 4 — Floating-point state | Preserve caller rounding mode and sticky exception flags on supported normal return paths |

## 1. Purpose, authority, and boundaries

This document specifies **VV0.1-B1**, the first independently testable C23 numerical comparison subsystem of BBTC's Verification, Validation, and Uncertainty Quantification (VVUQ) infrastructure. The terms **MUST**, **MUST NOT**, **SHOULD**, **SHOULD NOT**, and **MAY** are normative within this project. The accepted governing contracts take precedence; any discovered conflict requires explicit review rather than silent reinterpretation.

The comparison subsystem answers a narrow question: given two representable scalar values and a declared numerical criterion, **does an admissible mathematical comparison pass or fail for the exact represented input values?** It does not decide whether the reference is independent, the units agree, an uncertainty estimate is credible, a physical model is validated, or any firearm or ammunition configuration is safe. Those responsibilities belong to subsequent layers and phases.

**VV0.1-B1a is documentation-only.** Accepting it authorizes subsequent reviewable implementation of this narrow interface, not changes to BBTC physics, its public ABI, the governing scientific evidence model, or claims of empirical validation.

## 2. Architectural separation

1. The comparison core MUST be standard ISO C23 and MUST compile and link without `bbtc::bbtc`, its headers, or its source code. No production BBTC target may depend on the core.
2. Its own types and statuses MUST be under the `bbtc_vv_` namespace and MUST NOT reuse `bbtc_status_e` as if operation success and scientific findings were interchangeable.
3. The core MUST have no mandatory heap allocation, network access, file I/O, serialization, global mutable state, or third-party runtime dependency. Inputs and results are caller-owned.
4. The core MUST NOT invoke reference calculators, synthesize expected physical measurements, or silently substitute previously calculated BBTC outputs as independent references.
5. The B1 implementation MAY use a local static-library target, but MUST NOT install its internal header as a production BBTC public header.
6. **cmocka** SHOULD exercise deterministic self-tests. cmocka MUST be test-only; **DeepState** and **libFuzzer** remain optional later test targets. CTest SHOULD orchestrate B1 tests.
7. Build integration MUST keep an ordinary BBTC build possible without discovering or requiring cmocka, DeepState, libFuzzer, or other VVUQ-only packages.

## 3. Scope of the initial kernel

B1 MUST implement exactly three initial comparison methods, without assigning them global scientific meaning. For supported environments and admissible finite operands, every method MUST produce the mathematically correct definitive `PASS` or `FAIL`, not a conservative `INDETERMINATE`:

- **Exact:** numerical equality of finite reference and observation values. Oppositely signed zeros compare equal; signed-zero identity is **not** a bitwise-equality policy.
- **Absolute:** evaluate the nonnegative real-number relationship `|observation - reference| <= absolute_tolerance`, where the three operands are interpreted as the **exact mathematical values of their represented floating-point numbers**. Tolerance has the same quantity units as the compared values; verifying those units is a higher-layer duty.
- **Reference-relative:** for nonzero reference, evaluate `|observation - reference| <= relative_tolerance * |reference|`, again interpreting represented inputs as exact real values for the purpose of the decision. Relative tolerance is dimensionless and nonnegative. The reference is explicitly the denominator/scale: reversing the operands can change the answer.

Every inequality above is inclusive at equality. There MUST be no hidden default tolerance, epsilon multiplier, uncertainty substitution, or absolute fallback inside the reference-relative method. In absolute and reference-relative modes, a zero tolerance is legal and entails the same pass/fail equality relation as Exact.

ULP *distance*, mixed absolute/relative criteria, interval reference records, vector norms, convergence fits, dimensional-unit checking, covariance-aware statistical comparison, and empirical evidence judgments are **out of scope for B1**. They may be specified later with separate policies and tests.

## 4. Native scalar families and proposed C23 interface

The B1 implementation SHALL expose three distinct typed entry points, supporting the actual native `float`, `double`, and `long double` types. It MUST NOT route `float` or `long double` computations through `double`. It MUST NOT assume that `long double` is wider than `double` or that any type's `sizeof` uniquely identifies its floating-point format.

The following API is the accepted B1a implementation target. Declarations are internal to the developer validation subsystem; enum numbers and structure layout are not a production ABI promise.

```c
#include <stdint.h>

typedef enum bbtc_vv_comparison_method : uint8_t
{
    BBTC_VV_COMPARISON_EXACT = 1,
    BBTC_VV_COMPARISON_ABSOLUTE = 2,
    BBTC_VV_COMPARISON_REFERENCE_RELATIVE = 3
} bbtc_vv_comparison_method_e;

typedef enum bbtc_vv_operation_status : uint8_t
{
    BBTC_VV_STATUS_SUCCESS = 0,
    BBTC_VV_STATUS_INVALID_ARGUMENT = 1,
    BBTC_VV_STATUS_NAN_INPUT = 2,
    BBTC_VV_STATUS_NONFINITE_INPUT = 3,
    BBTC_VV_STATUS_OUTSIDE_DOMAIN = 4,
    BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT = 5,
    BBTC_VV_STATUS_NUMERICAL_FAILURE = 6
} bbtc_vv_operation_status_e;

typedef enum bbtc_vv_finding : uint8_t
{
    BBTC_VV_FINDING_NOT_EVALUATED = 0,
    BBTC_VV_FINDING_PASS = 1,
    BBTC_VV_FINDING_FAIL = 2,
    BBTC_VV_FINDING_INDETERMINATE = 3
} bbtc_vv_finding_e;

typedef enum bbtc_vv_metric_state : uint8_t
{
    BBTC_VV_METRIC_NOT_COMPUTED = 0,
    BBTC_VV_METRIC_FINITE_EXACT = 1,
    BBTC_VV_METRIC_FINITE_ROUNDED = 2,
    BBTC_VV_METRIC_EXCEEDS_RANGE = 3,
    BBTC_VV_METRIC_ROUNDED_TO_ZERO = 4,
    BBTC_VV_METRIC_UNRESOLVED = 5
} bbtc_vv_metric_state_e;

typedef struct bbtc_vv_policy_float_t
{
    bbtc_vv_comparison_method_e method;
    float tolerance;
} bbtc_vv_policy_float_t;

typedef struct bbtc_vv_policy_double_t
{
    bbtc_vv_comparison_method_e method;
    double tolerance;
} bbtc_vv_policy_double_t;

typedef struct bbtc_vv_policy_long_double_t
{
    bbtc_vv_comparison_method_e method;
    long double tolerance;
} bbtc_vv_policy_long_double_t;

typedef struct bbtc_vv_result_float_t
{
    bbtc_vv_finding_e finding;
    bbtc_vv_metric_state_e metric_state;
    float metric;
} bbtc_vv_result_float_t;

typedef struct bbtc_vv_result_double_t
{
    bbtc_vv_finding_e finding;
    bbtc_vv_metric_state_e metric_state;
    double metric;
} bbtc_vv_result_double_t;

typedef struct bbtc_vv_result_long_double_t
{
    bbtc_vv_finding_e finding;
    bbtc_vv_metric_state_e metric_state;
    long double metric;
} bbtc_vv_result_long_double_t;

bbtc_vv_operation_status_e bbtc_vv_compare_float(
    float reference,
    float observation,
    const bbtc_vv_policy_float_t* policy,
    bbtc_vv_result_float_t* out_result
);

bbtc_vv_operation_status_e bbtc_vv_compare_double(
    double reference,
    double observation,
    const bbtc_vv_policy_double_t* policy,
    bbtc_vv_result_double_t* out_result
);

bbtc_vv_operation_status_e bbtc_vv_compare_long_double(
    long double reference,
    long double observation,
    const bbtc_vv_policy_long_double_t* policy,
    bbtc_vv_result_long_double_t* out_result
);
```

The `metric` in Absolute mode represents the magnitude of the absolute difference in the native scalar family; in Reference-relative mode it represents the dimensionless ratio of absolute difference to absolute reference value. In Exact mode, `metric_state` MUST be `NOT_COMPUTED` and `metric` MUST be zero. A **metric is diagnostic data, never the authority for the exact finding**. All metrics are nonnegative when present.

Metric states mean:

- `NOT_COMPUTED`: no diagnostic requested by the selected method, or no valid operation completed; `metric` is zero.
- `FINITE_EXACT`: the exact mathematical metric is representable by the native scalar family and is returned exactly, **including exact zero**.
- `FINITE_ROUNDED`: the exact mathematical metric is positive, finite, nonzero, not representable exactly, and the returned finite nonzero `metric` is a correctly rounded approximation under the documented supported diagnostic rounding rule. This state MUST NOT claim exactness.
- `EXCEEDS_RANGE`: the mathematical metric exceeds the maximum finite value of the native scalar family; `metric` is zero, rather than infinity masquerading as a usable diagnostic.
- `ROUNDED_TO_ZERO`: the exact metric is strictly positive, but its diagnostic representation rounds to zero; `metric` is zero.
- `UNRESOLVED`: a diagnostic cannot be defensibly computed or classified under the documented algorithm despite the exact finding being decidable; `metric` is zero, and this condition MUST be distinguishable in tests/reports from genuine finite-zero error. The implementation MUST NOT use it as a routine substitute for producing available diagnostics.

The metric is evaluated from the exact mathematical values of the native-family inputs. The finite tolerance is likewise interpreted as the exact represented number, **not as the potentially different decimal literal or measurement the user originally intended**. Later reference-file ingestion MUST preserve decimal-to-binary conversion provenance where relevant. Exact findings MUST be independent of metric representability, metric rounding, and the caller's rounding mode. The diagnostic's rounding rule (including whether it follows the incoming rounding mode) MUST be fixed and documented before B1b acceptance; preserving the caller's rounding mode does not itself dictate the diagnostic's rounding rule.

The function signatures and result shapes are the accepted B1a implementation target. Subsequent changes require explicit contract review.

## 5. Argument validation, precedence, and failure outputs

1. A null `out_result` returns `BBTC_VV_STATUS_INVALID_ARGUMENT` without attempting an output write. A non-null result MUST first be cleared to a canonical, all-fields-zero state before examining any remaining argument.
2. A null policy pointer or an unsupported method (including zero and unrecognized enum values) returns `BBTC_VV_STATUS_INVALID_ARGUMENT`. These are structural checks; do not inspect the tolerance value yet.
3. Within a supported floating-point environment, numeric validation follows pointer and method/structural checks. Among the scalar `reference`, `observation`, and `tolerance`, any NaN MUST produce `BBTC_VV_STATUS_NAN_INPUT`, taking precedence over any infinity in this numeric validation layer; otherwise any infinity MUST produce `BBTC_VV_STATUS_NONFINITE_INPUT`. The sign or NaN payload MUST NOT alter the returned class.
4. Once finite values have been established, Exact requires a tolerance numerically equal to zero; Absolute and Reference-relative require a nonnegative tolerance. `-0.0` is acceptable as zero, whereas any finite negative tolerance is invalid (`BBTC_VV_STATUS_INVALID_ARGUMENT`).
5. A zero reference with Reference-relative mode returns `BBTC_VV_STATUS_OUTSIDE_DOMAIN`, even when both values are zero. Callers seeking zero-reference acceptance MUST choose Exact or Absolute explicitly.
6. After structural validation, operations requiring floating-point inspection MUST establish that the runtime/compiler/floating-point environment is supported before performing arithmetic or classification that could alter observable floating-point state. Unsupported or non-guaranteed environments MUST produce `BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT` rather than execute an unsound algorithm. This capability check MAY therefore precede numeric NaN/infinity checks when numeric inspection cannot safely occur; **within supported environments**, NaN retains precedence over infinity as in item 3.
7. On **every** non-success status, result fields MUST remain in their canonical cleared state: `NOT_EVALUATED`, `NOT_COMPUTED`, and a zero metric. On successful B1 comparisons, the finding MUST be either `PASS` or `FAIL`. The `INDETERMINATE` enumeration is reserved for later explicitly specified comparison methods and MUST NOT be emitted for supported, admissible B1 inputs.
8. `BBTC_VV_STATUS_NUMERICAL_FAILURE` is reserved for an unexpected internal execution failure, invariant violation, or inability to deliver B1's required exact decision in an environment that was claimed to be supported. It MUST NOT be used as a convenient substitute for ordinary threshold-boundary comparisons; such a failure blocks the relevant test or acceptance gate. Future methods MAY define genuine `INDETERMINATE` outcomes separately, but no such outcome is authorized for the initial three methods.

The validation-precedence list concerns this local API only. It does not alter BBTC production status precedence or define untrusted-pointer safety beyond what the C language permits. To protect sticky exception flags, implementation code MUST capture the required incoming floating-point environment before any operation that could modify it (including signaling-NaN classification). A null output or other early structural failure MUST preserve the incoming floating-point environment without requiring unnecessary floating-point work.

## 6. Numerical soundness and boundary discipline

### 6.1 The threshold decision is not the rounded diagnostic

For the Absolute and Reference-relative modes, the intended inequalities in section 3 concern the exact represented operand values, not the floating-point result of a naive subtraction, division, or multiplication. This distinction matters because a rounded subtraction may equal a tolerance even when the true difference is slightly larger.

For every **supported** format and **admissible** finite input, B1 MUST deliver the mathematically exact `PASS` or `FAIL` decision, including exact threshold equality, independent of intermediate rounding, excess precision, or the caller's rounding mode. A rounded metric MUST NEVER determine the finding on its own. Neither a false pass, a false fail, nor an `INDETERMINATE` escape hatch is permitted in these cases. An implementation that cannot establish the exact decision for the full claimed domain MUST narrow its declared support or reject the unsupported environment; it MUST NOT silently weaken the contract.

The concrete exact-decision algorithm is deferred to B1b. It MAY employ carefully verified binary-significand/exponent methods, exact bounded integer arithmetic, or another demonstrably equivalent approach; any use of rounded floating-point intermediates must come with an adequate correctness proof. **B1b MUST provide a reviewable soundness argument covering both absolute and relative inequalities**, including finite extreme exponents and products whose exact mathematical magnitude is not representable in the native type. Tests and agreement between two compilers alone are not proofs of the result.

### 6.2 Overflow, underflow, and numerical extremes

For finite input values, internal arithmetic that overflows or underflows MUST NOT create a false pass, return an accidental NaN-input status, or silently change the comparison criterion. For example, two finite values with opposite extreme signs may have an absolute separation larger than the scalar's largest finite representable number. A finite absolute tolerance cannot pass such a comparison, even if the diagnostic cannot store the separation.

For zero tolerance, equality MUST be decided by the original operands, not a rounded subtraction that might spuriously underflow to zero. If the numerical environment uses flush-to-zero or altered rounding semantics that break an algorithm's proof, the algorithm MUST account for them explicitly or return the environment status.

### 6.3 Exact equality and zero

For finite operands, Exact mode uses numerical equality (`==`). This makes `+0` and `-0` equal. Bitwise identity, payload equality, and signed-zero-sensitive physical semantics are separate questions and MUST NOT be implied by Exact mode. Relative comparison at a zero reference is prohibited as specified above; Absolute/Exact modes admit zero normally.

### 6.4 Supported binary formats and exact support gating

Initial support is **binary floating-point only**, and is a verified **compiler + target + scalar-family + numerical environment + build-flags** capability, not merely recognition of `sizeof` or `FLT_RADIX == 2`. B1b SHOULD prioritize native binary32, binary64, and the x87 extended `long double` used on the primary x86-64 GNU/Linux development platform; binary64-backed `long double` and IEEE binary128 MAY be enabled only after separate format/algorithm verification. Any primary target supported by B1b MUST provide the three native typed entry points; when a family cannot be certified, its entry point MUST reject that family/environment explicitly rather than narrowing it.

B1 MUST document `FLT_RADIX`, `FLT_EVAL_METHOD`, the relevant `*_MANT_DIG` and exponent limits, actual format when established, evaluation/excess-precision behavior, rounding modes, subnormal handling, and compiler contraction/optimization assumptions. A supported execution environment MUST have a reviewed exact-decision correctness argument for all admissible finite operand values in each enabled scalar family. Support for decimal, nonbinary, nonconforming, or insufficiently characterized formats is deferred.

### 6.5 Floating-point environment isolation

On **every normal return path in a supported environment**, including rejected arguments, the core MUST preserve both the caller's active rounding mode and the preexisting sticky exception flags exposed by supported `<fenv.h>` facilities. It MUST NOT clear preexisting flags, add new visible flags, change the rounding mode, or leak diagnostic arithmetic's effects into the caller's floating-point environment. The exact `PASS`/`FAIL` finding MUST be independent of all supported incoming rounding modes.

This is an observable contract, not merely a promise to call `fegetenv()`/`fesetenv()`. B1b MUST specify and test its save/restore strategy, `<fenv.h>` availability and semantics, supported optimization flags, FP contraction settings, and any necessary compiler controls. State capture MUST precede operations capable of raising exceptions (including signaling-NaN inspection). The implementation MUST distinguish restoration failure from an ordinary comparison result and must not silently report success if it cannot guarantee restoration; environments without reliable save/restore MUST be rejected rather than claimed supported.

Support for caller-enabled **unmasked floating-point traps** is deferred unless a separately reviewed implementation proves safe handling. Restoring flags after a trap does not undo the trap; unsupported trap configurations MUST be documented and excluded from guarantees. Compiler flags that invalidate the specified numerical or environment semantics (including unsafe fast-math transformations) are unsupported. B1b MUST verify the guarantees in optimized as well as unoptimized builds; compile-time capability macros by themselves do not constitute sufficient proof.

## 7. Verification of the comparison implementation

### 7.1 Independent positive and negative test oracles

Deterministic tests SHOULD use cmocka and MUST establish expected findings independently of the production comparison algorithm. Where feasible, use analytically derived dyadic examples and a separately implemented exact-rational or multiprecision test oracle; merely recomputing the same rounded formula in the test is unacceptable. Required cases for **all three native scalar families** include:

- Equal finite values; unequal exactly representable values; positive and negative zeros.
- Equality at absolute/relative tolerance zero; inclusive threshold cases whose correctness is provable exactly.
- Strictly inside and strictly outside bounds; negative and zero references; explicitly asymmetric reference-relative cases.
- Null policy and null output pointers; invalid enum values; invalid/negative tolerance; NaN and infinity in all scalar input positions; NaN precedence over infinity.
- Reference-relative comparison with a zero reference (including zero observation).
- Adjacent representable values around `1`, smallest normal and subnormal values when the target supports them, and values near maximal finite magnitudes.
- Oppositely signed extreme finite values whose exact separation exceeds the scalar's finite range.
- Inputs where a naive rounded subtraction misleadingly lands **on** the stated threshold although the true exact mathematical difference exceeds it.
- Every admissible boundary case MUST resolve to `PASS` or `FAIL` with no `INDETERMINATE`; deliberate fault injection MUST verify that unexpected numerical failures cannot count as passes.
- Output clearing on all failing statuses; no unintended mutation of input policy, rounding mode, or sticky floating-point exception flags; test pre-set flags and incoming rounding modes separately.
- Explicit behavior for unsupported environments, to the extent that relevant conditions can be constructed without undefined behavior.

A particularly useful adversarial case in a binary floating-point family with `p` significand digits is:

```text
reference = 2^p
observation = -1
absolute_tolerance = 2^p
```

The **exact** absolute difference is `2^p + 1`, strictly greater than the threshold. An ordinary native-precision subtraction may round to `2^p`. **This comparison MUST return `FAIL`** on every claimed-supported format, not `INDETERMINATE`. A corresponding Reference-relative case (`relative_tolerance = 1`) MUST likewise return `FAIL`, since the exact right side is `2^p`. These are mathematical adversarial tests, not experimental-ballistics references.

### 7.2 Additional tests and tooling

The mandatory B1 deterministic suite SHOULD run through CTest and must not require the production BBTC library. GCC and Clang strict builds, ASan, UBSan, and retained execution of the existing BBTC tests form the acceptance baseline. No sanitizer run is evidence of mathematical correctness by itself.

DeepState-generated arithmetic cases, libFuzzer byte-input cases, MPFR cross-checks, and mutation testing MAY be added later. MPFR MAY help independently check numerically difficult boundaries but MUST NOT be necessary to compile or run the initial core. Testing properties such as symmetry MUST account for method semantics: Reference-relative mode is generally **not symmetric**.

### 7.3 Reproducibility

Failure output and comparison diagnostics MUST be reproducible under a recorded supported floating-point environment. B1 tests SHOULD record compiler/toolchain family, optimization mode, `FLT_RADIX`, mantissa lengths, relevant floating environment, and scalar family in failure diagnostics. Identical bitwise results on every architecture are not promised.

B1 deterministic self-tests MUST additionally include (a) the same exact decision under every supported incoming rounding mode, (b) preservation of pre-set individual and combined sticky exception flags on success and failure returns, (c) zero exact metrics versus positive metrics rounded to zero, (d) exactly representable finite metrics versus finite rounded diagnostics, and (e) correct reporting of an otherwise sound finding when its scalar metric is outside finite range or unresolved. Unsupported or untestable platform-specific requirements MUST be made visible as capability limitations rather than silently skipped.

## 8. Staged source layout and build behavior

The first implementation SHOULD use a small source tree similar to:

```text
docs/validation/
    VVUQ_DESIGN_CONTRACT.md
    VVUQ_COMPARISON_CORE_CONTRACT.md   # this B1a document

tools/bbtc_vv/
    comparison.h
    comparison.c
    CMakeLists.txt

tests/validation/
    test_comparison.c
    CMakeLists.txt
```

A single implementation file MAY generate three family-specific implementations via carefully reviewed macros or explicit functions; however, generated code MUST not narrow computations or conceal divergent numerical behavior. Any helper must retain native family semantics.

B1b SHOULD add a guarded opt-in CMake option (candidate `BBTC_BUILD_VVUQ`) and a separate developer-test option or equivalent. When optional VVUQ tests are selected, missing cmocka SHOULD cause a clear configuration failure rather than an unnoticed zero-test run. The ordinary default BBTC build MUST remain unaffected and free of cmocka requirements. The implementation MUST be independently testable without linking `bbtc::bbtc`.

The exact target names and CMake option spelling remain subject to B1b implementation review. No parser, fixture interchange, physical-reference dataset, BBTC adapter, report CLI, or production API is part of this increment.

## 9. Review decisions and acceptance criteria

B1a may be accepted only when all of the following are explicitly reviewed:

1. The three modes have unambiguous mathematical definitions and no implicit scientific tolerance policy.
2. The documented native-family APIs do not narrow values through `double` or modify production BBTC headers.
3. Operation status, comparison finding, and diagnostic availability are distinct; failed operations always clear results.
4. Input validation, NaN/infinity precedence, signed zero, negative tolerances, reference zero, and exceptional inputs are precisely specified.
5. All admissible B1 comparisons in each claimed-supported format return **mathematically exact `PASS`/`FAIL`**, without false passes, false fails, or `INDETERMINATE` escapes. Exact threshold decisions are independent of metric rounding and supported caller rounding modes.
6. Extreme-magnitude and threshold-boundary tests include independently justified expected behavior, with the absolute and reference-relative `2^p + 1` adversarial examples returning `FAIL`.
7. Initial platform claims are restricted to verified binary formats/compiler environments, and unknown or unsupported family/environment combinations are rejected without precision narrowing.
8. Both rounding mode and sticky floating-point exception flags are preserved on every normal supported return path, including invalid inputs; unsupported trapping configurations and optimization modes are expressly excluded.
9. Metrics distinguish exact zero, representable exact and rounded finite values, overflowed magnitude, rounded-to-zero positive values, and unavailable diagnostics without altering the exact finding.
10. cmocka and optional fuzz tools are kept separate from the core and ordinary BBTC consumers.
11. The scope remains documentation-only, with no physics or public ABI edits and no claim of physical validation.
12. The final version is marked **Accepted** only after review and is committed separately before the B1b implementation.

## 10. Deferred questions

B1b MUST justify the concrete **mathematically exact decision algorithm** for each claimed-supported scalar family and show that no B1 comparison requires `INDETERMINATE`. It MUST separately select, document, and test the scalar metric rounding rule and exact-vs-rounded classification. It MUST demonstrate the floating-point state-preservation mechanism under GCC/Clang and supported optimization settings. Concrete algorithms, diagnostic state transitions, supported format probes, CMake target names, and optional test framework discovery details remain subject to that review; a platform is not 'supported' merely because it compiles.

Certified **lower/upper numerical-error enclosures** are explicitly required as the objective of a *later dedicated checkpoint*, **not** as fields or promises of B1's initial result type. The later checkpoint MUST be separately contracted and tested, including representability and open-ended-bound behavior. This deliberate deferral does not weaken B1's exact pass/fail requirement.

Later checkpoints will add independently derived reference oracles, BBTC adapters, provenance-aware case schemas, evidence and uncertainty layers, report generation, and empirical validation where trustworthy data exist. None is implied by the acceptance of this comparison-kernel contract.

---

**Accepted scope note:** This is review revision 2 of the accepted internal B1a comparison contract, incorporating Decisions 1–4: exact pass/fail; verified binary format support; diagnostic scalar fidelity with certified intervals deferred; and caller floating-point state preservation. VV0.1-B1a is accepted as documentation only; implementation remains pending. It does not claim empirical validation.
