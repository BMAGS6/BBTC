# VV0.1-B2b.3c.1 — Virial reference arithmetic proof and qualification profile

| Field | Record |
| --- | --- |
| Phase | VV0.1-B2b.3c.1 |
| Status | **PROPOSED SOURCE-LEVEL PROOF; COMPILER/ABI/FENV QUALIFICATION OPEN** |
| Prepared | 2026-11-09 |
| Reviewed source baseline | `e123e6192d57bf60558b7e7d7f67022bc2ad4cb5` |
| Development branch | `rewrite/vv0_1b2b_reference_adapter_implementation_v1` |
| Primary production source | `src/internal/first_order_virial_gas_model.c` |
| Reference test source | `tests/validation/test_virial_reference_matrix.c` |
| Accepted design | `docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md` v0.1.0, especially §§4, 6–9 |
| Reference provenance | `docs/validation/references/VV0_1_B2_REFERENCE_PROVENANCE.md` |
| Acceptance authority | Later explicit B2b review; **not** this record |

## 1. Purpose and limits

This record derives, from the **actual production source at the cited commit**, why
M-01 through M-04 are candidates for `BBTC_VV_COMPARISON_EXACT` with native
numerical zero tolerance. It records the required host and build conditions and
identifies evidence **not yet collected**. It does **not** alter production code,
change a comparison policy, approve a compiler, or assert physical-model accuracy.

The proof is deliberately narrower than a general Chebyshev-series correctness
proof. Its domain consists of exactly four bounded, degree-at-most-two cases,
three native C scalar families, and three returned coefficient-law quantities.
It cannot be extrapolated to arbitrary coefficients, temperatures, degrees,
calibration data, equations of state, or complete trajectories.

Four distinct questions must not be collapsed:

1. **Reference correctness:** are the declared expected real numbers justified
   independently of the production Clenshaw evaluator?
2. **Source arithmetic:** if the C source's relevant operations obey the stated
   floating-point semantics, are all intermediate results exactly representable?
3. **Compiled execution:** did a *particular* compiler, architecture, option
   set, and runtime environment actually preserve the necessary semantics?
4. **Scientific support:** what, if anything, does the numerical result establish
   about the physical model? (Here: **no empirical validation claim**.)

B2b.3c.1 addresses (1) and (2), specifies obligations for (3), and explicitly
excludes (4). The executable host checks belong to B2b.3c.2; captured build
and execution records and any qualification decision belong to B2b.3c.3.

## 2. Source boundaries and dependency audit

The three public evaluators are separate functions in
`src/internal/first_order_virial_gas_model.c`:

| Native family | Function | Source anchor at baseline |
| --- | --- | --- |
| `float` | `bbtc_ib_first_order_virial_temperature_law_evaluate_float` | Around line 111 |
| `double` | `bbtc_ib_first_order_virial_temperature_law_evaluate_double` | Around line 356 |
| `long double` | `bbtc_ib_first_order_virial_temperature_law_evaluate_long_double` | Around line 603 |

All three use the same **source-level** recurrence with appropriately suffixed
constants, native scalar locals, and native output structures. In particular,
their evaluation implementations neither route `long double` through `double`
nor compute the derivatives by finite differences. This conclusion is specific
to the reviewed revision. Source changes invalidate the audit until re-reviewed.

The B2 adapter (`tools/bbtc_vv/virial_adapter.c`) obtains observations by
calling the matching production public function. The independent reference
runner supplies expected values from the elementary polynomial identities
in §3, and the unmodified B1 `bbtc_vv_compare_*` API decides the finding.
These components have distinct responsibilities and result/status types.

**Build-graph finding, important:** the current `tools/bbtc_vv/CMakeLists.txt`
and `tests/validation/CMakeLists.txt` apply `-frounding-math`,
`-ffp-contract=off`, and `-fno-fast-math` to their own developer targets for
GCC/Clang. The production `bbtc` target in the top-level `CMakeLists.txt`
does **not** itself declare these target-specific flags. Global
`CMAKE_C_FLAGS` or a toolchain may supply them, but a green developer-only
CTest result does **not** prove the linked production object was compiled with
them. B2b.3c.2 MUST interrogate the producer's real compilation command.

## 3. Independent analytical reference, not an implementation oracle

Let `c0`, `c1`, and `c2` be the only possible nonzero Chebyshev coefficients.
For the mathematical model, independent of any Clenshaw implementation,

```text
T0(x) = 1               T1(x) = x          T2(x) = 2*x*x - 1
x = 2*(T - Tmin)/(Tmax - Tmin) - 1          s = 2/(Tmax - Tmin)
B(T)      = c0 + c1*x + c2*(2*x*x - 1)
dB/dT     = s*(c1 + 4*c2*x)
d2B/dT2   = 4*c2*s*s
```

All four cases have `Tmin = 256 K` and `Tmax = 512 K`; therefore the
mathematical range is `256 K` and `s = 1/128 K^-1 = 2^-7 K^-1`.
M-01, M-02, and M-03 use `T = 384 K`, hence `x = 0`. **M-04 uses `T = 320 K`**,
hence `x = -1/2`; its off-center point is mandatory, not a replaceable detail.

| Case | `N` | `(c0, c1, c2)` (m³/kg) | `x` | `B` (m³/kg) | `dB/dT` (m³/(kg·K)) | `d²B/dT²` (m³/(kg·K²)) |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| M-01 | 1 | `(0, —, —)` | `0` | `0` | `0` | `0` |
| M-02 | 2 | `(2^-10, 2^-12, —)` | `0` | `2^-10` | `2^-19` | `0` |
| M-03 | 3 | `(0, 0, 2^-12)` | `0` | `-2^-12` | `0` | `2^-24` |
| M-04 | 3 | `(0, 0, 2^-12)` | `-1/2` | `-2^-13` | `-2^-18` | `2^-24` |

The absent coefficients shown as `—` are **not** elements of the array; their
absence fixes the coefficient count and loop extent. Each reference value is
an exact rational number. Nonzero expectations are encoded using native-family
hexadecimal floating-point literals, and zero expectations using native zero
literals, in the B2b.3a/3b test. This table is a mathematical fixture, not a
measurement and not a producer-generated golden-output snapshot.

## 4. Proof against the actual Clenshaw source operations

### 4.1 Preconditions and temperature normalization

For every case and native family, the public law and temperature validations
succeed: the coefficient pointer has the declared readable extent, the
interval is strictly positive and ordered, and `T` lies inside the closed
interval. All inputs are finite. These assumptions are established by the
fixture and are separately covered by B2b adapter/status tests.

The production source computes, in this order:

```c
/* Schematic only; source is independently inspected per scalar family. */
range = Tmax - Tmin;
x = 2 * ((T - Tmin) / range) - 1;
s = 2 / range;
```

| Expression | M-01..M-03 | M-04 |
| --- | ---: | ---: |
| `Tmax - Tmin` | `256 = 2^8` | `256 = 2^8` |
| `T - Tmin` | `128 = 2^7` | `64 = 2^6` |
| `(T - Tmin)/range` | `1/2 = 2^-1` | `1/4 = 2^-2` |
| `2 * (...)` | `1` | `1/2` |
| `x = (...) - 1` | `0` | `-1/2` |
| `s = 2/range` | `1/128 = 2^-7` | `1/128 = 2^-7` |

All operands and exact results in this table are representable in the
candidate native binary formats. No intermediate is outside normal finite
range, and the division quotients terminate in base two. The source's clamp
of already-in-domain `x` to `[-1, 1]` is consequently inactive for these
selected cases; it does not conceal an approximation.

### 4.2 Clenshaw state and derivative recurrence

The implementation initializes the *value*, *first derivative with respect
to x*, and *second derivative with respect to x* auxiliary states to zero.
For `k = N-1` down to `1`, with two trailing states initially zero, it uses:

```text
b[k]     = 2*x*b[k+1] - b[k+2] + c[k]
b'[k]    = 2*b[k+1] + 2*x*b'[k+1] - b'[k+2]
b''[k]   = 4*b'[k+1] + 2*x*b''[k+1] - b''[k+2]
```

The implementation stores temporary `current_*` values before shifting the
`*_next` and `*_next_next` states; thus the right-hand sides use the preceding
step's values, not partially updated state.

**M-01:** `N=1`, so the loop is not entered. All auxiliary state remains
zero, and only `c0=0` contributes to the reconstructed result.

**M-02:** `N=2`; a single step `k=1` starts from zero:
`b1=c1=2^-12`, `b1'=0`, and `b1''=0`. The trailing `b2`, `b2'`, and `b2''`
states remain zero. There are no cancellations requiring rounding.

**M-03/M-04:** `N=3`. The first step, `k=2`, gives
`b2=c2=2^-12`, `b2'=0`, `b2''=0`. The next step, `k=1`, gives
`b1=2*x*2^-12`, `b1'=2*2^-12=2^-11`, and `b1''=0`.
The resulting states, including the zero `b2` derivatives, are:

| Case | `b1` | `b2` | `b1'` | `b2'` | `b1''` | `b2''` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| M-01 | `0` | `0` | `0` | `0` | `0` | `0` |
| M-02 | `2^-12` | `0` | `0` | `0` | `0` | `0` |
| M-03 | `0` | `2^-12` | `2^-11` | `0` | `0` | `0` |
| M-04 | `-2^-12` | `2^-12` | `2^-11` | `0` | `0` | `0` |

These are **exact real-arithmetic values of the source's intermediate
expressions**, not an additional implementation executed in the reference
runner. The table was also checked by a separate rational-arithmetic trace
while preparing this document. That check is supplementary, not independent
runtime qualification of compiled production objects.

### 4.3 Reconstruction of B and derivatives with respect to x

The production source's final expressions are:

```text
B       = x*b1 - b2 + c0
dB/dx   = b1 + x*b1' - b2'
d2B/dx2 = 2*b1' + x*b1'' - b2''
```

| Case | `x*b1` | `B` | `dB/dx` | `d²B/dx²` |
| --- | ---: | ---: | ---: | ---: |
| M-01 | `0` | `0` | `0` | `0` |
| M-02 | `0` | `2^-10` | `2^-12` | `0` |
| M-03 | `0` | `-2^-12` | `0` | `2^-10` |
| M-04 | `2^-13` | `2^-13 - 2^-12 = -2^-13` | `-2^-12 - 2^-12 = -2^-11` | `2^-10` |

The M-04 additions/subtractions are exact binary scalings and exact sums of
equal powers of two. No loss of low bits, overflow, or underflow is required
by the actual source operation order for any case in this table.

### 4.4 Temperature derivatives, explicit zero branches, and return status

The source conditionally writes the temperature derivatives only when the
corresponding x-derivative is numerically nonzero. The output structure is
initialized to zero before evaluation. Therefore a zero derivative leaves
its output field zero; an omitted multiply is **not** an unverified result.
For nonzero derivatives, the source evaluates:

```text
dB/dT   = (dB/dx) * s
d2B/dT2 = (d2B/dx2) * s * s     /* two sequential multiplications */
```

| Case | `dB/dT` | First `d²B/dx² * s` factor | `d²B/dT²` |
| --- | ---: | ---: | ---: |
| M-01 | zero branch | zero branch | zero branch |
| M-02 | `2^-12 * 2^-7 = 2^-19` | zero branch | zero branch |
| M-03 | zero branch | `2^-10 * 2^-7 = 2^-17` | `2^-17 * 2^-7 = 2^-24` |
| M-04 | `-2^-11 * 2^-7 = -2^-18` | `2^-10 * 2^-7 = 2^-17` | `2^-17 * 2^-7 = 2^-24` |

Every nonzero output is finite and normal under the candidate formats in §5;
the `isfinite` guards consequently succeed in the stated profile. Under the
source-level preconditions and arithmetic assumptions, each native evaluator
returns `BBTC_STATUS_SUCCESS` and returns the three mathematical expectations.

### 4.5 Why individual operations are exact in the candidate formats

The original temperatures `320 K` and `384 K` are small, exactly
representable integers (not powers of two). The remaining finite nonzero
inputs and *arithmetic intermediates* above are powers of two, or exact
addition/subtraction results that reduce to a power of two. The largest
input magnitude is `512 = 2^9`, and the smallest nonzero produced value is
`2^-24`. The source-level arithmetic trace needs at most one significant
binary digit for each nonzero intermediate after temperature subtraction.
Thus any correctly rounded radix-two operation at the stated precision and
exponent range returns the mathematical result without numerical rounding
error. This includes division by `256` and the two sequential multiplies by
`2^-7`, because the quotients/products are exactly representable and normal.

A conforming implementation may carry extra evaluation precision, but extra
precision does not change an *already exact, representable* intermediate.
It is **not** a license to ignore altered compiler expression semantics,
reassociation, unsafe finite-math assumptions, floating traps, or a different
library object; those remain part of the profile audit in §6. This proof
establishes numerical value equality, **not** bitwise identity of signed zero,
structure padding, or all floating-point status flags.

## 5. Candidate supported scalar representations (not yet certified)

An eligible profile MUST record the actual `bbtc_precision_info` output and
confirm correspondence with the compiler's `<float.h>`/`sizeof` properties.
It must also identify a supported native representation (not merely infer one
from storage size), and ensure all three scalar paths execute natively.

| Family | Candidate radix/significand | Candidate finite exponent capability | Notes |
| --- | --- | --- | --- |
| `float` | radix 2; `FLT_MANT_DIG=24` | Includes at least `[2^-24, 2^9]` | Typical binary32; verify rather than assume |
| `double` | radix 2; `DBL_MANT_DIG=53` | Includes at least `[2^-24, 2^9]` | Typical binary64; verify rather than assume |
| `long double` | radix 2; `LDBL_MANT_DIG` in `{53,64,113}` | Includes at least `[2^-24, 2^9]` | Distinguish binary64 alias, extended, and binary128-like semantics |

The B2b.3a runner already screens radix, mantissa and exponent macros,
`bbtc_precision_info`, scalar storage size, and `fegetround() == FE_TONEAREST`.
That is **screened but unqualified**, exactly as its current
`BBTC_VV_REFERENCE_PROFILE_SCREENED_UNQUALIFIED` enumerator says. It does not
detect every compiler transformation, target-specific option, excess-precision
choice, hardware mode, or trap state. Neither a matching `sizeof(long double)`
nor an accepted B1 comparator profile alone certifies the production evaluator.

The narrow source proof may remain true in more formats or rounding modes,
because the relevant operations are exactly representable; nevertheless, the
**approved test profile** is intentionally narrower. New configurations must
receive explicit independent review rather than be inferred from successful
execution or silently assigned a tolerant comparison policy.

## 6. Required compiled-execution profile and evidence

A candidate run MUST be associated with an immutable source identity and the
actual compiler and runtime environment. The following are **requirements to
verify**, not assertions already satisfied by this document:

1. **Source identity:** record source commit, source/blob identity for the
   production evaluator, adapter, comparison API, and runner. Any change to
   those inputs requires an impact assessment and, when relevant, re-proof.
2. **Toolchain identity:** compiler executable path, `--version`, compiler
   target triple, architecture, relevant compiler and linker options, and
   effective C23 mode. GCC and Clang observations are distinct profiles.
3. **Producer-specific flags:** inspect `compile_commands.json` (enable
   `CMAKE_EXPORT_COMPILE_COMMANDS=ON`) and record the command for
   `src/internal/first_order_virial_gas_model.c`. Require a justified strict
   floating-point configuration for the **producer object**, not just the
   VVUQ runner: `-frounding-math`, `-ffp-contract=off`, `-fno-fast-math`, and
   absence of overriding unsafe options. Explicitly inspect the **effective**
   flags, including inherited, toolchain, and later command-line overrides.
4. **Adapter/comparator flags:** independently capture actual compile commands
   for `virial_adapter.c`, `comparison_api.c`, and the reference runner.
   Do not assume a flag on one target propagates across a static-library link.
5. **Scalar ABI:** record `FLT_RADIX`, `FLT_MANT_DIG`, `DBL_MANT_DIG`,
   `LDBL_MANT_DIG`, exponent ranges, `sizeof` each native type, and the linked
   library's `bbtc_precision_info` values. Establish the actual representation
   and calling convention when needed (not just the ABI label).
6. **Floating-point environment:** verify `FE_TONEAREST` for the supported
   test profile, relevant exception-mask/trap and denormal handling, and any
   source/compiler-specific fenv assumptions. Explicitly reject or mark
   unsupported an environment not covered by the argument. Avoid claiming
   that the existing test's one `fegetround` check covers all environment
   state.
7. **Compiled behavior:** review generated instructions or other credible
   compiler evidence for the relevant source operations, normalization,
   derivative branches, return status and native-family paths. Confirm there
   are no unreviewed transformations that invalidate §4. Merely comparing
   output numbers cannot prove the compiled intermediate path.
8. **Positive/negative controls:** execute M-01–M-04 in all three families,
   verify the 36 EXACT findings, check the B2b.3b deliberately wrong expected
   values produce numerical FAIL, and verify invalid references and producer
   failures never become valid comparisons.
9. **Isolation:** the verification harness must remain developer-only; a
   production-only `bbtc` build must not acquire VVUQ targets, dependencies,
   installed headers, or test-only behavior.
10. **Provenance:** retain commands, outputs, versions, flags, case identities,
    status axes, failures, and reviewer disposition in a versioned evidence
    record. Do not interpret a passing test as physical safety assurance.

### 6.1 Suggested reproducible capture procedure (next increment)

For each configuration, start from a clean, separately named CMake build tree.
The **producer** must see the intended flags; the example uses global flags
because the existing production `bbtc` target has no strict-FP target options.
Do not modify the accepted production defaults merely for this audit.

```sh
# Illustration for an independently named strict GCC/Clang profile build.
# Use a fresh directory per compiler and per sanitizer configuration.
cmake -S . -B "$BUILD" -G Ninja \
    -DCMAKE_C_COMPILER="$CC" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_C_FLAGS='-O1 -g -frounding-math -ffp-contract=off -fno-fast-math' \
    -DBBTC_BUILD_TESTS=ON \
    -DBBTC_BUILD_VVUQ=ON \
    -DBBTC_VVUQ_BUILD_TESTS=ON \
    -DBBTC_VVUQ_ORACLE_TESTS=OFF
cmake --build "$BUILD"
ctest --test-dir "$BUILD" --output-on-failure
```

This is an **evidence-collection recipe**, not a claim that the commands have
been executed for B2b.3c.1. Use `compile_commands.json` to read the full
producer command and record any additional toolchain flags. The build's
compiled object is the evidence subject; the CMake cache by itself is not.

**Review checklist:** an acceptable evidence record names its exact library
object, compiler, target, ABI, flags, rounding/exception environment, the
reviewed arithmetic proof revision, all 12 case/family identities, all 36
findings, nine deliberate mathematical FAIL controls (three quantities in
each native family), and the observed contract/integrity test outcomes.
Unknowns require an explicit **UNRESOLVED** or **UNSUPPORTED** disposition.

## 7. Existing verification evidence and outstanding gaps

At the B2b.3b checkpoint, the project owner reported the following local tests
on 2026-11-09: GCC 16.2.1 Debug 29/29 CTest entries; Clang 23.1.4 with
`-O1` and strict floating-point flags 29/29; and Clang 23.1.4 with ASan/UBSan
29/29. The reference runner additionally reported 10/10 internal cmocka tests,
and production-only target isolation passed. These are **implementation
regression observations** from the specified development host, not a reviewed
B2b.3c qualification transcript. This record does not assert that strict
flags reached the producer in the GCC Debug run.

Open before *any* scoped EXACT qualification decision:

- Capture and check actual production-object compile commands for each build.
- Record actual compiler target, scalar formats, and exception/fenv behavior.
- Review relevant compiled instructions or equivalent credible toolchain
  evidence alongside the source operation-by-operation argument.
- Execute and record explicit supported/unsupported profile controls, without
  converting an unsupported host into a mathematically passing reference.
- Preserve the 36 comparison decisions and the separate negative controls.
- Complete independent review, publish evidence, and make an explicit scoped
  acceptance or non-acceptance decision at B2b.3c.3/B2b review time.

## 8. Change control and intended next actions

B2b.3c.1 adds only documentation. It introduces **no** public APIs, production
library changes, new CMake options, altered B1 comparison methods, or new
mandatory build dependencies. A documentation-only patch cannot be treated as
an executable qualification test; existing regressions remain required when
later implementation patches are introduced.

**Next: B2b.3c.2.** Add developer-only, heavily Doxygen-documented checks that
record and reject unsupported native/rounding profiles, and strengthen the
required-case evidence boundary without changing production behavior. Capture
compiler commands and actual environment information rather than guessing.

**Then: B2b.3c.3.** Review the source proof, compiled behavior and evidence
across each admitted compiler/ABI/fenv profile. Record explicit support scope,
limitations, reviewer sign-off, and whether B2b can proceed to its separately
scoped acceptance review. No empirical physical validation or safety
certification follows from this mathematical record.
