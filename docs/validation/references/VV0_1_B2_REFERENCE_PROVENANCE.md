# VV0.1-B2 — Mathematical reference case provenance index

| Field | Record |
| --- | --- |
| Status | **DRAFT IMPLEMENTATION EVIDENCE — NOT EMPIRICAL VALIDATION** |
| Prepared | 2026-11-09 |
| Scope | M-01 through M-04; first-order virial coefficient temperature law |
| Source baseline | `e123e6192d57bf60558b7e7d7f67022bc2ad4cb5` |
| Design authority | `docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md` v0.1.0 §4 |
| Arithmetic proof | `docs/validation/VV0_1_B2B3C1_VIRIAL_EXACT_ARITHMETIC_PROOF.md` |
| Runtime fixtures | `tests/validation/test_virial_reference_matrix.c` |

## 1. Provenance and independence

These are deterministic **MATHEMATICAL** identities of the represented
Chebyshev coefficient law. They are neither experimental observations nor
fit-derived model truth. Their expected values are obtained from the first
three Chebyshev polynomials and elementary differentiation, *not* from the
production Clenshaw recurrence. Production observations are obtained through
`tools/bbtc_vv/virial_adapter.c`, and the independent developer-only B1
comparison API evaluates numerical equality.

Every case has identity revision **1** and `required = true`. The initial
matrix consists of four case IDs, three native scalar families (`float`,
`double`, `long double`) and three output quantities: **36 comparison findings**
when the required environment is supported. Required-case coverage may not
be inferred merely from an aggregate count; the B2b.3b runner checks identity
and precision completeness independently.

The common temperature-law domain is `Tmin = 256 K`, `Tmax = 512 K`. Every
coefficient and `B(T)` uses **m³/kg**. The first derivative uses
**m³/(kg·K)**; the second derivative uses **m³/(kg·K²)**. The coefficient
array is borrowed, readable and immutable for the duration of each call.
Nonzero expected scalars use native-family hexadecimal floating-point
literals; zero expectations use native zero literals.

## 2. Frozen mathematical example identities

| Case and revision | Temperature | Exact coefficient array (m³/kg) | Expected `B` | Expected `dB/dT` | Expected `d²B/dT²` |
| --- | ---: | --- | ---: | ---: | ---: |
| M-01 r1 | `384 K` | `[0]` | `0` | `0` | `0` |
| M-02 r1 | `384 K` | `[2^-10, 2^-12]` | `2^-10` | `2^-19` | `0` |
| M-03 r1 | `384 K` | `[0, 0, 2^-12]` | `-2^-12` | `0` | `2^-24` |
| M-04 r1 | **`320 K`** | `[0, 0, 2^-12]` | `-2^-13` | `-2^-18` | `2^-24` |

The `B` column and both derivative columns use the units declared above.
M-04 is an intentionally off-center case; substituting 384 K changes its
meaning and is not a permissible test simplification.

For all cases, define `x = 2*(T-256 K)/(256 K) - 1`, `s = 2/(256 K)`, and
`T0 = 1`, `T1 = x`, `T2 = 2*x*x-1`. Then:

```text
B      = c0 + c1*x + c2*(2*x*x-1)
dB/dT  = s*(c1 + 4*c2*x)
d2B/dT2 = 4*c2*s*s
```

An absent `c1` or `c2` is not a readable coefficient: the array length is
1 for M-01, 2 for M-02, and 3 for M-03/M-04. This affects the production
loop bound and must remain consistent with the fixture's metadata.

## 3. Required evidence fields and admissibility

Each case/family/quantity comparison is admissible only when:

1. The case ID and revision match the accepted inventory; the case is required.
2. Temperature, domain, coefficient count, and actual readable coefficient
   storage are consistent with the mathematical example.
3. The expected quantity and unit are explicitly present and match the
   canonical identity; the expected numerical value is finite.
4. The matching native-family adapter reports production
   `BBTC_STATUS_SUCCESS` and a valid numerical observation.
5. The declared compiler/ABI/FP environment is supported by a reviewed
   qualification record, not merely screened by the current runner.
6. B1 `bbtc_vv_compare_*` returns `BBTC_VV_STATUS_SUCCESS` under
   `BBTC_VV_COMPARISON_EXACT` with native zero tolerance; its distinct finding
   is `PASS` or `FAIL` as warranted.

There is no uncertainty estimate for these exact mathematical identities.
An empirical uncertainty or a physical applicability claim may not be
invented. Exact equality on four examples is **not** complete numerical
correctness of the evaluator over its full domain, and it gives no evidence
of experimental validity or firearm/ammunition safety.

## 4. Negative controls and evidence classification

B2b.3b deliberately perturbs each of M-04's three expected fields for each
native precision family. Each incorrect expectation must yield an operationally
successful B1 comparison with **numerical `FAIL`**. Passing that test means
the negative control correctly detected a false reference; it is **not** a
successful mathematical reference result. Bad metadata, incompatible units,
missing or nonfinite expectations, an unsupported profile, or a failed
production call must not reach numerical comparison.

A runner may count 36 successful comparisons only after every required
case/family/quantity is present, executed, admissible, and numerically passed.
Record producer status, reference integrity, comparison operation status,
mathematical finding, environment support and scientific inference separately.

## 5. Qualification status and future maintenance

The **source-level** arithmetic argument is recorded in
`VV0_1_B2B3C1_VIRIAL_EXACT_ARITHMETIC_PROOF.md` for the cited baseline. The
actual compiler/ABI/fenv **qualification remains unresolved** pending
B2b.3c.2 executable checks and B2b.3c.3 captured evidence and review.

Changes to expected values, case temperatures, polynomial order, unit
identity, production arithmetic, coefficient ownership or native scalar types
require explicit provenance revision and reassessment. The existing B2a design
acceptance does not automatically approve revised implementation evidence.
