# BBTC VVUQ Independent References and Adapter — Design Contract

| Metadata                         | Value                                                           |
| -------------------------------- | --------------------------------------------------------------- |
| Contract version                 | **0.1.0**                                                       |
| Project phase                    | **VV0.1-B2a**                                                   |
| Status                           | **ACCEPTED — DESIGN ONLY; B2 IMPLEMENTATION NOT YET QUALIFIED** |
| Original draft date              | **2026-11-03**                                                  |
| Revision 2 date                  | **2026-11-05**                                                  |
| Owner approval date (US Eastern) | **2026-11-05**                                                  |
| Owner approval date (UTC)        | **2026-11-06**                                                  |
| Review authority                 | **BMAGS project owner; scoped design acceptance**               |
| Reviewed design commit           | `a9ae0c60a5562c5d7911428004d301ec3d106f39`                      |
| Acceptance branch                | `rewrite/vv0_1b2a_reference_adapter_design_v1`                  |
| Accepted B1 source baseline      | `af2a7591d74bd27b764268dd65f8c1b367bc0164`                      |
| Accepted prior checkpoint        | `vv0.1-b1-accepted-20261030`                                    |
| Acceptance decision record       | `docs/validation/VV0_1_B2A_FINAL_ACCEPTANCE_DECISION.md`        |
| Governing scientific contract    | `docs/validation/VVUQ_DESIGN_CONTRACT.md` v0.1.0                |
| Governing comparison contract    | `docs/validation/VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0       |
| Governing production contract    | `docs/design/BBTC_DESIGN_CONTRACT.md` v0.1.24                   |

> **Scoped acceptance boundary:** VV0.1-B2a accepts the documented reference-case
> and developer-only adapter **architecture**, not implementation code, executable
> qualification, or scientific performance. This decision neither changes
> production behavior nor authorizes an unreviewed public API, and it does not
> certify numerical behavior on new hosts, physical-model accuracy, ammunition,
> firearm, or structural safety. See the linked final acceptance decision.

## 1. Purpose and precedence

VV0.1-B2a is a **documentation-only design checkpoint** for the first set of
independently derived reference cases and a developer-only adapter that obtains
observations through existing public BBTC APIs. Its intended successor will be
an independently reviewed C23 implementation checkpoint, not an implicit part
of B2a acceptance.

The accepted contracts identified above remain authoritative. The terms
**MUST**, **MUST NOT**, **SHOULD**, **SHOULD NOT**, and **MAY** are normative
within this accepted design contract. A disagreement with an accepted contract must be documented
and resolved explicitly; a B2a decision cannot silently weaken an earlier
requirement.

The system must distinguish **(a) whether the program ran**, **(b) whether a
reference was well established**, **(c) whether a numerical comparison passed**,
and **(d) what scientific inference that finding permits**. The adapter and
reference cases do not turn numerical verification into experimental validation.

## 2. Scope and non-goals

### 2.1 Included in the B2a design

1. A source-identified inventory of suitable **already implemented public**
   interfaces, with the limits of each candidate recorded.
2. Four initial **MATHEMATICAL** reference examples for the public Chebyshev
   temperature-law evaluator, independently derived from polynomial identities,
   including a noncentral quadratic case.
3. **Separate** contract/status checks for validation, output clearing,
   return-status precedence, and precision metadata; the initial implementation
   increment MUST plan both the mathematical and contract/status families, while
   preserving their different evidentiary meaning.
4. Conceptual case identity, revision, units, provenance, scalar-family and
   applicability metadata, without freezing a wire representation.
5. A developer-only adapter boundary, data ownership and failure semantics.
6. Runner-coordinated integration with the accepted B1 comparison mechanism,
   using **EXACT only for prequalified arithmetic paths** and explicitly
   justified policies per case and quantity.
7. Test isolation, qualification scope, and acceptance criteria.

### 2.2 Excluded from B2a

- C implementation, API/ABI modifications, numerical kernel changes, or tests.
- Changes to the accepted B1 baseline, acceptance tag, or evidence transcript.
- A complete internal-ballistics solver, coupled physics, external trajectory
  calculations, new constitutive models, or production defaults.
- Empirical model-validation claims, fabricated measurements, calibration
  data, or physical safety determinations.
- A frozen JSON schema, parser, fixture interchange, hashes, canonicalization,
  comprehensive report serialization, CI policy, or uncertainty propagation
  engine: those require their own reviews, primarily in VV0.1-C or later.

## 3. Source-grounded public-API survey

The following statements refer to the **exact B1 acceptance commit** above,
not to an uninspected future branch or an uncommitted local modification.

| Candidate | Source evidence | Functionality present | Initial role | Limit |
| --------- | --------------- | --------------------- | ------------ | ----- |
| Virial coefficient temperature law | `include/bbtc/internal_ballistics/first_order_virial_gas_model.h`, `src/internal/first_order_virial_gas_model.c`, `tests/test_ib_first_order_virial_gas_model.c` | Native `float`, `double`, `long double` evaluation of `B(T)` and two derivatives | **Primary mathematical reference subject** | A coefficient-law check, **not** experimental EOS validation |
| Geometry record validator | `include/bbtc/internal_ballistics/geometry.h`, `src/internal/geometry.c`, `tests/test_ib_geometry.c` | Domain and input-status classification for three scalar families | Negative-path/adapter semantics | No numerical output to compare as a physical observable |
| Precision metadata | `include/bbtc/precision.h`, `src/precision.c`, `tests/test_precision.c` | Scalar format metadata and status | Record/check native family; adapter smoke test | Host characteristics, not independent physical reference |
| Noble–Abel thermodynamics | `include/bbtc/internal_ballistics/reduced_gas_thermodynamics.h`, `src/internal/reduced_gas_thermodynamics.c`, `tests/test_ib_reduced_gas_thermodynamics.c` | Native thermodynamic evaluation | Secondary candidate, **deferred** | More model assumptions and output semantics to review |
| B1 comparison kernel | `tools/bbtc_vv/comparison.h`, `tools/bbtc_vv/comparison_api.c` | Developer-only exact/absolute/reference-relative comparisons | Reuse unchanged | Does not validate units, reference independence, or evidence |

**Verification still required in the implementation checkpoint:** inspect the
exact public definitions, output-clearing guarantees, status ordering, scalar
representations, and production tests for every selected interface. The B2a
survey is selection evidence, not proof of every edge-case behavior.

## 4. Initial independent mathematical reference family

### 4.1 Subject, dimensions, and ownership

The public temperature law represents a second density virial coefficient
using Chebyshev polynomials of the first kind:

\[
B(T) = \sum_{k=0}^{N-1} c_k\,\mathcal T_k(x),\qquad
x=\frac{2(T-T_{\min})}{T_{\max}-T_{\min}}-1.
\]

- `T`, `T_min`, and `T_max` are **absolute temperatures in kelvins**.
- `B(T)` and each `c_k` have units **m³/kg**.
- `dB/dT` has units **m³/(kg·K)**.
- `d²B/dT²` has units **m³/(kg·K²)**.
- The coefficient array is borrowed, caller-owned, immutable during evaluation,
  and must remain valid for the entire public API call.
- The public API evaluates three native scalar families independently; the
  adapter must not route `long double` through `double`.
- A reference is an identity of the **declared mathematical model**, not
  experimental evidence for the correctness of the physical model.

### 4.2 Analytical derivation independently usable as an oracle

For just the first three Chebyshev terms, use the elementary polynomial
identities

\[
\mathcal T_0(x)=1,\quad \mathcal T_1(x)=x,\quad
\mathcal T_2(x)=2x^2-1.
\]

With `s = 2/(T_max - T_min)`, the exact real-valued expectations are

\[
B=c_0+c_1x+c_2(2x^2-1),\quad
\frac{dB}{dT}=s(c_1+4c_2x),\quad
\frac{d^2B}{dT^2}=4c_2s^2.
\]

These expectations arise from **polynomial identities and differentiation**,
not by executing BBTC's Clenshaw recurrence or copying BBTC test outputs.
The future reference implementation must be sufficiently distinct from the
production routine to avoid systematic shared algorithm defects.

### 4.3 Accepted mathematical case directions, not yet frozen fixture records

For **M-01 through M-03**, select `T_min = 256 K`, `T_max = 512 K`,
and `T = 384 K`. Then `x = 0` and `s = 2^-7 K^-1` as exact mathematical
values. The following powers-of-two samples give analytical expectations;
**M-04 uses an explicitly different evaluation temperature**, described
below the table:

| Example                                     | Coefficients (in m³/kg)                | Expected `B` (m³/kg)      | Expected `dB/dT` (m³/(kg·K))     | Expected `d²B/dT²` (m³/(kg·K²))    |
| ------------------------------------------- | -------------------------------------- | ------------------------- | -------------------------------- | ---------------------------------- |
| M-01: zero law                              | `c0 = 0`, `N = 1`                      | `0`                       | `0`                              | `0`                                |
| M-02: linear law                            | `c0 = 2^-10`, `c1 = 2^-12`             | `2^-10`                   | `2^-19`                          | `0`                                |
| M-03: quadratic law at midpoint             | `c0 = 0`, `c1 = 0`, `c2 = 2^-12`       | `-2^-12`                  | `0`                              | `2^-24`                            |
| M-04: quadratic law away from midpoint      | `c0 = 0`, `c1 = 0`, `c2 = 2^-12`       | `-2^-13`                  | `-2^-18`                         | `2^-24`                            |

**M-04 deliberately uses a distinct evaluation condition:** keep
`T_min = 256 K`, `T_max = 512 K`, but set `T = 320 K` (rather than 384 K).
Then `x = -1/2` and `s = 2^-7 K^-1`. Since
`\mathcal T_2(-1/2) = -1/2`, the analytical values are

\[
B=-2^{-13},\qquad
\frac{dB}{dT}=-2^{-18},\qquad
\frac{d^2B}{dT^2}=2^{-24}.
\]

M-04's temperature override is part of its distinct reference identity; it
MUST NOT be silently replaced by the midpoint condition for M-01–M-03.

These numbers follow independently from the identities in §4.2. They are
candidate **mathematical** expectations, **not** physical measurements, and
must not be given invented empirical uncertainties. The exact real results
and all chosen input scalars are binary-representable in the previously
qualified formats; however, that fact **alone** does not prove the production
algorithm returns those results bit-for-bit.

**Proposed initial comparison policy:** all three numerical output fields of
M-01 through M-04 select `BBTC_VV_COMPARISON_EXACT` with a numerically zero
native-family tolerance, but ONLY after the successor implementation checkpoint
establishes and checks:

1. The actual compiler/ABI/format/floating-point profile is qualified; binary
   radix, exponent range, significand precision, and native type are verified.
2. Every relevant intermediate of the actual public implementation (including
   temperature normalization, the Clenshaw recurrence, derivative scaling,
   and the source's explicit zero branches) is exact for these selected inputs,
   or an equivalent bounded argument establishes the same final exact result.
3. The relevant compile flags, floating-point environment assumptions,
   contraction behavior, and input constant conversions are recorded and
   compatible with the argument. The accepted B1 host profile is evidence for
   **B1 comparison**, not automatic qualification of every B2 producer path.
4. Independent positive/negative controls verify that `EXACT` is applied to
   **every** selected quantity and that an incorrect expected value gives FAIL.

If the argument or profile qualification fails, the case MUST be reported as
unsupported or unresolved for that profile until an **independently justified,
predeclared** alternative policy receives explicit review. Never silently
change EXACT to a loose tolerance after seeing a failed result. Do not promise
cross-platform bitwise determinism from mathematical representability alone.

**Source-level exactness screening (not implementation qualification):** At the
accepted source revision, the three native evaluators share the same bounded
Clenshaw recurrence and affine temperature normalization. For M-01 through
M-04, `T_max - T_min = 256 K`, `T - T_min` is respectively `128 K` or `64 K`,
`x` is `0` or `-1/2`, and `s = 2^-7 K^-1`. With at most three coefficients,
the recurrence states, polynomial reconstruction, and both derivative scalings
reduce to zero or signed powers of two. Their nonzero intermediate values are
exactly representable and remain in the normal range of the previously
qualified binary32, binary64, and x87 extended profiles. This is a
**conditional argument from inspected source and mathematical inputs**, not
new compiled B2 verification evidence: the successor implementation checkpoint
MUST confirm the actual compiler, optimization flags, floating-point behavior,
and all three native output paths before making an EXACT qualification claim.

The minimal proposed matrix is **four mathematical cases × three native scalar
families × three output fields = 36 comparison decisions** on a fully
qualified profile. This count is a design target, **not** executed test evidence.

Additional cases for endpoints, nonzero coefficients at `x = ±1`, higher
polynomial degree, cancellation, and conditioning remain deferred until an
error/conditioning rationale and resource limits are reviewed.

### 4.4 Distinct contract/status cases — included in the first increment

The first B2 implementation increment MUST plan the contract/status checks
**alongside** the four mathematical cases, but MUST keep their case identity,
execution classification, and evidence claims separate. These checks verify
public API behavior, **not** independent physical reference values.

The initial contract-check inventory is:

| Family | Test condition | Contractually expected behavior |
| --- | --- | --- |
| Required output | Null `terms` output pointer | `BBTC_STATUS_INVALID_ARGUMENT`; no invalid output access |
| Required law | Non-null `terms`, null law pointer | Output fields cleared; `BBTC_STATUS_INVALID_ARGUMENT` |
| Borrowed coefficients | Null coefficient pointer or zero count | Documented argument/domain status; no dereference of a null coefficient pointer |
| Temperature law | Nonpositive or inverted temperature interval | `BBTC_STATUS_OUTSIDE_DOMAIN` |
| Nonfinite law inputs | NaN and infinity in bounds/coefficient array | `BBTC_STATUS_NAN_INPUT` takes precedence over infinity **within the documented law-validation layer** |
| Evaluation temperature | NaN, infinity, or finite value outside the closed law interval | Documented distinct status after successful law validation; non-null output cleared |
| Valid evaluation | M-01 through M-04, all native families | `BBTC_STATUS_SUCCESS`; all three documented outputs must be compared |
| Output lifetime | Previously populated output followed by a failing call | Non-null output's **documented fields** cleared before return; test fields, not padded structure bytes |
| Adapter propagation | Every selected success/error path | Preserve the exact returned `bbtc_status_e`; do not synthesize a comparison finding on a failure |

Each test MUST use **valid, readable object storage** for every pointer that is
not intentionally null. A non-null coefficient pointer does not license
reading beyond the supplied allocation; malformed pointer/count combinations
that would cause out-of-bounds access MUST NOT be introduced as tests of the
production evaluator. The runner must not attempt numerical comparisons of
cleared failure outputs.

**Precedence is local, not globally invented:** the evaluator first checks a
null output pointer, then clears a non-null output and validates the law,
then checks the evaluation temperature. It must preserve the public
function's documented precedence when more than one layer is invalid. The
initial tests must use the exact source revision and its public header as the
authority for the status of each case; they must not infer an undocumented
precedence across all production APIs.

A separate **comparison-system negative control** MUST deliberately perturb at
least one mathematically correct M-01–M-04 expected quantity by a nonzero,
exactly representable amount (e.g. `2^-20` where it changes the expectation),
run the unchanged accepted B1 EXACT comparator, and require a **FAIL** finding.
This deliberate failure is a passing *self-test* only when its expected FAIL
is observed; it MUST NEVER be counted as a passing mathematical reference
case. Additional negative controls must cover malformed reference metadata,
missing required cases, and an inadmissible comparison request in the future
runner's own validation layer, without prematurely implementing VV0.1-C
fixture parsing.

Every expected status and output-clear requirement MUST cite the corresponding
public contract and be verified against the precise source revision.

## 5. Reference-case identity and scientific integrity

Each future case must conceptually specify at least:

1. **Case identity:** stable ID, revision, future schema version, intended
   required/optional designation, and evidence class.
2. **Subject:** exact public API and output field, mathematical model,
   quantity of interest, physical dimension, and canonical unit.
3. **Conditions:** exact input identities, native scalar family, temperatures,
   coefficient values and ownership, domain, sign and reference-state meaning.
4. **Reference:** independent derivation, producer/source, version,
   mathematical expected value/property, and arithmetic/rounding semantics.
5. **Comparison:** named B1 method, predeclared threshold, units when absolute,
   and technical reason a particular criterion supports the proposed claim.
6. **Evidence limitations:** reference category, known sources of uncertainty,
   unknowns explicitly marked **unknown**, independence limitations, and
   applicability domain.
7. **Reproduction:** BBTC source revision, model/adapter revision, relevant
   build flags, floating-point environment, and future raw/normalized identity.

A case ID is not a data-file hash; a case revision is not the BBTC commit; and
an evaluation-run ID is not an accepted scientific claim. Future fixture
schemas must preserve all of these distinctions.

Do not silently serialize all values as binary64. Do not equate zero
mathematical uncertainty in an analytical identity with zero real-world model
uncertainty or zero numerical implementation error.

### 5.1 Minimal runtime representation versus full scientific metadata

The initial adapter/runner implementation SHOULD introduce only the typed
runtime data necessary to execute the approved four-case mathematical matrix
and contract/status checks. This includes: selected public API identity,
case ID/revision, scalar family, caller-owned inputs and their readable lengths,
returned raw BBTC status, selected/valid output fields, reference value,
comparison policy and B1 operation status/finding, and required-case accounting.

The runner MUST know the **quantity and unit semantics** of each selected
output before comparing it; merely storing two floating-point numbers does
not establish their dimensional compatibility. Neither an omitted field nor
an unset flag implies a valid reference or a scientifically supported claim.

The **complete** scientific provenance schema (citations, source access,
uncertainty types, correlation, calibration relationships, model context and
applicability, reproducibility metadata) remains normatively required by the
governing VVUQ contract but SHOULD remain in reviewable documentary case
records until the VV0.1-C fixture/schema design is approved. Avoid freezing
C struct layouts or external enumeration values for metadata that the first
increment does not need. Documentary representation is a scope choice, **not**
a waiver of the governing scientific requirements.

## 6. Developer-only adapter boundary

### 6.1 Dependency direction

```text
      independent MATHEMATICAL references / contract expectations
                           |
                           v
                VVUQ runner (developer-only)
                 |                         |
       invokes   |                         | invokes comparison
                 v                         v
          B2 BBTC adapter         B1 comparison core
                 |                         |
                 | links                   | independent of production
                 v                         | BBTC; no production link
             bbtc::bbtc                   |

             bbtc::bbtc ---X---> any VVUQ component
```

**Normative responsibility split:** the adapter obtains actual observations
and raw API statuses through supported public BBTC entry points; it MUST NOT
select comparison policies, independently generate expected values, or decide
scientific evidence claims. The runner owns case selection, reference
admissibility, quantity/unit matching, invocation ordering, B1 comparison
calls, negative-control classification, and required-case accounting. The
B1 core compares two admissible native scalars under an explicit policy, but
does **not** determine scientific provenance, units, or physical validity.

This is a **conceptual** data/dependency diagram, not an approved source-tree
or CMake graph. The B1 core MUST remain linkable and testable without
`bbtc::bbtc`; the adapter MAY link `bbtc::bbtc`, while a higher-level
runner/test target may link **both** the adapter and the B1 comparison library.
Normal consumers must not inherit cmocka, Python, fixture parsers, or
developer validation dependencies.

### 6.2 Adapter duties

A future adapter must:

- Call the **actual supported public** typed BBTC function, not duplicate its
  private implementation or manufacture a simulated observation.
- Preserve exact input identity, selected scalar family, the entire returned
  observable (including all selected derivative fields), and raw operation
  status.
- Preserve returned warnings, applicability flags, termination or field-validity
  data **when the selected API actually supplies them**; absence is not a
  fabricated zero-warning assertion.
- Report which API, model, source commit, and reference-case revision were used.
- Treat non-successful production execution as a separate outcome; never use an
  uninitialized, invalidated, or failure-cleared output as a successful numeric
  observation.
- Keep input borrowed-storage lifetimes and output memory ownership explicit;
  provide no hidden cross-family narrowing.
- Reject unsupported case types and scalar-family mismatches explicitly.

### 6.3 Lifetime, output state, and status invariants

1. The adapter MUST retain ownership outside production BBTC: it may borrow a
   case's immutable coefficient array only for a call during which the array
   and every referenced element remain readable. The adapter MUST NOT free,
   modify, or retain borrowed pointers beyond the documented lifetime.
2. `float`, `double`, and `long double` evaluation MUST use the matching
   public typed entry point, native input objects, and native output objects;
   no intermediate conversion through binary64 is permitted.
3. Where the public API promises that a non-null output is cleared on failure,
   the adapter MUST preserve the observed cleared fields and the returned raw
   status independently. Tests MUST verify the documented fields, **not raw
   structure padding or arbitrary byte representations**.
4. A null required output pointer is tested solely through the public status
   contract; no code may dereference or attempt to inspect missing storage.
5. The adapter MUST expose whether each selected output field is valid for
   numerical comparison; a failure-cleared zero is not a valid observation.
6. Preserve distinct `bbtc_status_e` (production execution),
   `bbtc_vv_operation_status_e` (B1 comparison operation), and
   `bbtc_vv_finding_e` (B1 mathematical result). Mapping them into one success
   flag is forbidden.
7. The runner MUST prevent a comparison when a required reference, expected
   quantity/unit, or actual output is missing, inadmissible, or invalid; it
   MUST identify the reason rather than manufacturing PASS or FAIL.

### 6.4 Deferred implementation decisions

B2a **does not freeze** C function signatures, allocation layout, enum integer
values, file paths for future implementation, a runtime plugin mechanism,
fixture schema, or the full reporting wire format. A subsequent implementation
review must select the minimal set justified by approved cases and demonstrate
that validation of invalid states is testable.

## 7. Evaluation result axes and failure precedence

The future result model must preserve distinct conceptual axes already required
by VV0.1-A:

| Axis | Examples (conceptual, not C enum values) |
| --- | --- |
| Execution state | Not run, completed, execution error, interrupted |
| Reference integrity | Accepted, missing, invalid, unresolved |
| Comparison finding | Pass, fail, indeterminate, not applicable |
| Evidence support | Mathematical support, empirical support, contradiction, insufficient evidence |
| Applicability | In domain, outside domain, undetermined |
| Uncertainty completeness | Sufficient, incomplete, unknown, not applicable |

These axes are not reducible to a single `success` or `valid` Boolean.
In particular, a failed BBTC call is **not** a comparison mismatch, a bad
reference is **not** a validated prediction, and a skipped or unsupported
required case is **not** a passing test. A downstream comparison should run
only after the corresponding reference, unit compatibility, required output,
and production execution have been verified as admissible.

## 8. Native precision and numerical comparison

- Preserve the `float`, `double`, and `long double` families end-to-end.
- Inspect and record host precision information rather than inferring the
  format from `sizeof(long double)` or assuming wider-than-double precision.
- Reuse the accepted developer-only B1 `bbtc_vv_compare_*` interfaces without
  modifying their arithmetic or accepted return semantics.
- Select comparison method **per quantity and case**, not by default global
  percentages or after-the-fact threshold adjustment.
- Keep B1 operation status distinct from B1 numerical finding and from BBTC
  production operation status.
- Treat unsupported binary/rounding/fenv configurations as unsupported rather
  than declaring scientific PASS.
- Include exact/rounded diagnostic state without allowing diagnostics to
  override the exact mathematical finding of an admissible B1 comparison.

## 9. Build, self-test, and acceptance strategy

Existing top-level CMake opts into VVUQ via `BBTC_BUILD_VVUQ`; VVUQ tests and
Python oracles have additional gates. A later B2 implementation SHOULD add a
**small developer-only adapter target** linked to `bbtc::bbtc` but **not** to
the B1 comparator. The runner or test target SHOULD depend on the adapter and
`bbtc_vv_comparison` separately, so the adapter remains responsible solely
for invoking production APIs and preserving observations. The specific target
names and source paths are deferred to implementation review. No reverse
production dependency or change to the default production build is allowed.

The implementation checkpoint must plan for:

- A deterministic matrix across supported native scalar families.
- Independent mathematical expectations (not copied BBTC baselines).
- Deliberately wrong mathematical expectations that MUST yield B1 FAIL,
  plus a runner self-test proving that such an expected failure is **counted
  as a successful negative control**, not a passing scientific reference.
- Separate invalid-input, status-precedence, ownership, and output-clear
  contract checks; runner reference-integrity rejection; actual status
  preservation; no silent unexecuted required case.
- Strict GCC/Clang configurations and applicable sanitizers, with supported
  toolchain/ABI/floating-point environment recorded.
- Clearly distinguished implementation verification from model validation.
- Developer-only cmocka/CTest integration without mandatory production tools.

Passing B1 acceptance evidence does not qualify newly added B2 adapter code.
That adapter must obtain its **own** scoped verification evidence.

## 10. VV0.1-B2a acceptance conditions (documentation only)

The project owner explicitly accepted this **documentation-only** design on
2026-11-05 US Eastern (2026-11-06 UTC), after review of the following
conditions. Acceptance is recorded separately in
`docs/validation/VV0_1_B2A_FINAL_ACCEPTANCE_DECISION.md`. Each condition
concerns the **design** only; implementation qualification is deferred:

1. The exact accepted source baseline and governing contracts are identified.
2. API inventory distinguishes implemented functions from future interfaces.
3. A small, scientifically meaningful first case family is identified and its
   mathematical reference derivation is independent of production evaluation.
4. The quantity, SI units, derivative units, reference state, domain, scalar
   family, and coefficient-array ownership are stated for each candidate.
5. Each of M-01 through M-04 has three analytical outputs, with explicit
   conditions, SI units, and candidate native EXACT policy. The intended
   36-comparison matrix is recorded as **planned**, not claimed as executed.
6. Method/threshold policies are justified **before** qualification; every
   proposed EXACT policy has a documented source-level representability and
   intermediate-arithmetic argument with explicit platform prerequisites.
   Actual compiler/ABI/fenv qualification remains a successor B2 implementation
   gate and is **not** implied by documentation-only B2a acceptance.
7. Contract/status cases are separated from independent mathematical results.
8. The adapter cannot transform BBTC errors or incomplete records into PASS.
9. Reference integrity, comparison, applicability, evidence, execution, and
   uncertainty axes remain distinct.
10. The developer-only dependency direction and ordinary-build isolation are
   explicit; no public production ABI is changed.
11. Test strategy includes independent positive/negative controls, required-case
    completeness, three scalar families, and supported toolchain restrictions.
12. Fixture interchange, full reports, uncertainty algorithms, and detailed C
    signatures are explicitly deferred to their proper checkpoints.
13. A reviewer records revisions, outstanding concerns, an explicit acceptance
    decision, and the exact commit to which that decision applies.
14. **No C implementation, tests, production ABI, accepted B1 numerical code,
    tag, or physics contract is modified by accepting B2a.**

## 11. Resolved design decisions (accepted contract version 0.1.0)

The project owner approved all seven *design directions* on **2026-11-05**
and subsequently gave explicit approval for **scoped B2a design acceptance** on
2026-11-05 US Eastern (2026-11-06 UTC). This acceptance does **not** qualify
future B2 implementation code, new compiler/ABI/fenv profiles, or physical
model predictions.

| Decision | Revision-2 disposition | Governing section |
| --- | --- | --- |
| Mathematical and contract/status tests | Both in first implementation increment; distinct evidence roles | §§2.1, 4.4, 9 |
| Qualified EXACT comparisons | Initial four cases, three outputs per scalar; arithmetic proof required | §§4.3, 8 |
| Minimal runtime metadata | Typed execution essentials now; comprehensive schema deferred | §5.1 |
| Runner/adapter/B1 separation | Runner coordinates; adapter obtains observations; B1 compares | §§6.1–6.4 |
| Documentation and records | Validation docs under `docs/validation/`; named paths below | §11.1 |
| Ownership/status/output semantics | Borrowed lifetime, native type, cleared fields, distinct statuses | §§4.4, 6.3 |
| Off-center case and failing controls | M-04 at 320 K; deliberately incorrect reference MUST fail | §§4.3–4.4, 9 |

### 11.1 Documentation placement and planned records

- **Current B2a design:**
  `docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md`.
- **Approved B2a acceptance decision:**
  `docs/validation/VV0_1_B2A_FINAL_ACCEPTANCE_DECISION.md`.
- **Future case-derivation and provenance index (not yet created):**
  `docs/validation/references/VV0_1_B2_REFERENCE_PROVENANCE.md`.
- **Future captured implementation qualification evidence:**
  versioned files under `docs/validation/evidence/`, named at review time.

The future provenance index must retain stable case identities, derivations,
actual source/revision, quantities, units, uncertainty status, and limits of
independence. These documentary paths do **not** freeze JSON/fixture wire
formats, and the index does not claim experimental validation.

### 11.2 Remaining review and implementation qualifications

The six former *architecture-choice* questions are resolved as design
directions. The following **verification obligations remain open** and are
not resolved by design acceptance:

1. A reviewer must check the arithmetic exactness argument against **each
   actual** native implementation and admitted compiler/fenv profile before
   enforcing an EXACT test gate.
2. Every intended negative-path status and output-clearing claim must be
   checked against the precise public header and implementation revision;
   multi-invalid-input precedence is not universal across APIs.
3. The successor B2 implementation must define and test minimal typed
   structures, error-handling rules, and the adapter/runner target graph,
   without public ABI or B1 core changes.
4. The later VV0.1-C design must settle fixture representation, canonical
   identities, report schema, provenance transport, and CI gate policy.
5. The B2a decision remains tied to the immutable reviewed **draft** commit
   `a9ae0c60a5562c5d7911428004d301ec3d106f39` and its recorded SHA-256.
   Any substantive change to this accepted design requires explicit review;
   acceptance of a changed file must not be implied from this decision.

**Next checkpoint:** plan the separately reviewed B2 implementation, beginning
with compiler/fenv-qualified reference-case checks and a developer-only C23
adapter. No B2 implementation is accepted by this document.
