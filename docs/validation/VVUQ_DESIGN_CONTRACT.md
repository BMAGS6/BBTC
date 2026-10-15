# BBTC Verification, Validation, and Uncertainty Quantification Design Contract

| Metadata | Value |
| --- | --- |
| Contract version | **0.1.0** |
| Project phase | **VV0.1-A** |
| Status | **Accepted — implementation pending** |
| Acceptance date | **2026-10-15** |
| Target branch | `rewrite/vv0_1a_vvuq_design_contract_v1` |
| Baseline | `e5c249bfd4f8384c6bbe543eb78d0855f78ece6a` |
| Governing contract | `docs/design/BBTC_DESIGN_CONTRACT.md` v0.1.24 |

## 1. Purpose and authority

This contract establishes a scientific and software-engineering foundation for an independent **Verification, Validation, and Uncertainty Quantification (VVUQ)** system for BMAGS' Ballistic Trajectory Calculator (BBTC).

VVUQ exists to evaluate, preserve, and communicate evidence about software correctness, numerical accuracy, physical-model adequacy, uncertainty, and reproducibility. Its principal question is **what conclusion does the available evidence actually justify, for which quantity and within which domain?**

The words **MUST**, **MUST NOT**, **SHOULD**, **SHOULD NOT**, and **MAY** are normative within this project. Requirements of the accepted BBTC reconstruction contract remain authoritative. This companion contract MUST NOT silently supersede any production-physics, public-API, precision, diagnostic, or safety provision. Any conflict MUST be identified and resolved by explicit contract review.

VV0.1-A is a **documentation-only checkpoint**. Acceptance does not authorize physics changes, new public interfaces, new mandatory dependencies, or scientific performance claims. Changes to this document are subject to review and an explicit acceptance decision.

## 2. Scientific meaning and limits

### 2.1 Evidence classes

VVUQ MUST distinguish at least the following, without using one as a surrogate for another:

1. **Software verification:** evidence that code implements a specified contract, mathematical relationship, and required error semantics.
2. **Numerical verification:** evidence about discretization, rounding, representability, solver convergence, stability, conditioning, and algorithmic approximation error.
3. **Physical-model validation:** evidence about agreement between model predictions and independent, relevant observations of physical reality, under stated experimental conditions and within a defined context of use.
4. **Uncertainty quantification:** defensible characterization of uncertainty sources, assumptions, dependencies, and their effect on quantities of interest.
5. **Reproducibility:** the ability to identify and repeat the inputs, model, methods, software, environment, and analyses underlying an evidence record.
6. **Applicability assessment:** evaluation of whether an evidence-based conclusion is relevant to the requested model, physical quantity, conditions, and context of use.

A successful program execution, passing unit tests, conservation residual, cross-precision agreement, or high-precision reference calculation MUST NOT by itself be characterized as physical validation. A recorded numerical regression value MUST NOT be elevated to independent scientific truth.

### 2.2 Context of use and quantities of interest

Every physical-validation claim MUST identify its **context of use**, including the model or model combination, predicted quantity (measurand or observable), physical regime, supported input/parameter ranges, and experimental conditions. Evidence is not automatically transferable to other quantities, coupled models, physical regimes, or use cases.

The validator MUST distinguish a mathematical limit or idealized synthetic case from an experimentally observed state. A numerical benchmark can be valid evidence of implementation fidelity even if the physical model remains empirically unvalidated.

### 2.3 Non-claims and safety boundary

BBTC remains experimental modeling software. VVUQ MUST NOT label ammunition, loading configurations, firearms, structural assemblies, or other physical systems **safe**, **proof tested**, **certified**, or **approved** by inference from successful simulation or a comparison score. This contract MUST remain consistent with the canonical notice in `DISCLAIMER.md` and governing contract section 18.

A validation-domain statement is not a structural-integrity assessment or a substitute for independent safety engineering and testing. Missing warnings or applicability flags MUST NOT imply physical safety or empirical validation.

## 3. Separation of concerns and dependency boundary

### 3.1 Independent developer-facing infrastructure

VVUQ MUST have a reusable **scientific comparison and evidence core** implemented in ISO C23 and testable without linking `bbtc::bbtc`. The core MUST NOT invoke BBTC production evaluators to generate its own reference expectations and MUST NOT include or depend upon the BBTC production implementation for its fundamental comparison, classification, reference-integrity, or uncertainty-metadata behavior.

A **BBTC-specific adapter** MAY link against `bbtc::bbtc` and MUST record which BBTC API, scalar family, and model revision generated each observed result. The adapter MUST preserve production API statuses, warnings, termination reasons, and applicability information rather than replacing them with a single VVUQ result flag.

The dependency direction is strictly **VVUQ runner/adapter → BBTC library**, never **BBTC library → VVUQ**. Ordinary BBTC consumers MUST NOT inherit VVUQ runtime, parser, testing, or reporting dependencies.

### 3.2 Main components

The conceptual architecture comprises:

- A dependency-free C23 comparison/metadata core (within the developer tooling, not the BBTC public ABI).
- Independent analytical or computational reference providers with documented provenance.
- Strict reference-case ingestion and schema validation.
- A BBTC adapter that obtains predictions from actual public APIs.
- An evaluation runner that accounts for all selected and mandatory cases.
- An evidence register and deterministic, machine-readable reporting.
- A separate self-test and fuzzing infrastructure for the validator itself.

The design SHOULD keep these responsibilities distinct without prematurely creating separate linkable libraries for every conceptual subsystem.

### 3.3 Tools and external dependencies

**cmocka** SHOULD be the preferred deterministic unit/mocking framework for newly written VVUQ self-tests, subject to integration review. Existing BBTC tests need not be converted. Mocks MUST be confined to tests and MUST NOT be counted as independent mathematical or physical evidence.

**DeepState** MAY be used for property-based or generated-input testing. **LLVM libFuzzer** MAY be used for coverage-guided fuzz testing, especially parsers and boundary handling. These are independent optional test modes, not required VVUQ runtime dependencies. Toolchain compatibility, including C23 and supported operating systems, MUST be demonstrated before enabling a backend as a required CI gate.

CTest SHOULD orchestrate deterministic test execution. Strict GCC/Clang builds and available sanitizers SHOULD exercise the VVUQ implementation. Additional libraries such as MPFR MAY support *optional* independent high-precision reference programs where justified, but MUST NOT become mandatory production BBTC dependencies or be mistaken for empirical truth.

### 3.4 Build isolation

A default ordinary BBTC library build MUST remain possible without configuring or discovering VVUQ-only third-party dependencies. VVUQ-enabled CMake targets SHOULD be opt-in, with independent options for its runtime, deterministic tests, property tests, and fuzz targets. Candidate names include `BBTC_BUILD_VVUQ`, `BBTC_VVUQ_BUILD_TESTS`, `BBTC_VVUQ_ENABLE_DEEPSTATE`, and `BBTC_VVUQ_ENABLE_FUZZING`; their names and precise build graph are **not frozen** by VV0.1-A.

The validation runner MAY allocate memory, perform file I/O, and generate reports. The production BBTC library's ownership, allocation, reentrancy, and I/O guarantees MUST remain unchanged.

## 4. Reference authority and independence

### 4.1 Reference categories

Every reference MUST declare a category, at least:

- `REGRESSION`: previously recorded output from BBTC or another implementation; supports change detection only.
- `MATHEMATICAL`: analytically derived expectation or independently established mathematical identity; supports defined mathematical verification.
- `COMPUTATIONAL`: result from a separately implemented numerical method or independent reference program; supports numerical cross-checking subject to shared assumptions and reference error.
- `EXPERIMENTAL`: observed physical data suitable for model validation, subject to measurement quality and applicability.
- `SYNTHETIC`: deliberately generated artificial data for infrastructure testing; never, by its mere existence, evidence of real-world predictive accuracy.

A case MAY have multiple references of different categories. The result MUST preserve their independent meanings instead of choosing whichever produces a favorable comparison.

### 4.2 Independence is documented, not assumed

Each reference SHOULD document its derivation, producer, implementation lineage, underlying physical assumptions, and relationship to BBTC or other references. The case MUST identify any known shared equations, data, algorithms, calibration sets, or code. It MUST NOT use an unsubstantiated label such as “independent” solely because another program or compiler produced the value.

A higher-precision implementation is not necessarily independent; identical defective equations may agree at arbitrary precision. An internally maintained golden file is legitimate regression evidence but does not establish empirical model fidelity.

### 4.3 Experimental provenance and calibration independence

An experimental reference MUST identify the physical quantity measured, measurement method, units, instrumentation when known, raw-data lineage or a verifiable citation, published conditions, calibration status, and uncertainty information or an explicit lack of it.

Data used to estimate model parameters MUST be distinguishable from **held-out validation data**. A model fitting its calibration dataset cannot be described as having independent out-of-sample predictive validation. Repeated measurements from common lots, apparatus, operators, or conditions MAY be correlated and MUST NOT be presented as independent solely because they are separate rows.

Source access, licensing, permissions, and redistribution restrictions MUST be recorded. Restricted data MAY be referenced by stable identifiers or locally supplied private fixtures without adding them to the public repository. Cryptographic hashes establish content identity or integrity, **not scientific authenticity**.

## 5. Reference-case identity, schema, and units

### 5.1 Stable identity and revision

Every case MUST include a stable case identifier and schema version. Material scientific changes to reference data, semantics, or acceptance criteria MUST generate a traceable revision; they MUST NOT silently overwrite an accepted interpretation. Duplicate identifiers or ambiguous revision selection MUST be rejected.

Case manifests MUST separate **identity of the case** from **identity of the underlying data bytes**, from **implementation revision**, and from **report/run identity**.

### 5.2 Required conceptual fields

A reference-case representation MUST be able to preserve:

| Field group | Required meaning |
| --- | --- |
| Identity | Case ID, case revision, schema version, evidence category |
| Subject | Model/phenomenon, quantity definition, context of use |
| Conditions | Exact input definitions, canonical units, source units if distinct, reference states and sign conventions |
| Expected result | Reference value or accepted mathematical property, representation and scalar/precision semantics |
| Provenance | Derivation or citation, authors/producer when known, document/data revision, calibration relationship |
| Uncertainty | Components, interpretation, units, coverage or distribution when applicable, known correlations or explicit unknowns |
| Applicability | Supported domain, relevant assumptions, known limitations and exclusions |
| Comparison | Named policy, predeclared thresholds or rationale, expected behavior at boundaries |
| Integrity | Raw fixture/data content identity and reproducible normalization procedure |

Not every category requires an experimental uncertainty estimate, but absence MUST be represented as **missing or unknown**, never silently as exact zero.

### 5.3 Quantity semantics and dimensional compatibility

VVUQ MUST define quantities by physical dimension and semantics, not merely by a textual unit label. Canonical comparison values SHOULD use SI units and MUST preserve whether a value is absolute versus relative, gauge versus absolute, intensive versus extensive, cumulative versus instantaneous, and reference-state-dependent versus reference-state-independent.

Pressure, temperature, energy, and time quantities with different definitions MUST NOT be compared as interchangeable. Units, coordinate frames, sign conventions, sampling or aggregation method, and measurement locations MUST be compatible before numerical comparison. Invalid or ambiguous conversions MUST produce an explicit reference/fixture error.

### 5.4 Numeric representation and interchange

The format MUST preserve the precision and origin of the reference. Parsing every numeric value via binary64 `double` is forbidden where it would silently degrade a native `long double` or externally computed higher-precision reference.

A versioned, strict JSON-based fixture format is the **preferred design direction**, with numeric strings (decimal or hexadecimal where justified) and explicitly defined precision/rounding semantics. **The exact JSON wire schema, parser choice, binary serialization, and canonicalization algorithm remain deferred** to VV0.1-B/C review. Conceptual metadata and scientific meaning are frozen here; no unreviewed implementation format is implied.

NaNs, infinities, signed zeros, subnormals, overflow, underflow, and representational limits MUST have explicit policies for each field. A fixture claiming an ordinary finite physical measurement MUST NOT smuggle a nonfinite value through permissive parser behavior.

## 6. Comparison policies and numerical verification

### 6.1 Named policies, not universal tolerances

Each comparison MUST select an explicit, documented method appropriate to its quantity and evidence class. No global percentage rule or single all-purpose `abs + rel` formula may be used as the sole criterion across all scientific problems.

Supported methodological families SHOULD include, when justified:

- Exact discrete classification and contract invariants.
- Absolute error, relative error with a defined nonzero reference condition, and scale-aware error.
- Representable-spacing/ULP analysis where the floating-point representation is relevant.
- Condition-aware forward/backward-error bounds and interval-style enclosures where justified.
- Independent numerical references with their own demonstrated error or convergence characterization.
- Convergence studies and refinement-order analysis where a sequence and asymptotic regime are meaningful.
- Measurement/prediction discrepancy analysis with properly characterized uncertainty and covariance.

Thresholds MUST be declared **before** judging a case, with units, methodology, rationale, and applicability. They MUST NOT be widened ad hoc to force a desired outcome. A zero or unknown reference uncertainty MUST NOT be silently substituted for an engineering tolerance.

### 6.2 Stable computation

Implementations MUST avoid arithmetic overflow/underflow introduced solely by a naïve error formula where a mathematically equivalent stable method is available. The comparison itself requires self-testing at extreme magnitudes, near-zero differences, signed zero, adjacent representable values, subnormals, and nonfinite inputs.

An absolute comparison near zero SHOULD NOT be transformed into an undefined or misleading relative comparison. ULP distance MUST be defined for the actual representation and special cases; storage size alone does not determine `long double` format. Numerical comparisons SHOULD distinguish the *mathematical error*, the *reported rounded value*, and the *reference computation's uncertainty*.

### 6.3 Three native scalar families

`float`, `double`, and `long double` remain independently evaluated native families. VVUQ MUST NOT silently implement either `float` or `long double` comparison by narrowing through `double`. Cross-family agreement MAY be reported as diagnostic evidence but MUST NOT imply `long double` is exact mathematical or experimental truth.

Relevant metadata SHOULD include floating-point radix, precision, exponent range, actual format when established, selected rounding mode, compiler flags, and subnormal/excess-precision behavior where observable. Fast-math modes that invalidate required nonfinite/IEEE semantics MUST NOT be used in reference builds.

### 6.4 Numerical convergence is not physical validation

When full solvers exist, solver tolerances, step-size refinement, event-location convergence, conservation balances, and nondimensionalization effects SHOULD be tested within declared numeric domains. A nearly zero residual does not independently authenticate the underlying physical model or the correctness of every upstream source term.

A skipped case, failed reference calculation, or unconverged numerical oracle MUST remain accounted for, and MUST NOT be reported as a successful comparison.

## 7. Uncertainty representation and interpretation

### 7.1 Separate sources

VVUQ MUST distinguish measurement, input-parameter, calibration, numerical, and model-form uncertainty, and SHOULD distinguish experimental variability and repeatability from systematic effects and between-group differences when the data permit. Correlation is part of an uncertainty model, not an optional cosmetic annotation.

A field MUST distinguish **known zero uncertainty** (only where defensible), **quantified nonzero uncertainty**, **unknown**, **not supplied**, and **not applicable**. No generic “unset means zero” convention is permitted.

### 7.2 Meaning of an uncertainty estimate

A numeric uncertainty record MUST state its interpretation (for example: standard uncertainty, expanded uncertainty, interval bound, standard deviation, confidence/coverage interval, or distributional parameter), units, evidence, assumptions, and applicable confidence or coverage conventions. A numerical bound and a probabilistic confidence interval MUST NOT be treated as interchangeable.

Covariance, dependence, and shared systematic contributions MUST be incorporated when supported and relevant. Independence MUST NOT be assumed merely because covariance data were omitted. Unresolved dependence SHOULD be reported as an evidence limitation or used in clearly labeled sensitivity scenarios.

### 7.3 Propagation and model discrepancy

Methods such as linear sensitivity propagation, Monte Carlo propagation, and uncertainty-aware model comparison MAY be developed in later phases. Each MUST specify its applicability, sampling or approximation assumptions, reproducible seeds if applicable, convergence diagnostics, and limits.

Unknown model-form discrepancy MUST NOT be concealed by inflating experimental uncertainty until a prediction passes. Uncertainty components MUST NOT be double-counted. Apparent precision of a model prediction MUST NOT be promoted to certainty about physical reality.

### 7.4 Empirical comparison decisions

An uncertainty-normalized residual or statistical test MAY contribute to a validation assessment but MUST NOT, on its own, establish general physical-model validity. An empirical assessment SHOULD examine systematic bias, residual structure, measurement uncertainty, correlation, sample size, repeated observations, calibration leakage, possible multiple testing, and the stated validation domain.

An assessment MUST identify the claim being tested and the predeclared decision rule. If uncertainty information is insufficient to support that rule, the appropriate result is **insufficient evidence** or **indeterminate**, not an invented accuracy guarantee.

## 8. Results, failure semantics, and scientific claims

### 8.1 Orthogonal result axes

A VVUQ record MUST preserve separate axes for:

1. **Execution state** — not run, completed, execution error, or interrupted/incomplete.
2. **Reference integrity** — accepted, invalid, missing, or unresolved.
3. **Comparison finding** — pass, fail, indeterminate, or not applicable.
4. **Evidence class and support** — mathematical/numerical support, empirical support, contradiction, or insufficient evidence, as applicable.
5. **Applicability** — in evaluated domain, outside domain, or undetermined.
6. **Uncertainty completeness** — sufficiently characterized for stated inference, incomplete, unknown, or not applicable.

These are **conceptual axes**, not frozen C enum constants. Formal names, storage widths, and serialization values will be specified in an implementation checkpoint. A single `valid` Boolean MUST NOT be the authoritative scientific outcome.

### 8.2 Mandatory distinctions

The reporting layer MUST distinguish at least:

- A failed BBTC API call from a successfully evaluated but discrepant physical prediction.
- A corrupted or missing reference from an actual comparison failure.
- A numerical nonconvergence from an empirically unsupported model.
- A comparison outside the stated validation domain from an in-domain mismatch.
- A known, deliberate skip from an execution that never ran unexpectedly.
- A synthetic test from an actual physical experiment.

Undefined, impossible, or contradictory state combinations MUST be rejected or identified as internal validator defects.

### 8.3 Claim governance

Statements such as “verified,” “validated,” “accurate,” or “supported” MUST specify what was checked, against what, by which method, within what domain, and with what limitations. Claims MUST be narrower than or equal to the available evidence.

A change in model version, calibration parameters, source data, or numerical method MUST trigger review of affected evidence and validation claims. Evidence for an isolated thermodynamic primitive MUST NOT automatically validate a coupled chamber model or full trajectory.

## 9. Reproducibility, reporting, and provenance

### 9.1 Run identity

A machine-readable evaluation report MUST be capable of identifying:

- BBTC source commit/version, VVUQ version, model revision, and reference schema versions.
- Raw and normalized input identity, fixture IDs/revisions/hashes, and source-data revisions.
- Compiler/toolchain identity, optimization and relevant numerical flags, target operating system and architecture.
- Scalar family and relevant floating-point capabilities/environment.
- Solver options, comparison policies, thresholds, applicability checks, and evaluation configuration.
- Any sampling/random seeds, algorithms, and sample counts.
- Which cases were requested, selected, run, skipped, interrupted, invalid, or omitted, with reasons.
- Individual residuals, decisions, numerical diagnostics, uncertainty limitations, and aggregate results.

### 9.2 Determinism and provenance integrity

A stable record structure, case identity, output ordering, and versioned serialization SHOULD permit meaningful reproducible diffs. Byte-for-byte identical floating-point outputs on heterogeneous platforms MUST NOT be promised without an independently specified deterministic execution profile.

A raw-content digest (preferably SHA-256) SHOULD identify exact fixture/data bytes; the digest scheme and canonicalization rules MUST be explicit before relying on normalized content hashes. Run IDs MUST distinguish environmental metadata from deterministic scientific input identities.

The runner MUST be able to produce meaningful reports without network access. Failed report generation or serialization MUST NOT silently produce an apparent scientific pass. Machine-readable output MUST have a declared schema version and MUST NOT contain ANSI escape sequences.

### 9.3 Evidence register

A reviewable evidence register SHOULD link accepted claims and limitations to stable reference cases, runner versions, recorded reports, and the associated model/implementation revisions. Prior experimental evidence MAY be registered as historical evidence with its exact limitations; it MUST NOT be reclassified as physical validation without suitable empirical references.

Reference additions, removals, methodological changes, and rebaselining of expected results MUST be accompanied by an explicit justification and review. Reports SHOULD distinguish regression status changes from genuine changes in scientific evidence.

## 10. Completeness, CI, and acceptance gates

### 10.1 No silent green results

A run MUST declare its intended case selection and identify **every** required case. Missing, skipped, reference-invalid, interrupted, unsupported, or unexecuted required cases MUST be counted explicitly and MUST NOT be rolled into successful results.

The CI gate MUST distinguish between **required computational tests**, whose unexpected absence or failure blocks acceptance, and **physical-validation evidence gaps**, which may be correctly classified as unresolved without making normal software development impossible. An evidence gap MUST NEVER be displayed as a physical-validation pass.

### 10.2 Checkpointed coverage

A compact deterministic self-test and comparison suite SHOULD run on supported routine CI builds. Costly fuzzing, extensive numerical cross-checks, and long-running uncertainty studies MAY run in separate opt-in, nightly, or manually triggered jobs. Optional testing cannot replace the required deterministic suite.

A failed required comparison, validator invariant, schema integrity check, reference authenticity check where applicable, or report-completeness invariant MUST cause the applicable gate to fail. The gate policy MUST be versioned, reviewed, and recorded alongside the report.

### 10.3 Recorded skips and waivers

Any deliberate waiver of a required test MUST identify the case, technical reason, approving revision, duration or review condition, and consequence for claims. A waived case MUST NOT count as passed and MUST NOT disappear from the report. “Expected upstream failure” is a separate classification, not a successfully verified invariant.

## 11. Security, robustness, and validator self-verification

### 11.1 Untrusted input

Reference files and metadata MUST be treated as untrusted. Parsers and loaders MUST enforce bounded lengths, counts, nesting where relevant, allocation limits, integer-overflow checks, duplicate-ID detection, strict numeric grammar, unit/schema validation, and explicit NaN/infinity policies. File/path ingestion MUST resist traversal and unexpected external file access where applicable. Fixture content MUST NOT execute code or shell commands.

Malformed input MUST produce a reproducible classified failure rather than a crash, memory corruption, or partially successful validation report. Input handling SHOULD be tested under sanitizers and fuzzing.

### 11.2 The validator must itself be challenged

The validator MUST have independently checkable positive and negative cases before its outputs are relied on. Mandatory self-test families SHOULD include:

- Correct comparisons with independently calculated expectations.
- Deliberately wrong answers and references, including opposing errors that cancel in a mean.
- Incorrect physical dimensions, compatible-but-different unit scales, reference-state mismatches, and invalid conventions.
- Exact, adjacent-ULP, signed-zero, subnormal, extreme-magnitude, cancellation, overflow, underflow, NaN, and infinity cases.
- Unknown uncertainty, misleading precision, omitted covariance, and incompatible uncertainty interpretations.
- Invalid/duplicate IDs, conflicting revisions, truncated input, corrupted hashes, and malformed numerical tokens.
- Missing/skipped mandatory cases, unexpected execution failures, inconsistent result-axis combinations, and incomplete reports.
- Deterministic failure and output-clearing semantics where the API specifies them.
- Golden report-schema regression checks, property tests, fuzz regression corpora, and targeted mutation tests when the tooling exists.

Test oracles MUST be independent enough that a defect in a VVUQ comparison routine is not systematically reproduced by the corresponding expected-result computation. Mocked sources MAY test control flow but cannot substitute for evidence-bearing references.

### 11.3 Tool-specific use

cmocka SHOULD test deterministic functions, deliberately injected failure paths, and error propagation. DeepState SHOULD be considered for generated inputs, metamorphic relations, and replayable cases where it adds coverage. libFuzzer SHOULD be considered for byte-oriented parsers and fast, bounded numerical input classification. Sanitizer and compiler coverage SHOULD be documented per test class and platform.

No tool's test count, code coverage, or absence of crashes by itself establishes scientific accuracy. An adversarially corrupted reference that passes unnoticed is a validator defect requiring investigation.

## 12. Initial reference scope and staged implementation

### 12.1 Initial supported domains

VV0.1-B SHOULD begin with **self-contained mathematical and numerical primitives** already present in the rewrite, where independent expectations are tractable. Candidates include thermodynamic identities/limits, dimensional conversions, geometry invariants, specified endpoint semantics, and pure comparison-kernel behavior. Specific cases MUST be selected against the actual current public API and accepted physics contract at implementation time.

The initial reference suite MAY include synthetic and analytical data, clearly labeled by category. BBTC-to-BBTC regression comparisons MAY be retained as regression evidence without being represented as independent scientific validation.

### 12.2 Defer unsupported claims

Physical validation of a complete coupled internal-ballistics or trajectory solver MUST await an actual implemented model and trustworthy independent experimental data matched to its intended quantities and domain. No fictional measurements, fabricated calibration conditions, or arbitrary physical accuracy percentages may be introduced to claim validation.

Sensitivity/uncertainty tools SHOULD expand after their numerical methods, parameter provenance, and target quantities have explicit contracts. A future mathematical or experimental result MUST NOT be back-propagated to earlier evidence records without a reviewed re-evaluation.

### 12.3 Milestone sequence

- **VV0.1-A:** accept a documentation-only VVUQ scientific/architectural contract.
- **VV0.1-B1:** implement standalone comparison/evidence primitives and their deterministic self-tests, initially with compiled-in references.
- **VV0.1-B2:** add independently derived analytical/numerical reference cases and a BBTC adapter for supported primitives.
- **VV0.1-C:** introduce strict fixture interchange, evidence registration, deterministic reporting, and CI gates.
- **Later checkpoints:** advanced uncertainty propagation, covariance-aware statistics, convergence studies, expanded fuzzing/mutation campaigns, and empirical validation where evidence permits.

These are proposed checkpoint labels; exact source layout and public CLI details remain reviewable implementation decisions.

## 13. Proposed source layout (not yet binding)

```text
docs/
    design/
        BBTC_DESIGN_CONTRACT.md
    validation/
        VVUQ_DESIGN_CONTRACT.md
        EVIDENCE_REGISTER.md
        README.md
validation/
    schema/
    reference_cases/
tools/
    bbtc_vv/
        CMakeLists.txt
        main.c
        comparison.c
        reference_case.c
        report.c
tests/
    validation/
```

Generated reports and fuzzer artifacts SHOULD live in ignored build/output directories unless explicitly reviewed as versioned evidence. The directory plan, option names, and file decomposition MAY change during later design review without changing the scientific contract.

## 14. VV0.1-A acceptance criteria

VV0.1-A MAY be accepted only after explicit review confirms that:

1. Scientific evidence classes and contexts of use are distinct and cannot be conflated by a successful test count or single outcome Boolean.
2. The C23 VVUQ core is independently testable without `bbtc::bbtc`; only explicit adapters may depend on it, and the reverse dependency is prohibited.
3. cmocka is the preferred deterministic VVUQ self-test tool, with DeepState/libFuzzer optional and isolated from ordinary BBTC consumers.
4. Reference identities, categories, independence, provenance, calibration relationships, quantity/units semantics, and integrity rules are specified.
5. Comparison policies are explicit, precision-aware, justified by the quantity/evidence class, and not governed by a single universal percentage tolerance.
6. Native `float`, `double`, and `long double` handling is preserved without narrowing and without treating higher precision as physical ground truth.
7. Unknown/incomplete uncertainty, covariance, calibration leakage, and model discrepancy have honest representation and inference limits.
8. Execution, reference integrity, comparison, evidence, applicability, and uncertainty completeness have independent outcome semantics.
9. Required-case completeness, skip accounting, evidence limitations, report metadata, and reproducibility obligations prevent silent false success.
10. Untrusted fixture security requirements and adversarial self-verification obligations are defined.
11. Safety and scientific-claims restrictions remain consistent with the accepted BBTC governing contract.
12. Implementation sequencing remains narrow and dependency disciplined, while format, specific enum/API signatures, numeric thresholds, and optional libraries not yet justified remain open for later review.
13. **No production source, public API, CMake configuration, test implementation, empirical data, or active scientific claim is changed by accepting this documentation-only checkpoint.**
14. This contract is explicitly approved, marked **Accepted** with an acceptance date and a reviewed version, and committed separately before implementation begins.

## 15. References and methodological context

The following references guide terminology and methods; their inclusion **does not claim BBTC certification, endorsement, or formal compliance**. Application of particular procedures requires separate documented technical justification.

1. **ASME VVUQ 1-2022.** *Verification, Validation, and Uncertainty Quantification Terminology in Computational Modeling and Simulation.* https://www.asme.org/codes-standards/find-codes-standards/verification-validation-and-uncertainty-quantification-terminology-in-computational-modeling-and-simulation
2. **ASME V&V 20-2009 (R2021).** *Standard for Verification and Validation in Computational Fluid Dynamics and Heat Transfer.* Specific to validation variables/points and associated solution/data uncertainty; not a blanket validation standard for BBTC. https://www.asme.org/codes-standards/find-codes-standards/standard-for-verification-and-validation-in-computational-fluid-dynamics-and-heat-transfer
3. **JCGM 100:2008.** *Evaluation of measurement data — Guide to the expression of uncertainty in measurement* (GUM), with the published 2026 amendment on nonlinearity in measurement models where relevant. https://www.bipm.org/en/committees/jc/jcgm/publications
4. **JCGM 101:2008.** *Propagation of distributions using a Monte Carlo method*, supplement to the GUM. https://www.bipm.org/en/committees/jc/jcgm/publications
5. **NIST Technical Note 1297 (1994 ed.).** Taylor and Kuyatt, *Guidelines for Evaluating and Expressing the Uncertainty of NIST Measurement Results*. https://www.nist.gov/pml/nist-technical-note-1297
6. **LLVM libFuzzer documentation.** https://llvm.org/docs/LibFuzzer.html
7. **DeepState project documentation.** https://github.com/trailofbits/deepstate
8. **cmocka project.** https://cmocka.org/
9. **BBTC governing reconstruction contract**, version 0.1.24, sections 2, 7, 9, 15–16, 18–20 and relevant subsequent accepted phase contracts, at baseline commit `e5c249bfd4f8384c6bbe543eb78d0855f78ece6a`.

---

**Accepted scope note:** VV0.1-A freezes conceptual scientific obligations and dependency boundaries. It intentionally does **not** freeze the exact C ABI, storage types, wire schema, default numerical tolerances, statistical thresholds, external toolchain, or CI implementation before the appropriate narrower checkpoint.
