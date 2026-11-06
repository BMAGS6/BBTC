# VV0.1-B2a — Independent Reference and Adapter Design: Final Acceptance Decision

| Metadata                     | Value |
| ---------------------------- | --- |
| Document class               | Project-owner design acceptance decision and review record |
| Status                       | **ACCEPTED — SCOPED, DOCUMENTATION-ONLY DESIGN** |
| Owner approval               | 2026-11-05, America/New_York (2026-11-06 UTC) |
| Decision authority           | BMAGS project owner |
| Repository                   | `BMAGS6/BBTC` |
| Reviewed branch              | `rewrite/vv0_1b2a_reference_adapter_design_v1` |
| Reviewed design-draft commit | `a9ae0c60a5562c5d7911428004d301ec3d106f39` |
| Reviewed draft path          | `docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md` |
| Reviewed draft SHA-256       | `24368651dc369db15d5dd5078ab0889038f0fa4b80cac2e59e1e99fe5dc574bf` |
| Accepted predecessor commit  | `af2a7591d74bd27b764268dd65f8c1b367bc0164` |
| Accepted predecessor tag     | `vv0.1-b1-accepted-20261030` |
| Governing contracts          | `VVUQ_DESIGN_CONTRACT.md` v0.1.0; `VVUQ_COMPARISON_CORE_CONTRACT.md` v0.1.0; `BBTC_DESIGN_CONTRACT.md` v0.1.24 |

> **Decision boundary:** The project owner explicitly approved **acceptance of
> the VV0.1-B2a design milestone** after a review of its architecture and stated
> limitations. This approval applies to the **content reviewed at the draft
> commit**, promoted to accepted design metadata in the accompanying
> documentation-only change. It does **not** certify or qualify any B2
> implementation, production model, numerical execution on new platforms, or
> physical configuration. The decision is an internal project acceptance, not
> an external scientific audit or third-party certification.

## 1. Decision

**Formal disposition: ACCEPT — DOCUMENTATION-ONLY DESIGN, WITH EXPLICIT
SUCCESSOR IMPLEMENTATION GATES.**

Accept the independent-reference case directions and BBTC adapter architecture
in the reviewed design draft, with the editorial status promotion and decision
record described here. The accepted design authorizes a **separately reviewed
B2 implementation plan**; it is not permission to skip implementation review,
execution testing, compiler qualification, or scientific evidence checks.

The reviewed Git commit identifies the *draft's actual reviewed content*.
The subsequent acceptance-documentation commit, once recorded, will identify
this decision and the contract-status promotion. The acceptance commit is not
an alternative source-code qualification baseline. No source or numerical
implementation was created or accepted during B2a.

## 2. Accepted design scope

1. **Source-identified public APIs:** the existing first-order virial
   temperature-law evaluator is the initial mathematical subject; geometry
   validation and precision metadata are supporting contract/status subjects.
   Reduced-gas thermodynamics remains a separately reviewable later candidate.
2. **Independent analytical expectations:** four deliberately simple,
   mathematically derived Chebyshev reference directions (`M-01` through
   `M-04`), including the off-center quadratic case. Each proposes checks of
   the coefficient and two temperature derivatives across `float`, `double`,
   and `long double` (36 **planned**, not executed, numeric comparisons).
3. **Comparison policy:** initial EXACT comparison is a *candidate* policy for
   cases supported by a source-level dyadic-arithmetic screening argument.
   A successor must independently verify arithmetic and admitted compiler,
   ABI, native format, optimization, and floating-point environment behavior
   before enabling a required EXACT qualification gate.
4. **Distinct contract checks:** status precedence, invalid input, output
   clearing, borrowed-data lifetime, native scalar dispatch, and observation
   integrity are separate from mathematical reference outcomes.
5. **Architectural separation:** the developer-only adapter obtains actual
   observations from public `bbtc::bbtc` APIs; the runner controls execution,
   compares through the accepted B1 comparison API, and preserves separate
   scientific/evidence axes. `bbtc::bbtc` must never depend on VVUQ.
6. **Minimal initial runtime types:** retain only the fields necessary for
   trustworthy typed execution; full fixture interchange, provenance
   transport, strict schemas, and deterministic reporting remain later work.
7. **Validator self-challenge:** deliberate wrong-reference controls must
   demonstrate a mathematical **FAIL**, with the runner correctly accounting
   for an *expected negative-control result* rather than misreporting a
   scientific PASS or a silent skip.

## 3. Evidence and review disposition

| Gate | Disposition at this documentation-only checkpoint |
| --- | --- |
| Accepted predecessors and exact baseline identified | **SATISFIED** — commits, tag, and governing contracts identified |
| Implemented public API inventory grounded in source | **SATISFIED FOR DESIGN** — selected public interfaces and limitations enumerated |
| Independent mathematical case derivation | **SATISFIED FOR DESIGN** — four analytical examples and three outputs each defined |
| Quantity, dimensional, ownership, and scalar semantics | **SATISFIED FOR DESIGN** — explicit semantics; runtime validation deferred |
| EXACT policy justification | **SATISFIED AS PROPOSAL** — source-level screening; compiled qualification deferred |
| Contract/status references separated from analytical evidence | **SATISFIED** — independent result meanings maintained |
| Adapter, runner, and comparison-core boundary | **SATISFIED** — dependency direction and responsibilities separated |
| Failure accounting, result axes, and negative controls | **SATISFIED AS DESIGN** — mandatory successor checks defined |
| Minimal representation, deferred fixture and report formats | **SATISFIED** — no premature wire-format/ABI commitments |
| Developer-only build and production isolation | **SATISFIED AS DESIGN** — no B2 implementation added |
| Project-owner approval | **SATISFIED** — explicit scoped approval, dated above |

The approved draft was committed as a single Markdown file at
`a9ae0c60a5562c5d7911428004d301ec3d106f39`. The owner verified that
its content SHA-256 matched the reviewed document and that the worktree was
clean following the draft commit. Source-level inspections of the selected
public interfaces and governing contracts informed the review. No additional
B2 compilation, sanitizer campaign, fixture execution, experimental-data
analysis, or independent third-party review is claimed.

The documentation-only status promotion and this acceptance decision must be
reviewed and committed together in a subsequent commit. Before publication,
the owner should verify that only these two Markdown paths changed, that Git
reports no whitespace errors, and that the working tree is clean after commit.

## 4. Explicitly unaccepted work and restrictions

- No B2 C23 adapter, runner, or test implementation has been produced or
  qualified by this decision.
- The proposed EXACT comparisons have **not** been executed and qualified
  across the intended compiler/ABI/fenv configurations. B1 comparison-core
  acceptance does not automatically extend to the B2 adapter.
- No JSON fixture schema, parser, canonicalization, reference database,
  evidence register, reporting interchange, or CI acceptance gate is frozen
  or accepted here.
- No experimental physical-model validation, uncertainty quantification,
  predictive-accuracy claim, material safety assessment, firearm safety
  determination, or ammunition suitability determination is established.
- The accepted B1 source, tag, evidence archive, and production public ABI
  remain unchanged by this checkpoint.
- Regression fixtures, synthetic data, or mathematical identities must not be
  presented as independent experimental observations.

## 5. Successor implementation requirements

1. Establish an independently reviewed B2 implementation checkpoint on the
   accepted design baseline; do not amend the reviewed B2a draft commit.
2. Inspect all selected public definitions and all three native evaluator
   source paths at the exact implementation baseline before writing adapter
   interfaces or judging error-status precedence.
3. Qualify the EXACT-case arithmetic against the compiled code and supported
   numerical environment; preserve floating-point family and B1 API semantics.
   Do not relax a failed EXACT policy after observing a result without a
   separate evidence-backed review and contract change.
4. Build a minimal, documented developer-only adapter and separate runner
   orchestration, preserving the existing production ABI and normal build.
5. Provide independently checkable positive and negative controls, including
   explicit required-case accounting and preservation of raw BBTC statuses.
6. Capture reproducible implementation verification evidence and its
   limitations before claiming that B2 functionality is accepted.
7. Defer full fixture serialization, comprehensive evidence reporting, and
   CI gate design to the separately reviewed VV0.1-C milestone.

## 6. Versioning and publication

- **Reviewed design content:** draft commit
  `a9ae0c60a5562c5d7911428004d301ec3d106f39` and SHA-256 above.
- **Accepted design version:** `0.1.0`, recorded by updating the draft's
  status and recording this owner-approved decision.
- **Acceptance documentation commit:** to be obtained from Git after the
  two-file documentation change is reviewed and committed. This document
  deliberately does not invent an as-yet-uncreated commit SHA.
- **Suggested annotated tag:** `vv0.1-b2a-accepted-20261106`, to be created
  **only after** the acceptance commit and its contents are verified. The tag
  must identify the reviewed draft revision separately from the acceptance
  documentation commit and must not imply B2 implementation qualification.
- **Remote publication:** a separate, explicitly verified push of the branch
  and annotated tag; no remote publication is implied by this decision alone.

**Owner acceptance applies strictly to the design scope above.** Future
changes to the accepted contract or its mathematical reference directions
require a reviewed successor revision; this record must not be used as a
blanket approval for future code or scientific claims.
