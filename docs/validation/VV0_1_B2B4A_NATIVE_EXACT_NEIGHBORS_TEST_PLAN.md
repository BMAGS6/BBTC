# VV0.1-B2b.4a Stage 2 — Native EXACT adjacency and signed-zero tests

| Field | Value |
| --- | --- |
| Phase | B2b.4a, stage 2 |
| Status | **REVIEW DRAFT; Q1/Q2 DEFERRED; NO B2b ACCEPTANCE** |
| Required source baseline | `58d4679dc798b6ce6b67baeb66b417b037fdaa04` |
| Governing contract | Accepted B1 EXACT comparator; accepted B2a M-01..M-04 analytical references |
| New test | `tests/validation/test_virial_exact_neighbors.c` |
| CTest entry | `bbtc.vv.virial_exact_neighbors` (developer-only) |
| Production changes | **None** |

## 1. What EXACT means (and what it does not)

The approved B1 `BBTC_VV_COMPARISON_EXACT` policy requires *numerical equality*
of the supplied, finite, admissible native floating-point scalars, with a
numerically zero tolerance. It does not imply bit-for-bit identical object
representations, nor that a supplied reference equals the ideal real-number
result. In particular, both `+0` and `-0` must compare as PASS; NaN or
infinity must never become successful comparison inputs. EXACT comparisons
return a noncomputed diagnostic metric, canonical positive zero, and a
separately meaningful operation status and mathematical finding.

Stage 1's independent Python `fractions.Fraction` oracle checks that the fixed
four mathematical references and the relevant source arithmetic are exactly
representable in the candidate native formats. Stage 2 challenges the B1
comparison engine with *different numerical values* immediately adjacent to
those analytical outputs. Both stages are supporting evidence for Q1; neither
alone qualifies an actual compiler, runtime, native ABI or physical model.

## 2. Fixed test matrix and accounting

The new suite repeats the frozen analytical expected values from M-01 through
M-04 for each of the three native scalar formats. This purposeful duplication
makes the comparator challenge independent of producer outputs, while remaining
traceable to the accepted B2a cases and stage-1 oracle. If the frozen fixtures
are revised, the oracle, matrix and this suite must receive a joint identity
and change-control review; identical old fixtures must not pass as proof of a
changed reference contract.

Within each native scalar family (`float`, `double`, `long double`):

1. For each of four cases and three ordered quantities (B, dB/dT, d²B/dT²),
   compare the native reference to itself: **12 required operation SUCCESS and
   finding PASS results**.
2. Use native `nextafterf`, `nextafter`, or `nextafterl` to obtain the immediately
   lower and upper representable values. Compare both against the same
   reference: **24 required operation SUCCESS and finding FAIL results**.
3. Reverse each `nextafter` step toward the reference and require numerical
   equality. For zero references, additionally require that both neighbors
   classify as `FP_SUBNORMAL` (not flushed away).
4. Compare `+0` against `-0` and vice versa: **two additional operation SUCCESS
   and finding PASS results**. These check numerical, not bitwise, equality.
5. Require every successful EXACT comparison to return
   `BBTC_VV_METRIC_NOT_COMPUTED` and a canonical positive-zero metric.

Across three families, the required counts are **36 self-equality PASS,
72 adjacent-value FAIL, and 6 signed-zero PASS** (114 B1 comparisons). These
are *comparator-only challenges*, not 114 additional production observations
and not 114 separate CTest entries. The new CTest contains three cmocka tests,
one per family. The existing 36-case production reference matrix remains a
separate regression gate with its own provenance and statuses.

## 3. Floating-point environment and fail-closed behavior

Before obtaining subnormal neighbors, each native test requires both of the
existing read-only profiles to return `SCREENED_UNQUALIFIED` with zero issues:
producer/compiler metadata consistency, ISO C nearest rounding, supported
native formats, and x87/MXCSR control restrictions. In particular, the control
screen rejects FTZ and DAZ and unmasked exceptions. No failed screen is
silently skipped, converted into a B1 numerical finding, or described as
formal qualification.

A successful preflight precedes `feholdexcept`, which captures the caller's
fenv, temporarily masks traps and clears sticky flags in this **isolated test
executable**. The test runs all adjacency operations, including zero to
subnormal transitions that may raise underflow/inexact, and calls `fesetenv`
to restore the saved original environment before the final profile screen.
An early assertion failure aborts this developer-only process; it does not
modify production state. Separate B2b.4b work will challenge *actual live
unsupported control states* in isolated subprocesses.

## 4. Build integration and evidence boundary

CMake creates the new target only under the existing VVUQ developer-test gate,
links the accepted B1 comparison library, VVUQ profile/control screens, and
cmocka, and supplies the existing strict floating-point compiler options. No
installed headers, public production ABI, B1 numerical algorithm, or default
production dependencies are changed. Python remains optional and is not needed
to run this CTest.

Required review evidence: exact Git baseline and patch hash; GCC strict,
Clang strict, and Clang ASan+UBSan build/test commands and outcomes; executed
CTest name and cmocka counts; full source diff and whitespace checks; unchanged
production-only configuration and target isolation. A PASS is only a positive
verification result for this narrow comparator challenge on the *screened*
execution profile, not a declaration that M-01..M-04 compiled production
arithmetic is qualified or that B2b is accepted.

## 5. Stage disposition and follow-up

Stage 2 is **proposed until independently built and reviewed**. After passing
on the required configurations and recording evidence, review it as a scoped
B2b.4a checkpoint; keep the separate, uncommitted
`VV0_1_B2B_FINAL_IMPLEMENTATION_AUDIT_DRAFT.md` intact. Next stage B2b.4b
will introduce controlled mutation experiments and isolated live fenv
rejection, followed by B2b.4c compiler/ABI hardening and B2b.4d reevaluation.

**Q1, Q2 and Q4 are not approved by this test. No acceptance tag is authorized.**
