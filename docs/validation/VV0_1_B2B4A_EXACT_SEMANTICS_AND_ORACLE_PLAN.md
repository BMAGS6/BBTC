# VV0.1-B2b.4a — EXACT semantics and independent rational-trace oracle (initial draft)

| Field | Value |
| --- | --- |
| Phase | `VV0.1-B2b.4a`, stage 1 |
| Status | **REVIEW DRAFT — Q1 AND Q2 DEFERRED; NO B2b ACCEPTANCE** |
| Owner Q1 disposition | Deferred: stronger evidence required for scoped EXACT admissibility |
| Owner Q2 disposition | Additional numerical-assurance testing required before accepting residual limitations |
| Functional scope Q3 | Existing B2a/B2b scope retained as a working proposal, not a formal owner approval |
| B2b acceptance Q4 | Not authorized while mandatory Q1/Q2 evidence work remains open |
| Implementation baseline | `ee39cf2040574bb004e4465ae097340e431558a4` |
| Committed evidence review baseline | `25e7cdb989eaa2a8c3a5d5074b65aa3f6933c012` |
| Governing contracts | Accepted B1 numerical comparison; accepted B2a design v0.1.0 |
| New developer-only helper | `tools/bbtc_vv/virial_exact_rational_oracle.py` |

## 1. EXACT terminology — what is and is not established

1. **B1 `BBTC_VV_COMPARISON_EXACT`:** admissible, finite native operands
   compare numerically equal with numerically zero tolerance. This is neither
   a bitwise comparison nor an automatic proof that either operand is the
   mathematically correct value. In the accepted comparator, `+0` and `-0`
   compare equal; NaN and infinities do not become successful comparisons.
2. **Exact analytical reference:** the intended real mathematical result is
   independently derived and exactly representable in the claimed native format.
3. **Exact evaluated result:** every relevant source-level arithmetic operation
   and its compiled execution preserves the mathematically exact trace for the
   specified fixture and execution profile.
4. **Reproducibility:** passing one native comparison does not imply identical
   object bytes, universal bitwise behavior, or unchanged floating-point state
   across all compiler optimizations and machines.

No existing EXACT tolerance, status enumeration, public ABI, accepted B1 code,
production evaluator, or B2a acceptance contract is modified by stage 1.

## 2. Stage 1: independent rational reference and trace

The Python helper uses `fractions.Fraction` only; it does **not** import a BBTC
shared library, read production outputs, or use host floating-point arithmetic.
It performs two separately coded calculations for each of the accepted four
fixed cases:

- **Independent mathematical reference:** elementary Chebyshev `T0`, `T1`,
  `T2` polynomial identity and its analytical first and second derivatives.
  The expected values are immutable, independently declared rational literals.
- **Source-operation mathematical trace:** temperature normalization, the
  degree-at-most-two backward Clenshaw recurrence, intermediate reconstructed
  quantities, and conditional temperature derivative scaling. It records
  individual relevant arithmetic results, including intermediate sums and
  products, and verifies that they agree with the independent polynomial
  results.

Every recorded nonzero value must be an exactly representable **normal** binary
value under each of the three *captured candidate* native formats:
24-bit `float`, 53-bit `double`, 64-bit x87-extended `long double` significands.
Zero is admitted without claiming bitwise zero-sign equivalence.

The helper also deliberately perturbs each case's independently declared
expected `B` literal and asserts that the altered expectation is rejected.
This is **oracle self-challenge**, not yet a mutation of the C producer and
not a native one-ULP comparison test.

Execute from the repository root:

```sh
python3 tools/bbtc_vv/virial_exact_rational_oracle.py
python3 tools/bbtc_vv/virial_exact_rational_oracle.py --verbose
```

The result is pure mathematical/source-trace evidence; its PASS output **cannot
qualify compiled GCC, Clang, x87, MXCSR, the actual C call graph, or a host ABI**.
It also does not establish correctness for arbitrary temperatures, coefficients,
polynomial order, equation-of-state parameters, or physical measurements.

## 3. Remaining gates before reopening the deferred Q1 and Q2 decisions

| Gate | Required evidence | Status after stage 1 |
| --- | --- | --- |
| G1: EXACT meaning | B1 comparator review, signed zero, nonfinite rejection, strict zero tolerance | Semantic definition documented; targeted native C probes still needed |
| G2: Independent rational oracle | Fixed analytical literals, source trace, normal exact-representability check, self-challenge | Implemented by stage 1 helper; independent code review pending |
| G3: Native adjacent values | `nextafterf`, `nextafter`, `nextafterl` both directions for applicable outputs; EXACT FAIL for every distinct neighbor | Open — stage 2 |
| G4: Mutation detection | Targeted isolated production/runner mutations with predefined detection expectations | Open — stage 2 or later |
| G5: Live fail-closed controls | Isolated child processes with actually changed supported fenv/x87/MXCSR controls; rejection before result admission | Open — stage 2 or later |
| G6: Compiled call/constant bridge | Review actual original object/ELF bytes, constant encodings, relevant calls, source-to-instruction relationship | Open — separate compiled-code audit |
| G7: Optimizer diversity | Additional strict-FP producer profiles and comparison against fixed cases; diagnostic only until approved | Open — separate profile campaign |
| G8: Evidence and decision | Reproducible logs, archive identities, reviewer dispositions, and explicit owner decision | Open |

The stage 1 script should be reviewed, tested, and committed **separately from
formal qualification or B2b acceptance**. Add it to optional CMake/CTest in a
later reviewed increment if needed; no mandatory Python dependency is allowed
in ordinary production builds.

## 4. Safe staging and change control

- Preserve the earlier source/evidence identities; the reviewed evidence
  remains favorable but has not granted qualification.
- The prior uncommitted `VV0_1_B2B_FINAL_IMPLEMENTATION_AUDIT_DRAFT.md`
  must be accounted for independently. Do not silently overwrite, drop, or
  promote it to an acceptance decision.
- No new acceptance tag is authorized. The project owner may review and
  approve each specific qualification claim **after** the required evidence.
- Failures in the new helper, invalid expectation mutations, or disagreement
  between the rational identity and operation trace are grounds to investigate
  the test/proof or source, not to relax a numerical threshold.
