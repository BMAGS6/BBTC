# VV0.1-B2b.3c.3b — Consolidated floating-point qualification review

| Field | Value |
| --- | --- |
| Phase | `VV0.1-B2b.3c.3b` |
| Record class | **Technical evidence review draft; project-owner decision pending** |
| Status | **FAVORABLE SCOPED VERIFICATION EVIDENCE — FORMAL QUALIFICATION NOT GRANTED** |
| Reviewed implementation commit | `ee39cf2040574bb004e4465ae097340e431558a4` |
| Branch | `rewrite/vv0_1b2b_reference_adapter_implementation_v1` |
| Evidence manifest | `docs/validation/evidence/VV0_1_B2B3C3B_EVIDENCE_MANIFEST.md` |
| External immutable archive identity | SHA-256 `f5fcec44f6b75229a04ef27a4a1d543e42a3f46db7e318d07f1de7dd666548f9` |
| Review scope | M-01–M-04; native `float`, `double`, `long double`; three returned terms; three named x86-64 Linux compiler profiles |
| Decision authority | BMAGS project owner, after independent scoped review; not this draft |

## 1. Question and conclusion

**Question:** Does the compiled execution evidence support the source-level
EXACT-arithmetic argument for the fixed M-01 through M-04 mathematical
reference cases on the captured development platform?

**Technical disposition: FAVORABLE, WITH EXPLICIT LIMITATIONS.** All three
fresh builds were successful, all 31 CTest entries passed in each profile,
and 25 focused cmocka tests per profile passed. All six captured translation
units per profile passed the strict-FP *argument* screen. The observed ISO C
metadata and x87/MXCSR control-state screens passed. Textual instruction review
finds no obvious floating-point operation conflicting with the source proof in
the three extracted production evaluator functions.

This is **not** an assertion of mathematical correctness for arbitrary inputs,
a formally qualified compiler/runtime, a general equation-of-state validation,
a complete ballistics model, or physical/firearm safety. The executable screens
explicitly say `SCREENED_UNQUALIFIED` and `FLAGS_SCREENED_UNQUALIFIED`; this
record must not silently upgrade those enumerators or call their results
certification. A scoped acceptance/qualification judgment requires a separate
explicit project-owner decision and the remaining checks in §7.

## 2. Governing source and mathematical scope

The accepted B2a design contract (version 0.1.0) governs separation of
mathematical references, public observations, B1 numerical findings, and
scientific inference. The exact-arithmetic argument is in
`VV0_1_B2B3C1_VIRIAL_EXACT_ARITHMETIC_PROOF.md` and case provenance is in
`references/VV0_1_B2_REFERENCE_PROVENANCE.md`.

Only the fixed temperature interval `[256, 512] K`, the four prescribed
coefficient lists, the prescribed temperatures (`384 K` or `320 K`), and the
three outputs `B`, `dB/dT`, `d²B/dT²` are in scope. For each of four cases,
three native families and three output fields yield 36 required B1 EXACT
comparison decisions per successful reference-matrix execution. Negative
controls and API contract/status tests are separate evidence, not positive
mathematical or physical findings.

The production evaluator source records an affine temperature normalization,
a degree-at-most-two Clenshaw recurrence for these fixtures, and sequential
derivative scaling. The B2b.3c.1 argument derives exact normal dyadic
intermediates under specified binary/rounding/compiler semantics; it does not
generalize to higher-degree coefficients, arbitrary cancellation, subnormals,
or unreviewed target implementations.

## 3. Captured profiles and numerical runtime state

| Profile | Compiler | Effective C mode | Regression | Focused cmocka | Compile screening |
| --- | --- | --- | ---: | ---: | --- |
| GCC strict | GCC 16.2.1, x86-64 | `-std=c2x` | 31/31 | 25/25 | 6/6 `FLAGS_SCREENED_UNQUALIFIED` |
| Clang strict | Clang 23.1.4, x86-64 | `-std=c23` | 31/31 | 25/25 | 6/6 `FLAGS_SCREENED_UNQUALIFIED` |
| Clang sanitizer | Clang 23.1.4 with ASan+UBSan, x86-64 | `-std=c23` | 31/31 | 25/25 | 6/6 `FLAGS_SCREENED_UNQUALIFIED` |

The capture reports `Linux-7.2.14-arch1-1-x86_64-with-glibc2.44`,
`FLT_EVAL_METHOD=0`, C rounding `FE_TONEAREST`, radix 2, mantissas 24/53/64,
and 4/8/16 storage bytes for `float`/`double`/`long double` respectively.
All three controls observations were `x87=0x037f`, `mxcsr=0x00001fa0`,
`fenv_round=0`, issues zero. x87 uses the 64-bit significand precision mode;
MXCSR has nearest rounding, masked exceptions and FTZ/DAZ off. MXCSR bit 5 is
sticky inexact status, not a change to the checked control settings.

The six recorded compilation roles include the *production* virial evaluator,
not just the verification runner. Its observed options contain
`-O1 -g -frounding-math -ffp-contract=off -fno-fast-math` in all three
profiles, with ASan/UBSan compile and link options in the sanitizer profile.
No missing/forbidden arguments were reported under the script's narrow policy.
This is not evidence for uninspected environment variables, compiler wrappers,
LTO, alternative architectures, or future compiler revisions.

## 4. Linked code and generated instruction review

The exact source revision is `ee39cf2040574bb004e4465ae097340e431558a4`.
The manifest identifies the source, production-object, static library, and
final reference-executable digests for each profile, as well as compiler
commands and linker command chains. The local hash-check transcripts report
success for all twelve linked library/executable files and the production
object check succeeded before disassembly capture. **The binary bytes
are not included in the archive**, so the reviewer cannot independently
recompute those object/executable digests from the attachment.

**Instruction-level findings from 18 provided `objdump` text files:**

- All nine object-to-linked evaluator pairs contain the same parsed mnemonic
  sequence, with equal instruction counts. Every changed textual operand is
  associated with a branch/call destination or RIP-relative address/reference
  after link relocation; no other changed operand class was observed.
- The GCC and Clang `float` and `double` evaluators use scalar SSE operations
  including `addss/addsd`, `subss/subsd`, `mulss/mulsd` and `divss/divsd`.
  The `long double` evaluators use x87 load/store, add/subtract, multiply and
  divide operations; sanitizer variants also contain instrumentation calls.
- No FMA, approximate reciprocal/reciprocal-square-root or explicit x87/MXCSR
  control-register-setting instruction was detected in these extracted
  evaluator functions. This is consistent with contraction disabled and the
  stated hardware settings.
- The instruction scan does **not** constitute formal proof of every constant
  encoding, the full call graph, transient thread/fenv state, or byte-for-byte
  source-to-executable equivalence. The archived listings do not include
  independent `.rodata` dumps, relocation tables or the original ELF binaries.

The supported arithmetic inference is restricted to the four short, normal,
dyadic test traces covered by the source proof and passing expected-value
checks. Avoid generalizing from these paths to other input cases.

## 5. Positive, negative and infrastructure evidence

The 10-test reference runner passed in all three builds, including the
required case/family accounting and nine intentionally wrong expected
quantities (three output fields across three scalar families). The six-test
format/fenv preflight and nine-test hardware-control suite both passed.
The reference gate checks hardware state before and after the 36 comparisons;
its two samples are not continuous instruction monitoring.

Tests preserve distinct raw BBTC producer status, reference integrity,
B1 comparison operation status and mathematical PASS/FAIL finding. Deliberate
negative-control FAIL findings are successful *tests of the verifier*, not
successful mathematical reference cases. The archived output reports per-test
success; it does not serialize every individual observed/expected value into
a permanent case-by-case execution report. The accepted B2a contract defers
full fixture serialization and detailed reporting to VV0.1-C; absence of
that wire format alone must not be invented as a B2b failure.

Prior local reports indicated that ordinary production-only builds exclude
VVUQ dependencies. **The committed-source evidence archive does not itself
contain a fresh production-only isolation run**. Recheck that condition during
B2b final acceptance against this exact source revision (§7).

## 6. Qualification boundaries and invalidation

| Question | This review's evidence-based finding |
| --- | --- |
| Four independent analytical expectations | Defined and justified by source proof and accepted design |
| Fixed-case source arithmetic exactness | Conditional source-level argument recorded |
| Strict producer, adapter and comparator flags | All captured commands screened; no toolchain qualification inferred |
| Runtime native format and x87/MXCSR environment | Passed on this host during the recorded tests |
| Required 36 numerical findings / matrix coverage | Regression execution passed; required-case accounting tested |
| Wrong-reference and rejection controls | Focused tests passed |
| Extracted production evaluator instruction review | Favorable for these functions, with archive limits stated |
| General compiler/ABI/fenv certification | **NOT GRANTED** |
| B2b implementation acceptance | **PENDING** separate owner decision |
| Experimental physical-model validation | **NOT CLAIMED** |

Any material change to production evaluation source, adapter, comparison
engine, fixture identities, compiler, toolchain defaults, flags, target
architecture, native floating formats, library linkage, or relevant runtime
control settings requires an explicit impact assessment and, if indicated,
regeneration of supporting evidence. A passing reference does not authorize
relaxing the predeclared EXACT policy afterward.

## 7. Required actions before a formal B2b acceptance decision

1. **Review this draft and the manifest against the original evidence archive.**
   Check repository cleanliness, source revision, reported hashes, reviewer
   conclusions, and the explicit unqualified labels. Any independent reviewer
   should distinguish inspected disassembly text from direct binary inspection.
2. **Perform the final production-only isolation check against `ee39cf2`.**
   Use a fresh CMake build with `BBTC_BUILD_TESTS=OFF`, `BBTC_BUILD_VVUQ=OFF`,
   `BBTC_VVUQ_BUILD_TESTS=OFF`, `BBTC_VVUQ_ORACLE_TESTS=OFF`, and verify that
   only the `bbtc` production library target is built. Keep its log with the
   evidence. This is not a compiler numerical qualification step.
3. **Carry out an explicit B2a-contract-to-B2b-implementation audit** of
   status precedence, public output clearing, borrower lifetime,
   native-family dispatch, fixture/unit identity, negative-control semantics,
   and CMake dependency direction. Regression outcomes are supporting
   evidence, not a substitute for contract mapping.
4. **Record a project-owner decision** (accept scoped implementation, accept
   with recorded limitations, or defer/reject with named blockers), naming
   reviewed commit, evidence archive checksum, profile scope, and residual
   unsupported cases. No acceptance tag is created by this draft.

After review, the report and manifest may be committed as a documentation-only
checkpoint. Subsequent formal B2b acceptance, if warranted, should have its
**own** decision document and acceptance tag. The report commit does not
retroactively change the reviewed implementation baseline `ee39cf2`.
