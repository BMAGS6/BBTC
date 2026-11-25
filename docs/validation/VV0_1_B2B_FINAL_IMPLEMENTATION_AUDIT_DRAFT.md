# VV0.1-B2b — Final implementation acceptance audit (review draft)

| Field | Value |
| --- | --- |
| Record class | Implementation-to-design traceability **audit draft**, not an owner decision |
| Review status | **PROVISIONALLY FAVORABLE; Q1–Q3 OWNER DISPOSITIONS RECORDED; FORMAL B2b ACCEPTANCE NOT GRANTED** |
| Accepted design | `docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md`, v0.1.0 (B2a) |
| Accepted B2a decision | `docs/validation/VV0_1_B2A_FINAL_ACCEPTANCE_DECISION.md` |
| Historical reviewed implementation | `ee39cf2040574bb004e4465ae097340e431558a4` (B2b.3c) |
| Supplemental verified implementation baseline | `a58b13a27c327ca5315d78bbfbc789e9354fffae` (B2b.4a–4c; production virial evaluator) |
| Current technical-audit source HEAD | `2f766ad07085cd28f1d14ad5892d5981f0df967b` (B2b.4d.2 committed; parent `a58b13a`) |
| Current source tree at that HEAD | `758460a7546b6b3fc91ddafe570e7d377e9ed61d` |
| Evaluator source SHA-256 (unchanged) | `da6c511a96e3d1b5ce38427ccc93b180df80b187d54ec8dc70276d73408ce518` |
| Historical documentation-only review commit | `25e7cdb989eaa2a8c3a5d5074b65aa3f6933c012` |
| Source branch | `rewrite/vv0_1b2b_reference_adapter_implementation_v1` |
| Numerical evidence archive | `bbtc_vv0_1_b2b_3c3b_ee39cf2_evidence_complete.tar.gz`; SHA-256 `f5fcec44f6b75229a04ef27a4a1d543e42a3f46db7e318d07f1de7dd666548f9` |
| Supplementary full-build archive | `vv0_1b2b_3c3b_ee39cf2.tar.gz`; SHA-256 `b89f7584af2d99179d86bbfed92483f77bfc7dd089f64ac661c6a083dfae4cad` |
| Production-only pre-acceptance log | SHA-256 `e5b24fd14ca71eb065b54fc2cb62bb7570ed3ace884aa00cad21815b020d3f20` |
| Authority to accept | **Project owner has decided Q1–Q4; separate explicit final milestone sign-off still required** |
| Document review checkpoint | 2026-11-24 (America/New_York); owner reviewed the two drafts and expressed satisfaction with their presentation; no separate milestone disposition recorded |

> **Claim boundary.** This draft examines implementation conformance to the accepted B2a design, for M-01–M-04 and the selected three native precision families on the three documented x86-64 Linux compiler profiles. This is not a general numerical compiler certification, physical EOS validation, trajectory validation, ammunition/firearm safety assessment, or automatic grant of floating-point qualification. `SCREENED_UNQUALIFIED` and `FLAGS_SCREENED_UNQUALIFIED` remain unchanged. **No acceptance tag should be created from this draft.**

## 1. Historical B2b.3c evidence and baseline integrity

1. The reported local pre-acceptance check reached `25e7cdb989eaa2a8c3a5d5074b65aa3f6933c012` on the expected branch with a clean working tree. The only changes between the implementation and review commits were two Markdown records under `docs/validation/`; `CMakeLists.txt`, `include/`, `src/`, `tools/`, and `tests/` were unchanged.
2. **Historical B2b.3c result:** The preserved complete numerical-evidence archive matched the recorded SHA-256. Each fresh GCC, Clang, and Clang sanitizer build reported 31/31 CTest entries passed, including the required matrix, profile, and hardware-control tests. These are **93 CTest executions**, not 93 independent mathematical reference cases.
3. Six monitored translation-unit compilations per profile reported `FLAGS_SCREENED_UNQUALIFIED`; the production virial evaluator's **own** command was included. The reported flags included `-O1 -g -frounding-math -ffp-contract=off -fno-fast-math` and, for the sanitizer build, the sanitizer options. A passing flags screen is not a formal compiler qualification.
4. The separately uploaded **full-build archive contains the compiled object, static-library, and executable bytes**. Independent SHA-256 recomputation from those bytes matched **18/18** captured translation-unit object hashes and **12/12** recorded linked artifact hashes. The three `compile_commands.json` byte hashes also matched their JSON reports. This independently strengthens the *earlier* review's limitation that the smaller evidence-only archive did not contain binary bytes. That older statement remains true about the smaller archive; it is **superseded as a restriction on the combined evidence set**.
5. The checked-in technical review describes nine matched production-object/linked-executable mnemonic sequences; a separate binary-based `objdump` scan over the supplied originals found corresponding evaluator function bodies in both files, with no obvious FMA, approximate reciprocal, or floating-point control-setting instructions. These narrow instruction observations do not prove all called routines, every constant, or every transient execution state.
6. The corrected pre-acceptance record verifies a production-only Release library build. All four VVUQ/test CMake options in the archived production-only `CMakeCache.txt` are `OFF`. No developer-only targets matched the correctly anchored target-name search; the archived production `libbbtc.a` defines none of the `bbtc_vv_` symbols. The preceding false alarm came from a case-insensitive grep matching the **directory name** `BBTC_VVUQ_B2b` in an ordinary root `CMakeLists.txt` target. Both the false alarm and its correction are retained as evidence.

The production-only Release archive is evidence of **dependency isolation**, not evidence that its compiler options satisfy the same strict-FP numerical profile as the three developer verification builds.

## 1A. Supplemental B2b.4a–4d evidence and revision provenance

The B2b.3c results above remain historical evidence for their *original* commits and build configurations. The following later results do not retroactively change those original artifacts or their 31/31 CTest count.

| Increment / artifact | Identity and observed scope | Audit status |
| --- | --- | --- |
| B2b.4a — independent rational oracle and native EXACT-neighbor checks | Independent dyadic reference traces for M-01–M-04; 36 fixed-case decisions; reported 114 adjacent-representable-value probes and signed-zero semantics; source-based test and review evidence | **Favorable; limited to defined analytical cases** |
| B2b.4b — controlled mutation and live FP control rejection | Per compiler: 15 detected required mutations plus three deliberately surviving left-clamp controls; six real x87/MXCSR/fenv rejection challenges. Preserve **DETECTED** versus **SURVIVED** as separate categories | **Favorable; survivors assessed in B2b.4d.3, not reclassified as detected** |
| B2b.4c.1–2 — strict compiled evidence and independent binary review | `bbtc_vv0_1_b2b4c1_a58b13a_evidence.tar.gz`, SHA-256 `a320c3df37867361bfdf33c287de619ff5236c3d71d91a7315e5d99848832c08`; 60/60 manifest entries checked; nine producer function listings examined across GCC strict, Clang strict, Clang ASan/UBSan | **Favorable artifact review; no full compiler certification** |
| B2b.4c.3 — production-only isolation | `vv0_1b2b4c3_prodonly_a58b13a.tar.gz`, SHA-256 `cb8f103951562347200f8131f5120da6a688e960744dd42c200105abd6c72f87`; 20 production objects, tests/VVUQ OFF, no defined or unresolved developer-only `bbtc_vv_` symbols | **Scoped production isolation PASS** |
| B2b.4c.4 — public C23 consumption | `vv0_1b2b4c4_external_consumer_a58b13a.tar.gz`, SHA-256 `20e9e111c57969d3be8b90896241d886c25ed561929b18163b075651fd94af0d`; 13/13 evidence hash entries; direct-link and `add_subdirectory` CMake consumer, one CTest | **Scoped external-consumer PASS; not an installed `find_package` test** |
| B2b.4c.5 — compiler-interoperability consumer | `bbtc_vv0_1_b2b4c5_a58b13a_evidence.tar.gz`, SHA-256 `2c2496bcc7a0d72f5f0758f2986110ac5e0fc3bd9c18cf52f05299a01ab17260`; GCC and Clang consumers linked the *same GCC-built library*; 16/16 internal hashes plus provenance hash | **Scoped interoperability PASS; not universal ABI certification** |
| B2b.4d.2 — integrated live gate | Commit `2f766ad07085cd28f1d14ad5892d5981f0df967b`; changes only `tests/validation/test_virial_reference_matrix.c` and `docs/validation/VV0_1_B2B4D2_LIVE_REFERENCE_GATE_PLAN.md`; GCC, Clang, Clang+ASan/UBSan fresh builds and focused cmocka cases | **Reported 33/33 CTests/profile (99/99 total); 2 new real-hardware gate cases/profile. Full binary/log archive not independently supplied** |
| B2b.4d.3 — clamp reachability and binary corroboration | `bbtc_vv0_1_b2b4d3_clamp_evidence_2f766ad.tar.gz`, SHA-256 `0c203b4884100c83a1e1fcb85b73038671d8841b71440ca9f350429d8e316bff`; 27/27 manifest hashes, nine disassemblies matched regenerated earlier evaluator listings | **Proposed conditional left-clamp mutant equivalence** |

The B2b.4c.3 and B2b.4c.4 production library copies share SHA-256 `d2a665e1431e133e742653de66d4594b8a8880526d9933bd289b8116e8423d8d`. Their byte identity strengthens the isolation-to-consumer evidence chain. The three profiles used `-O1 -g -frounding-math -ffp-contract=off -fno-fast-math` with sanitizer additions where applicable. The compiler/test evidence is specific to the observed x86-64 Linux formats (`float` binary32, `double` binary64, and 80-bit extended `long double` stored in 16 bytes).

**Evidence caveats.** The B2b.4d.2 test count comes from the supplied execution transcript and published two-file commit; unlike the independently audited B2b.4c archives, no complete new B2b.4d.2 executable-and-log archive has been reviewed. B2b.4d.3's new archive holds textual disassemblies and recorded object hashes, not the newer `.o` bytes; the nine-way listing match is against earlier object bytes. Neither claim should be described as a complete new-binary byte-level audit.

### B2b.4d.3 surviving clamp mutation disposition

For validated native-family inputs, `0 < Tmin < Tmax` and `Tmin <= T <= Tmax`. With the actual sequence of native subtraction, division, exact doubling and subtraction of one, monotonic correctly rounded operations and gradual underflow maintain `-1 <= x <= 1`. Under the recorded strict floating-point profiles and admitted hardware modes, changing only the test `x < -1` to `x < -2` is therefore **conditionally semantically equivalent on the admitted input domain**. The three observed survivors remain three survivors in mutation statistics; **they are not three additional detected mutants**. This is a scoped mathematical/source/instruction argument, not a proof for changed compiler settings, all architectures, concurrent fenv mutation, arbitrary inputs, or the whole numerical library.

The producer comment explaining endpoint clamp activation by roundoff warrants later documentation review, but **no production source alteration is part of this audit**. Leave the guards and all accepted test expectations unchanged.

## 2. Accepted B2a-to-B2b traceability matrix

The dispositions below are **technical provisional findings**, not formal owner acceptance. “Supported” refers to inspected source and bounded evidence from the historical and supplemental campaigns, as specifically identified. The evidence is not extended to new configurations or untested inputs.

| Gate | Governing B2a design requirement | Implementation and test anchors | Provisional disposition |
| --- | --- | --- | --- |
| A01 | Source-identified selected public API and limited subject (§§3–4) | `include/bbtc/internal_ballistics/first_order_virial_gas_model.h`; `src/internal/first_order_virial_gas_model.c`; `docs/validation/references/VV0_1_B2_REFERENCE_PROVENANCE.md` | **Supported** for the virial coefficient temperature-law API only |
| A02 | Independent M-01–M-04 analytical references, off-center M-04, native quantities/units (§§4.1–4.3, 5) | `tests/validation/test_virial_reference_matrix.c`; B2b.3c.1 source proof; B2b.4a independent rational oracle | **Supported** for four specified analytical cases, not arbitrary model inputs |
| A03 | Three typed native families without binary64 intermediary (§§4.1, 6.3, 8) | `tools/bbtc_vv/virial_adapter.c` and `.h`; `tests/validation/test_virial_adapter.c`; reference-runner typed fixtures | **Supported** for observed `float`, `double`, `long double` entrypoints |
| A04 | Borrowed coefficient lifetime and exact raw execution-status propagation (§§4.4, 6.2–6.3) | Thin adapter forwards caller-owned law and terms, returns original status; `test_virial_adapter.c` and `test_virial_adapter_contract.c` | **Supported** for exercised API calls; no runtime claim for invalid non-null storage |
| A05 | Required pointer, NaN/Inf, domain, status precedence, and cleared-output contracts (§§4.4, 6.3) | `tests/validation/test_virial_adapter_contract.c`: 15 cmocka cases, five per family, with reusable sentinel outputs; production API implementation/header | **Supported** for the tested defined-input contract paths |
| A06 | Retain separate observation validity, production status, B1 status and numeric finding (§§6–7) | `virial_adapter.c`; reference-runner evidence types and `check_complete_evidence()` | **Supported** in inspected data-flow and required-case assertions |
| A07 | Admissibility gate for missing/invalid reference, wrong unit/quantity, producer error, and unexecuted case (§§5, 6.3, 7, 9) | `test_reference_integrity_rejection`, `test_failed_observation_gates_comparison`, `test_required_case_accounting`, `test_invalid_comparison_policy` | **Supported** by negative-control implementation and passing test reports |
| A08 | Predeclared per-quantity EXACT, with 36 required decisions and intentional wrong-reference FAIL (§§4.3–4.4, 8–9) | Required matrix and negative controls; B2b.4a rational and adjacent-value oracles; B1 comparison API | **Owner approved narrowly scoped M-01–M-04 EXACT-evidence admissibility on 2026-11-20; see §3 and separate decision draft** |
| A09 | Compiler/ABI/native format and fenv review before scoped EXACT claims (§§4.3, 8–9) | B2b.3c.1–3c.3b; B2b.4c.1–4c.5 compiled/link/consumer archives; B2b.4d.2 live reference-gate cases; 4d.3 clamp review | **Favorable tested-configuration evidence; owner approved narrow Q1 evidence scope, not blanket compiler qualification** |
| A10 | Developer-only dependency direction, no public ABI modifications, no mandatory cmocka/Python (§§2, 6, 9) | Root and developer CMake files; B2b.4c.3 production-only build, B2b.4c.4–5 external C23 consumers | **Supported** for the tested production-only and subproject-consumer configurations |
| A11 | Clear distinction between implementation checking and empirical physical validation (§§1–2, 5, 7) | Mathematical-only case scope; qualification report and test comments; no fabricated measurements | **Supported** as scope separation |
| A12 | Full fixture/schema/report interchange and general CI acceptance policy deferred (§§2.2, 5.1, 10–11) | No B2b requirement to implement a stable fixture wire schema | **Deferred/out of B2b scope**, not a missing feature |
| A13 | Explicit exact revision, evidence, reviewer disposition and owner approval (§§10–11) | Historical `ee39cf2`/`25e7cdb`, supplemental `a58b13a`, committed live gate `2f766ad`, archived hashes in §1A, and this unapproved draft | **OPEN: owner Q1–Q4 choices recorded, but final B2b acceptance declaration and reviewed decision record not yet approved** |

## 3. Points requiring a decision, not more indiscriminate testing

### Q1 — Limited EXACT-arithmetic admissibility (owner chose Option A, 2026-11-20)

The accepted design permits required `EXACT` findings only after relevant arithmetic and execution-profile prerequisites have been assessed. The code *executes* the 36 comparisons after format/fenv/hardware **screening**, while the screening components intentionally cannot return `QUALIFIED`. The independent rational source traces, adjacent-value/signed-zero negative controls, compile-command evidence, actual binary identities, sampled disassembly, public-consumer checks, new real-hardware runner rejection cases and conditional clamp reachability argument add **favorable evidence for the four fixed dyadic cases**. They did not independently overturn the *historical* Q1 deferral or grant blanket compiler/ABI/fenv qualification. **The project owner subsequently selected Option A on 2026-11-20:** admit the specified 36 M-01–M-04 EXACT findings as bounded implementation-verification evidence for the documented configurations only.

That Q1 choice is **recorded as a bounded evidence-admissibility decision**, not standalone formal B2b milestone acceptance. It covers only the identified cases and tested profiles subject to the listed assumptions and limitations. It changes no API's `SCREENED_UNQUALIFIED` output; those enum values refer to the probes' restricted authority. Do **not** rename them to `QUALIFIED` or treat green CTest as universal authorization.

### Q2 — Independence and review limits

The reference values derive from polynomial identities and an independent rational trace rather than production Clenshaw code; however, the test harness, compiler and host environment are not independent experimental calibration. The detailed arithmetic proof remains conditional on compiled execution assumptions. The binary-based checks strengthen artifact traceability but do not constitute a formal proof of the whole call graph or of arbitrary-input correctness. **Owner decision (2026-11-20): YES**, accept these documented residual independence and review limits for the scoped B2b evidence. Acceptance of a limitation does not make that limitation disappear.

### Q3 — Acceptance scope

An affirmative B2b decision, if ultimately made, must be **implementation acceptance only**: developer-only adapter, fixed mathematical matrix, contract tests and validation infrastructure at the identified revisions/profiles. It is not acceptance of physical/empirical virial EOS validity, complete internal or external ballistics, ammunition or structural safety, or future toolchains. Source/toolchain/flags/ABI/fenv/target changes require an impact review and, where appropriate, refreshed evidence. **Owner decision (2026-11-20): agrees with this implementation-only scope (Q3).**

### Q4 — Owner authority and history

The original reviewed B2b implementation content is `ee39cf2`; the two-file technical review at `25e7cdb` added no implementation changes. Later numerical-evidence baseline `a58b13a` and integrated-gate commit `2f766ad` must be recorded *separately*, not substituted for the older source-history statements. This draft and any later owner decision must record their own final Git commits without retroactively changing the source baseline. Preserve the previously accepted B1 and B2a records. **Owner decision (2026-11-20): Q4 requires a separate final acceptance record.** A decision-record candidate is prepared for review; do not issue an annotated acceptance tag before the owner separately approves the complete milestone disposition and it is committed.

## 4. Consolidated technical disposition and next steps

**Proposed technical finding:** No observed B2b implementation nonconformance on the exercised defined-input paths; fixed-case numerical findings, fail-closed admission checks, linker/consumer evidence, and conditional surviving-mutant analysis are favorable within the recorded x86-64 Linux profiles. **The technical evidence review remains a draft.** Owner choices on Q1 (scoped admissibility), Q2 (accepted bounded residual limits), Q3 (implementation-only scope), and Q4 (separate record) are now explicit. A13 and final B2b milestone acceptance still require review and sign-off on the separate decision record. A passing implementation test must not be represented as acceptance.

**Do not silently change** the reference cases, thresholds, exception handling, source-level comparison semantics, production ABI, or previously accepted B1 behavior to force acceptance. If an actual discrepancy is identified, create a separately reviewed corrective increment and recapture the affected evidence.

Before an acceptance **commit or tag**:

1. Review the revised A01–A13 mapping against the accepted B2a contract and current source, including the two-file B2b.4d.2 diff. Correct any unsupported row rather than rubber-stamping it.
2. Preserve the owner's **Q1 Option A** scoped evidence-admissibility choice and **Q2/Q3** limitations as recorded on 2026-11-20. No compiler certification or scientific validation is implied. Confirm that the decision record describes the actual reviewed code and evidence rather than expanding the claim.
3. Review the separately prepared `docs/validation/VV0_1_B2B_FINAL_ACCEPTANCE_DECISION.md` candidate. Record a **separate explicit owner milestone acceptance or deferral** after reviewing its complete scope and conditions; do not manufacture that additional sign-off from the earlier Q1–Q4 policy choices.
4. Review and commit the separate decision documentation. Only after an affirmative, documented decision, consider a scoped annotated B2b acceptance tag.

**Owner Q1–Q4 decision ledger (not yet formal B2b milestone acceptance):**

| Question | Owner disposition recorded 2026-11-20 | Remaining action |
| --- | --- | --- |
| Q1 | **OPTION A APPROVED — bounded M-01–M-04 EXACT-evidence admissibility** | Confirm exact cases, ABI, flags, environment and exclusion wording in the separate record |
| Q2 | **YES — documented residual independence/review limits accepted** | Preserve limitations without implying they were eliminated |
| Q3 | **AGREED — implementation-only scope** | Record exclusions and revalidation triggers |
| Q4 / A13 | **SEPARATE RECORD REQUESTED; A13 STILL OPEN** | Final owner sign-off on full B2b milestone; reviewed documentation commit before any acceptance tag |

**Evidence-control note:** This addendum is a documentation *candidate* against the earlier 83-line draft (SHA-256 `74446a67656312554fcaa4afef9419173588ce3162e9e480ca3ebf4fcff0dc28`). Q1–Q4 owner choices are recorded by the proposed decision document; committing a draft alone does not grant formal B2b milestone acceptance. Any independent reviewer should verify the exact latest commit and local worktree again before applying this revision.

**Current milestone decision:** **DOCUMENT REVIEW RECORDED; FINAL OWNER SIGN-OFF PENDING — NO AUTOMATIC B2b ACCEPTANCE, UNIVERSAL QUALIFICATION, OR TAG.**
