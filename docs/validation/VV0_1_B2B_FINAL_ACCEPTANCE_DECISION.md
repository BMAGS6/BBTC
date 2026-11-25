# VV0.1-B2b — Scoped Implementation Acceptance Decision (review draft)

| Field | Value |
| --- | --- |
| Record class | Project-owner decision record **candidate**, separate from the implementation-to-design audit |
| Status | **Q1–Q4 OWNER CHOICES RECORDED; FINAL B2b MILESTONE ACCEPTANCE PENDING** |
| Owner choices communicated | 2026-11-20 (America/New_York) |
| Repository | `BMAGS6/BBTC` |
| Implementation branch | `rewrite/vv0_1b2b_reference_adapter_implementation_v1` |
| Exact reviewed implementation HEAD | `2f766ad07085cd28f1d14ad5892d5981f0df967b` |
| Git source tree at that commit | `758460a7546b6b3fc91ddafe570e7d377e9ed61d` |
| Unchanged evaluator source SHA-256 | `da6c511a96e3d1b5ce38427ccc93b180df80b187d54ec8dc70276d73408ce518` |
| Earlier B2b.3c implementation | `ee39cf2040574bb004e4465ae097340e431558a4` |
| Later source/evidence checkpoint | `a58b13a27c327ca5315d78bbfbc789e9354fffae` |
| Governing design | `docs/validation/VVUQ_REFERENCE_ADAPTER_DESIGN_CONTRACT.md` v0.1.0, accepted B2a |
| Accompanying technical audit | `docs/validation/VV0_1_B2B_FINAL_IMPLEMENTATION_AUDIT_DRAFT.md` |
| Document review checkpoint | 2026-11-24 (America/New_York); owner reviewed both drafts and expressed satisfaction with their presentation |
| Final owner milestone approval | **Not yet separately declared** |
| Decision-documentation commit | **Not yet created; do not invent a commit SHA** |

> **Authority and scope.** This record captures the owner's affirmative choices about Q1,
> Q2, and Q3 and preference for a separate final record (Q4). The separate
> *milestone-level* B2b ACCEPT/DEFER decision still requires review of this
> completed record and an explicit statement from the project owner. No
> acceptance commit or tag, compiler certification, physical-model validation,
> or firearm/ammunition safety finding is created by this draft.

## 1. Owner dispositions on Q1–Q4 (2026-11-20)

| Question | Owner choice | Recorded meaning |
| --- | --- | --- |
| Q1 — Fixed-case EXACT arithmetic | **Option A accepted** | Admit the specified 36 M-01–M-04 native-family EXACT findings as *bounded implementation-verification evidence* for the reviewed x86-64 Linux profiles and execution conditions below, not universal arithmetic qualification |
| Q2 — Independence and review limits | **Yes; accepted as residual limits** | Mathematical references are independently derived, while full compiler/call-graph proof, external physical calibration and additional artifact independence are not established |
| Q3 — Acceptance boundary | **Agreed** | Any eventual B2b acceptance is limited to the developer-only implementation, exact reference cases, contract tests, fail-closed screens and verified integration at the identified revisions and profiles |
| Q4 — Decision form | **Separate acceptance record requested** | Preserve the accepted B1/B2a histories; formal milestone disposition must be separately recorded, reviewed, and committed |

The owner choices above supersede the *decision status*, but **not the historical
facts**, in the older audit's Q1 deferral and pending Q2/Q3 ledger. These
choices do **not** mean that the final B2b milestone itself has already been
formally accepted.

## 2. Precise admissibility conditions (Q1)

The bounded finding covers **M-01, M-02, M-03, and M-04 only**, each exercising
three independently derived quantities (`B(T)`, `dB/dT`, and `d²B/dT²`) in each
of the three native C scalar families (`float`, `double`, `long double`):
**4 × 3 × 3 = 36 required EXACT findings**. Wrong-reference controls establish
expected mathematical FAIL behavior and are not additional required PASSes.

The supporting execution conditions are:

1. The reviewed source history up to implementation commit `2f766ad` and the
   recorded production evaluator source bytes; no unreviewed model/adapter/
   comparator/fixture changes.
2. The evidenced **x86-64 Linux** GNU **16.2.1** strict,
   Clang **23.1.5** strict, and Clang **23.1.5** ASan/UBSan verification
   profiles using `-O1 -g -frounding-math -ffp-contract=off -fno-fast-math`
   with the recorded sanitizer flags where applicable.
3. The observed native representation: binary32 `float` (24 significand bits),
   binary64 `double` (53), and 80-bit-extended `long double` (64 significant
   bits, 16 bytes of storage); this is not a promise about another ABI.
4. Passing live input/output environment screening of native format and
   floating-point controls, including nearest rounding, reviewed x87/MXCSR
   conditions and no FTZ/DAZ, as required by the tested profile. A failed
   screen, invalid fixture, missing observation, failed producer call, or
   failed B1 operation never becomes an admissible numerical PASS.
5. The previously captured compiler commands, source/object/binary evidence,
   rational and adjacent-value tests, mutation outcomes, and external consumer
   evidence are considered **together**, with the caveats below. The screen's
   result `SCREENED_UNQUALIFIED` (and compile screen
   `FLAGS_SCREENED_UNQUALIFIED`) remains unchanged: its name accurately
   describes the *probe*, not this separately bounded owner evidence decision.

**Scope of this choice:** admissibility of the specified findings as evidence
that the reviewed implementation behaves as required on those fixed analytical
cases. This is **not** a certification of the compiler, a universal EXACT
arithmetic guarantee, or a validation of the underlying physical equation of
state.

## 3. Independent evidence register

The accompanying audit retains the detailed A01–A13 traceability and the
historical B2b.3c results (31 CTests/profile). The later evidence is additive,
not retroactive to those old builds.

| Evidence increment | Artifact or source identity | SHA-256 / finding |
| --- | --- | --- |
| B2b.4c.1–2 strict compile and sampled binary assurance | `bbtc_vv0_1_b2b4c1_a58b13a_evidence.tar.gz` | `a320c3df37867361bfdf33c287de619ff5236c3d71d91a7315e5d99848832c08` |
| B2b.4c.3 production-only isolation | `vv0_1b2b4c3_prodonly_a58b13a.tar.gz` | `cb8f103951562347200f8131f5120da6a688e960744dd42c200105abd6c72f87` |
| B2b.4c.4 external C23 consumer | `vv0_1b2b4c4_external_consumer_a58b13a.tar.gz` | `20e9e111c57969d3be8b90896241d886c25ed561929b18163b075651fd94af0d` |
| B2b.4c.5 GCC/Clang same-library consumer | `bbtc_vv0_1_b2b4c5_a58b13a_evidence.tar.gz` | `2c2496bcc7a0d72f5f0758f2986110ac5e0fc3bd9c18cf52f05299a01ab17260` |
| B2b.4d.2 fail-closed hardware gate | Commit `2f766ad07085cd28f1d14ad5892d5981f0df967b`; provided test transcript | Reported **33/33 CTests/profile** (99/99), plus two added cmocka real-hardware gate cases/profile; the full new executable/log archive has **not** been independently reviewed |
| B2b.4d.3 clamp-equivalence review | `bbtc_vv0_1_b2b4d3_clamp_evidence_2f766ad.tar.gz` | `0c203b4884100c83a1e1fcb85b73038671d8841b71440ca9f350429d8e316bff` |

Historical B2b.3c numerical and full-build archive hashes remain in the audit;
those archives must not be relabeled as B2b.4d.2 builds. A separate strict
production-library snapshot was byte-identical between B2b.4c.3 and 4c.4
(SHA-256 `d2a665e1431e133e742653de66d4594b8a8880526d9933bd289b8116e8423d8d`).

## 4. Q2 — Accepted limitations and explicit nonclaims

The owner accepts the following **as limitations of this narrow milestone**,
not as verified absence of risk:

- No independent third-party certification and no experimental physical
  calibration or validation of a virial EOS, projectile trajectory, or safety.
- Sampled compiled-instruction and relocation inspections do not prove every
  constant, every callee, all execution paths, or all transient fenv states.
- The live-hardware admission demonstration uses identified x87/MXCSR
  mutations on x86-64 Linux, not arbitrary architectures, threads or external
  fenv changes. It is a sampled gate, not continuous monitoring.
- The 15 detected and **three surviving** left-clamp mutants per compiler
  retain their observed categories; the latter have a conditional
  domain/profile-specific equivalence argument, not a claim of detection.
- B2b.4d.2's 99/99 tests are established by the user-supplied execution log
  and committed source; the new binary set has not been independently
  hash-audited. B2b.4d.3's disassembly archive does not include those new
  object bytes.
- External C23 consumer and GCC/Clang interoperability checks apply to the
  exercised interfaces and matching host ABI, not installed-package behavior
  or general binary compatibility.

None of these conditions authorizes suppressing future failures, weakening
reference integrity, relaxing EXACT expectations, or bypassing a failed
execution-environment screen.

## 5. Q3 — Accepted implementation-only boundary

An eventual affirmative B2b disposition would accept **only**:

- Developer-only BBTC observation adapter and existing public API invocation;
- the four predeclared analytical fixtures, typed runner, B1 EXACT comparison
  integration and required-case/negative-control accounting;
- exercised validation/status/output-clearing and borrowed-array contracts;
- the documented environment screening, developer CMake integration and
  tested production/developer dependency isolation.

It would **not** accept the entire BBTC internal/external-ballistics library,
all possible numerical inputs, new compiler releases, non-x86-64 hosts, new
floating-point formats, an installed CMake package, full fixture interchange,
CI policy, empirical EOS truth, projectile predictive accuracy, or firearm/
ammunition/structural safety. The separately accepted B1 and B2a milestones
remain unchanged. The deferred VV0.1-C fixture/report/CI work is not a B2b
acceptance blocker.

Changes to the production evaluator, adapter, B1 comparator, reference
fixtures, compiler flags, compiler, C ABI, target, hardware-control profile,
or execution policy require an **impact review** and proportionate revalidation
before extending these findings. No production source changes are approved by
this decision record.

## 6. Final B2b milestone decision and publication control (Q4/A13)

**Current disposition: NOT YET RATIFIED — formal B2b milestone acceptance is
PENDING.** The owner explicitly chose the bounded Q1 admissibility option,
accepted Q2 residual limitations, agreed to Q3 scope, and requested a
separate Q4 record. Those choices are not silently rewritten as a separate
unconditional acceptance statement.

For a final affirmative decision, the project owner must review this completed
record and separately confirm an explicit statement materially equivalent to:

> I accept the VV0.1-B2b implementation milestone **only within the Q1–Q3
> conditions and exclusions documented in this decision record**, for source
> commit `2f766ad07085cd28f1d14ad5892d5981f0df967b`. I do not grant
> general compiler certification, physical validation or ammunition/firearm
> safety approval.

After that decision, update the document's status and actual approval date,
record the decision in the audit without erasing historical deferrals, and
commit the reviewed documentation only. The acceptance-documentation commit
must be obtained from Git **after** creation; it is not an alternative
implementation baseline. An annotated acceptance tag (name chosen from the
actual final approval date) may be considered **only after** the owner-approved
record is committed and checked. No tag is authorized by this draft.

If the owner instead defers or declines final milestone acceptance, preserve
this record as **DEFERRED** with concrete blockers and no acceptance tag.

**Document revision state: OWNER Q1–Q4 POLICY CHOICES AND DOCUMENT
REVIEW CHECKPOINT RECORDED; FORMAL MILESTONE ACCEPTANCE PENDING.**
