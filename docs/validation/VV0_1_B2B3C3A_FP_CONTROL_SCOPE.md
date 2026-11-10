# VV0.1-B2b.3c.3a — x87/MXCSR runtime control-state screening

| Field | Value |
| --- | --- |
| Phase | **VV0.1-B2b.3c.3a** |
| Status | **IMPLEMENTATION CANDIDATE — NOT FORMALLY QUALIFIED** |
| Expected starting commit | `651dcfa56494444a603eabd046b905501320dc09` |
| Mathematical source argument | `docs/validation/VV0_1_B2B3C1_VIRIAL_EXACT_ARITHMETIC_PROOF.md` |
| Compiler-command evidence helper | `tools/bbtc_vv/capture_virial_fp_compile.py` |
| Scope | M-01..M-04, three native C scalar families, strict developer-only builds |

## Purpose

The B2b.3c.1 proof establishes *conditional* arithmetic exactness for four
independently calculated polynomial values and their derivatives. B2b.3c.2
screens ISO C format metadata and `fegetround()` and captures the separately
compiled producer's strict-FP command-line arguments. Neither step directly
reads the hardware control registers used by SSE and x87. In particular,
`fegetround() == FE_TONEAREST` alone does not demonstrate that separately
addressable x87 and MXCSR rounding controls agree.

This increment adds **read-only developer tooling**, not a new physics feature,
production dependency, numeric policy, or scientific acceptance decision.
The observable results are `SCREENED_UNQUALIFIED` and `UNSUPPORTED` only.

## Architecture and execution scope

On GCC/Clang x86-64, or 32-bit x86 compiled with SSE support, the probe reads
`fnstcw` and `stmxcsr` into separately typed integer fields. This is a
per-thread **point-in-time** observation. It neither sets rounding nor changes
exception masks, sticky flags, x87 precision-control, FTZ, or DAZ.

On other targets or toolchains, the instruction path is not emitted and the
hardware observation is `UNSUPPORTED`. The developer-only profile test checks
that this outcome is detected; the mandatory mathematical-reference test
**fails** if the actual execution environment is unsupported. Unsupported must
never count as a numerical PASS or a skipped, green mathematical case.

This currently restricts a *candidate admitted test profile* to conventional
x87 precision control (`PC=11`) and masked exceptions, and SSE with all
exception masks enabled. It deliberately does not claim that these settings
are strictly necessary for exactness of the four small, normal, dyadic cases.
A broader profile requires independent justification and review.

## Register-bit requirements

| Source | Bits | Interpretation | Admitted value |
| --- | --- | --- | --- |
| ISO C | `fegetround()` | C rounding mode | `FE_TONEAREST` |
| x87 CW | 0..5 | Six exception mask bits | All `1` |
| x87 CW | 8..9 | Precision control | `11` (64-bit significand) |
| x87 CW | 10..11 | Rounding control | `00` (nearest) |
| MXCSR | 6 | DAZ, denormals-are-zero | `0` |
| MXCSR | 7..12 | Six exception mask bits | All `1` |
| MXCSR | 13..14 | Rounding control | `00` (nearest) |
| MXCSR | 15 | FTZ, flush-to-zero | `0` |
| MXCSR | 0..5 | Sticky exception status | Recorded, **not screened for zero** |
| MXCSR | 16..31 | Outside the reviewed architectural state encoding | All `0` |

A typical masked, nearest, extended-precision x87 control word is `0x037f`;
a typical SSE control/status word with no sticky flags is `0x00001f80`.
**Neither literal is hard-coded as the expected *complete* register value**:
reserved/other x87 bits and MXCSR sticky status are not used to fabricate
an unsupported numerical error. Negative controls mutate one component at
a time in an in-memory fixture and do not change processor registers.

## Reference-matrix integration

The full 36-field matrix performs its existing B2b.3c.2 format/ABI check,
then requires the new hardware screen. After the matrix it samples controls
again and requires the control bits to remain unchanged. MXCSR sticky status
bits 0..5 are intentionally excluded from this *control-state preservation*
comparison; evaluating floating-point expressions may set status indicators.
The x87 control word is compared as a complete 16-bit value.

**Limitations:** sampling before and after the matrix does not continuously
monitor every instruction, prove the absence of transient hardware-state
changes, demonstrate the producer's actual optimized assembly semantics, or
independently establish the link between a recorded object digest and a
running instruction address. Stronger claims require compiler/linked-code
review and explicit qualification authority. A passing 36/36 numerical
comparison remains *implementation verification*, not physical validation.

## Required verification and evidence

1. Start from the expected clean repository revision before applying.
2. Build fresh strict-FP GCC, Clang, and Clang ASan/UBSan configurations.
3. Confirm all CTest entries pass and the control-state test reports raw
   x87/MXCSR/fenv evidence (for example with `ctest -V`).
4. Confirm adversarial tests reject x87 precision/rounding/mask and MXCSR
   rounding/mask/FTZ/DAZ defects, while accepting mere sticky status flags.
5. Re-run `capture_virial_fp_compile.py` and independently review the
   `compile_commands.json` entry for **production** evaluator objects.
6. Verify a production-only build excludes `bbtc_vv_virial_control` and
   the new `bbtc.vv.virial_fp_control` test.
7. Preserve source/object/executable hashes, disassembly, test output, control
   observations, and reviewer disposition in the B2b.3c.3 evidence record.

A passing screening run must remain labeled **SCREENED_UNQUALIFIED** until
the separate evidence and acceptance review. No test here permits changing
EXACT into a relaxed comparison tolerance, assigning fabricated physical
uncertainties, or claiming ballistics/firearm safety certification.
