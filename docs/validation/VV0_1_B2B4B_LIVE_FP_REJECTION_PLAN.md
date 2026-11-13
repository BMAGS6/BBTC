# VV0.1-B2b.4b Stage 2 — Live hardware floating-point rejection verification

| Field | Value |
| --- | --- |
| Phase | `VV0.1-B2b.4b`, Stage 2 |
| Record | **IMPLEMENTATION/TEST PLAN — Q1/Q2 REMAIN DEFERRED** |
| Intended base commit | `798893c962dd9828cdcbceec89e8d5d021e9da35` |
| Existing authority | B2a mathematical-reference contract; B2b.3c.2 profile and B2b.3c.3a hardware-control screens |
| New test source | `tests/validation/test_virial_live_fp_rejection.c` |
| New CTest entry | `bbtc.vv.virial_live_fp_rejection` |
| Executable audience | Developer-only x86-64 Linux, GCC or Clang, POSIX `fork()` |
| Production modification | **None** |

## 1. Assurance question and exclusions

The existing hardware-control tests validate copies of x87 and MXCSR register
observations, and separately take a read-only live observation. This increment
asks whether a **real, modified processor control state** is correctly sampled
and rejected by the actual hardware-screening path. It does not assert that
one altered state corrupts an M-01 through M-04 numerical result.

A verification precondition failure is **not** a B1 numerical FAIL, and a
successful adversarial test is **not** one of the 36 admitted reference
comparisons. A screen can report `SCREENED_UNQUALIFIED`, never `QUALIFIED`.
The increment does not qualify compiler-generated instructions, hardware
behavior during arbitrary evaluation, other architectures, or physical models.

## 2. Execution architecture and safety properties

1. A single-threaded cmocka test process first demands a real successful
   read-only control screen on the parent. An unsupported baseline is a test
   failure, **never** an uncounted skip or a fabricated control record.
2. Each of six challenges forks a fresh child, which saves both x87 and MXCSR
   state before writing any controls. The child uses the existing
   `bbtc_vv_virial_control_screen()` to observe and classify **actual hardware**.
3. The child restores both original register control words, records restoration,
   sends a fixed-size evidence record through a POSIX pipe, and exits with
   `_exit()`; it never calls the ordinary `exit()` cleanup sequence.
4. The parent checks a complete record, normal child exit, actual changed
   bits, a specific mandatory rejection issue bit, child restoration, and
   unchanged parent controls. `EINTR` and short pipe I/O are accounted for.
5. Test mutations never enable floating-point exception traps, never request
   unsafe/unmasked exceptions, and never run arithmetic under FTZ/DAZ solely
   to force a numerical anomaly. Sticky exception status bits are ignored when
   comparing controls, as allowed by the existing hardware-screen contract.
6. A child's control change must not be assumed to survive an `exec()` call.
   All modifications and observations occur in the **same forked child**.
7. No test writes the checked-out source, changes a production target, or adds
   new runtime dependencies to `bbtc::bbtc`. The target is present only when
   VVUQ developer tests are enabled on a supported host.

## 3. Predeclared individual rejection cases

| Test | Actual control perturbation | Required evidence |
| --- | --- | --- |
| C rounding | `fesetround(FE_UPWARD)` | C rounding rejected by both profile and hardware checks |
| x87 rounding | Load x87 control word with upward rounding | `X87_ROUND` present |
| MXCSR rounding | Load MXCSR with upward rounding | `MXCSR_ROUND` present |
| x87 precision | Set a reduced x87 precision-control encoding | `X87_PRECISION` present |
| MXCSR FTZ | Set real FTZ bit | `MXCSR_FTZ` present |
| MXCSR DAZ | Set real DAZ bit | `MXCSR_DAZ` present |

Additional issue bits are permitted: for example, `fegetround()` can observe
an inconsistent x87/SSE rounding configuration. The named bit for the
specific control under test is mandatory regardless of additional bits.

The accepted host must have nearest rounding in C, x87 and MXCSR, x87 64-bit
precision control, fully masked exceptions and both FTZ/DAZ disabled. If
baseline values are unsupported, this test must fail rather than silently
claim successful rejection coverage.

## 4. Build and first test

Use the same strict flags and x86-64 Linux compiler settings as B2b.4a.
With VVUQ developer tests on and oracle tests off:

```sh
cmake --build "$BUILD"
ctest --test-dir "$BUILD" \
    -R '^bbtc\.vv\.virial_live_fp_rejection$' \
    --output-on-failure -V
ctest --test-dir "$BUILD" --output-on-failure
```

The new target has **six cmocka cases**. The expected integrated suite count
on the existing three profiles increases from 32 to 33 CTest entries. Retain
configure, build, focused test, full CTest output, and `compile_commands.json`
per profile. Repeat with GCC strict, Clang strict and Clang ASan/UBSan.
Check production-only target isolation separately before final acceptance.

## 5. Acceptance and future-work boundary

- All six real-control challenges must run and produce `UNSUPPORTED` with
  their required cause; neither child crashes nor setup failures count as a
  successful rejection.
- The parent retains an admissible control state after every child exits.
- Three compiler profiles must pass the focused and full regressions with
  reproducible build flags and logs; an unsupported host is a **failure** for
  this specifically x86-64 scoped verification checkpoint.
- This stage validates the **hardware screen** under real perturbed controls.
  It does not directly execute `test_complete_reference_matrix` under those
  controls, nor demonstrate that screen status remains stable at every
  instruction of the production evaluator. These are explicit residual
  obligations to consider at B2b.4c/4d.
- Owner Q1/Q2 remain deferred. No result or document from this increment
  grants universal EXACT admissibility, physical validity, or B2b acceptance.
