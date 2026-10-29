# VV0.1-B1b.3d-3a — Numerical Capability Gate Review

**Status:** support-scope correction; final B1 acceptance **OPEN**.

**Governing contract:** `VVUQ_COMPARISON_CORE_CONTRACT.md` version 0.1.0.
**Audit link:** `VV0_1_B1B3D2_ACCEPTANCE_AUDIT.md` obligations A08–A11.

## 1. Finding and corrected scope

Previous B1b.3d-2 evidence qualified one host family: ISO C23 on Arch Linux
x86-64, `FLT_RADIX == 2`, `FLT_EVAL_METHOD == 0`, binary32 `float`, binary64
`double`, and an x87 `long double` with 64 significand bits and 16-byte
storage. The previous source also accepted an **unqualified** alternative:
53-significand-bit, binary64-backed `long double`.

The exact dyadic and rational-rounding kernels now both require the recorded
evaluation method and only permit the tested padded x87 extended long-double
profile. The independent capability probe has the same evaluation-method
requirement. The exact metric constructor inherits the dyadic gate, and the
complete API delegates to the checked components.

The changes restrict acceptance of *environments*; they do not alter exact
arithmetic, nearest-even rounding, threshold policy, or outputs on the
previously qualified primary machine.

## 2. Failure and environment contracts

For structurally valid input on an unqualified format, the exact dyadic kernel
returns `BBTC_VV_DYADIC_UNSUPPORTED_ENVIRONMENT`, leaving its output false.
The outer developer API maps this to
`BBTC_VV_STATUS_UNSUPPORTED_FP_ENVIRONMENT` and preserves canonical cleared
result values. The independently callable rounding kernel performs its own
format check and returns the same public-facing environment status, with
provided state/value outputs cleared. Null pointers and invalid methods keep
their established higher-priority structural-validation behavior.

Checking `FLT_EVAL_METHOD == 0` is **necessary for the current qualification**
but not by itself sufficient to certify arbitrary hardware, compiler flags,
libraries or runtime environment. Unmasked floating-point traps, noncanonical
x87 representations, denormals-are-zero/flush-to-zero changes, unsafe fast-
math, and all unreviewed formats remain excluded. The patch makes no claim to
dynamically recognize every possible unsupported trap state.

## 3. Qualification requirements

The candidate must pass the existing reproducible acceptance campaign:
strict optimized GCC and Clang (nine CTest entries each), Clang Debug under
ASan/UBSan (nine entries), the independent Python exact-rational oracles,
API-symbol checks, source-hash capture and production-only build separation.
The new `platform_capability_probe.c` evaluation-method gate must print
`PRIMARY_FORMAT_PROFILE_MATCH=1` under both primary compilers.

Those tests demonstrate regression on the **qualified** host. They do not
constitute a negative runtime test under binary64-backed long double; that
configuration remains explicitly out of scope pending ABI-specific review.

## 4. Audit disposition

- A10: the extra long-double configuration is no longer inadvertently admitted
  and the expected floating-point evaluation method is now checked.
- A08/A09/A11: native semantics, fenv restoration, and exclusions still require
  final source-by-source evidence and explicit review dispositions.
- A01–A07/A12–A14: not automatically certified by this narrow scope correction.

The final A01–A14 acceptance matrix remains **OPEN** until separate review.
Nothing here validates BBTC physical predictions or modifies production code.
