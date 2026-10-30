# VV0.1-B1b.3d-3c-2 — Independent Oracle and Evidence-Integrity Audit

**Disposition:** conditional source-review evidence for **A13** (oracle independence)
and **A14** (reproducibility/evidence integrity). **B1 remains unaccepted.**
The exact-metric oracle has been tightened and its protocol validator given
mutation-oriented self-tests. Acceptance requires fresh GCC/Clang/ASan+UBSan
host evidence and review of the *committed* final source, not merely this note.

**Scope:** the developer-only numerical comparison core. These tests do not
validate physical models, ballistic experiments, units, measurement provenance,
uncertainty budgets, or firearm safety. No numerical C kernel is changed.

## 1. Source boundaries and reference independence

| Test source | Reference method | C boundary | Independence observation |
| --- | --- | --- | --- |
| `test_exact_dyadic_oracle.py` | Independently generated exactly represented dyadic operands; Python `fractions.Fraction` comparisons | `dyadic_oracle_driver.c` | Does not implement C's signed 1,036-limb grid or partial-product sign algorithm |
| `test_exact_metric_oracle.py` | Python `Fraction` difference and exact quotient | `exact_metric_oracle_driver.c` | Shares the **test-data generator** from the dyadic oracle, not its arithmetic decision; no C limb code used |
| `test_exact_round_oracle.py` | Python arbitrary-precision `divmod` of a scaled rational, remainder-based nearest-even | `exact_round_oracle_driver.c` | Does not use C's virtual-bit-grid comparison or coefficient binary search |
| `test_comparison_api_oracle.py` | Independent Python contract interpreter and `Fraction` rounded diagnostic | `comparison_api_oracle_driver.c` | Doesn't call the C helper algorithms or import the three component-oracle implementations |

**Critical nuance:** the two Python rounding references use the same basic
mathematical characterization of nearest-even (integer quotient and remainder).
They are independent of the *C implementation*, but **not independent
mathematical algorithms from each other**. Likewise GCC and Clang repeat the
same seeded cases: repetitions provide cross-toolchain evidence, not distinct
mathematical samples. These limits must remain explicit in final A13 wording.

The C drivers are test-only *transport*, not independent reference sources.
The exact metric driver prints status, a complete unsigned numerator as hex,
numerator exponent, unsigned denominator as hex, and denominator exponent.
Rounding and complete-API drivers use `%a`/`%La` for hexadecimal native
output, preserving the x87 `long double` range without a Python `float`
conversion. The float promotion to double for C `printf("%a")` is exact under
the qualified binary families. No physical BBTC production source is linked.

## 2. Found oracle blind spot and correction

**Previous behavior:** `test_exact_metric_oracle.py` treated any reply with
`denominator == 0` as `(status, None)`. In the expected OUTSIDE_DOMAIN case,
it checked only the operation status and the missing rational. As a result,
`4 1 0 0 0` could pass despite the C contract requiring **every metric field
to be cleared on failure**. The prior oracle also accepted, for successful
results, mathematically equal but contractually wrong denominator encodings
(e.g. `2/2` instead of the absolute-mode denominator `1*2^0`).

**Corrected verification:** each five-field record now receives *three*
separate forms of scrutiny, with any discrepancy fatal:

1. **Protocol validity:** exactly five fields, unsigned hexadecimal numerator
   and denominator, signed decimal exponents/status, no extra tokens or malformed
   numbers. The driver return code and total record count remain checked.
2. **Representation contract:** every exposed field is zero on operation
   failure; successful outputs have a nonzero denominator; an exact zero
   numerator has zero numerator exponent; absolute mode requires `(D,F)=(1,0)`;
   relative mode requires the unsigned **reference's actual p-bit normalized
   significand and exponent**, not merely an equivalent quotient.
3. **Mathematical value:** the reconstructed rational is compared against a
   separately computed Python `Fraction` from the two exact native inputs,
   including negative-reference magnitude and zero-reference domain handling.

The expected reference denominator is computed **independently** with Python
`int.bit_length` and shifts. For an integer significand `s>0` with bit length
`b<=p`, the normalized unsigned reference is

```
    m = s * 2^(p-b)
    e = input_exponent - (p-b)
    |reference| = m * 2^e
```

This is exact for both normal and subnormal generated operands. It does not
use the implementation's `frexp*`/`scalbn*` calls; no binary64 narrowing occurs.
The positive denominator retains exactly this native-family representation.

**What remains unobservable in this protocol:** C structure padding, the
explicit `used` count, and nonzero words beyond the reported `used` prefix.
The native cmocka unit tests separately check those field invariants; the
wire protocol alone does **not** prove they are correct. A hypothetical failure
that corrupts hidden unused limbs but leaves the displayed ratio unchanged is
not detectable by the five-field Python oracle.

## 3. Oracle mutation/self-test evidence

`tests/validation/test_oracle_integrity.py` tests the *pure Python validator*
without compiling or invoking BBTC. Every deliberate corruption must raise
`AssertionError`, including:

- OUTSIDE_DOMAIN with stale numerator, numerator exponent, or denominator
  exponent although the wire denominator is zero;
- Unexpected SUCCESS instead of OUTSIDE_DOMAIN;
- SUCCESS with a zero denominator;
- Successful absolute results encoded with a mathematically equivalent but
  noncanonical denominator;
- Relative results that evaluate correctly but carry an incorrectly normalized
  reference denominator;
- Zero numerator with a nonzero numerator exponent;
- Malformed hexadecimal, negative unsigned fields, missing/extra wire fields;
- Correctly shaped outputs carrying the wrong exact rational.

Valid absolute, non-dyadic relative and zero/failure records are retained as
positive controls, so an implementation that rejects *all* records cannot
pass the self-test. The test file is intentionally Python-standard-library
only, suitable for early execution before a full CMake build. The capture
script runs it with `PYTHONDONTWRITEBYTECODE=1` and fails immediately if any
case fails. This is an extra **preflight test**, not a tenth CTest entry: the
existing nine-suite registration and numerical corpus sizes are unchanged.

For independent reproduction:

```bash
cd ~/Projects/BBTC_VVUQ_B1b || exit 1
PYTHONDONTWRITEBYTECODE=1 python3 tests/validation/test_oracle_integrity.py -v
```

## 4. Corpus, execution and evidence integrity (A14)

The documented seeded capture campaign requests the same CTest structure:

- Dyadic: `3 * (100,000 + 18) = 300,054` oracle cases.
- Exact metric: `3 * 2 * (10,000 + 12) = 60,072` oracle cases.
- Exact rounding: `3 * (10,000 + 18) = 30,054` oracle cases.
- Complete API: previously observed `31,530` cases at 10,000 random per
  family, including all mandatory directed/fenv combinations.

The established total remains **421,710 oracle comparisons per compiler
configuration**, or 1,265,130 executions across the three configured builds.
The additional 15 standalone Python self-tests are tracked separately; they
are not added to the numerical-comparison counts.

CTest is configured with `BBTC_VVUQ_ORACLE_TESTS=ON`, four distinct oracle
registrations and the established deterministic seeds. The acceptance script
fails if it cannot find the exact nine CTest entries, if any test exits
nonzero, if the API symbols disappear, or if the production library includes
unexpected VVUQ targets/symbols. It now also logs SHA-256 for **all four**
Python oracle programs, all four transport drivers, the self-test program,
the CMake test registration, and the capture script itself, as well as the
existing five numerical C implementation files.

The source hashes identify the actual *on-disk* bytes tested. A transcript
captured while files are staged or untracked does **not** alone establish the
same evidence as a clean, committed source tree. In the **final acceptance**
checkpoint rerun this complete campaign after committing, record `git rev-parse
HEAD` and a clean `git status --porcelain`, and preserve the full transcript
out of tree. Compare the hashes against the committed files; never substitute
an unverified commit ID or declare mathematical acceptance from a green CTest
summary alone.

## 5. Remaining limitations and disposition

**A13:** the independent mathematical reference construction is supported by
source review, and the identified exact-metric protocol-validation gap is
addressed. This finding becomes qualified evidence only if the updated Python
self-tests and all four live-driver oracles pass on the primary host.

**A14:** evidence identification and replay are strengthened, but final closure
requires the final clean-commit acceptance capture. The present patch is not
an acceptance certificate; acceptance matrix items A01–A14 must each receive
explicit source/contract/test evidence and a supported disposition.

Other limitations remain unchanged: qualified binary32/binary64/x87 host and
FP environment only, no unmasked-trap portability claim, no experimental
ballistics validation, no guarantees about compiler implementations outside
the recorded toolchain, and no proof that sampled test cases exhaust all
possible numerical inputs.
