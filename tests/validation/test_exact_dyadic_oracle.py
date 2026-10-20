#!/usr/bin/env python3
"""Independent, reproducible, exact-rational differential oracle for BBTC VVUQ.

WHY THIS EXISTS
---------------
The C23 kernel decides whether exact mathematical values of *represented*
float, double, and x87 extended long double operands satisfy an exact,
absolute, or reference-relative criterion. The oracle must not duplicate
its signed-bignum/limb algorithm or apply rounded host floating arithmetic.

Instead, this script independently manufactures exact IEEE/x87 dyadic values
from integer significands and exponents; `fractions.Fraction` evaluates the
entire mathematical comparison. The standalone C driver parses each number
from an exact hexadecimal C floating token and invokes the kernel. Results
are compared as (status, pass), so a false success, false failure, or silent
unsupported-environment return all fail the oracle.

NOT A FORMAL PROOF
------------------
Randomized differential testing establishes evidence only for sampled cases.
It neither proves correctness for all represented inputs nor certifies any
ballistic model or physical safety claim. Fixed seeds support replay. The
subprocess never links against production BBTC code.

REQUIREMENTS
------------
Python 3.10+ standard library only. On the C side: the initial little-endian or
big-endian binary32 / binary64 / x87-extended families as gated by the C
kernel; the driver itself explicitly requires 64-bit significand long double.
The input protocol carries *exact hexadecimal dyadics*; no lossy decimal
round-trip or Python binary64 narrowing occurs.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
from itertools import islice
from pathlib import Path
import random
import subprocess
import sys
from typing import Iterator


# Private prototype statuses match exact_dyadic.h; they are deliberately
# separate from the accepted future bbtc_vv_operation_status_e API.
SUCCESS = 0
INVALID_ARGUMENT = 1
OUTSIDE_DOMAIN = 4


@dataclass(frozen=True)
class Family:
    """Binary format limits, independent of C's frexp-based decomposition.

    `p` includes the leading significand bit. For subnormals, the smallest
    quantum is 2 ** (1 - bias - (p-1)). `emax` is the greatest unbiased
    normal exponent. Encoding is informational: the transport uses hex values.
    """

    key: str
    p: int
    bias: int
    emax: int

    @property
    def min_quantum_exponent(self) -> int:
        return 1 - self.bias - (self.p - 1)

    @property
    def max_coefficient(self) -> int:
        return (1 << self.p) - 1


FAMILIES = (
    Family("F", 24, 127, 127),
    Family("D", 53, 1023, 1023),
    Family("L", 64, 16383, 16383),
)


@dataclass(frozen=True)
class Dyadic:
    """Exact real number `(-1 if negative else 1) * significand * 2**exponent`.

    This is deliberately NOT the same decomposition used by the C kernel:
    test inputs originate in format-specific integer fields, without frexp,
    scalbn, uint64_t conversion, or the C limb accumulator.
    """

    significand: int
    exponent: int
    negative: bool = False

    def token(self) -> str:
        """Encode exactly, with no Python float conversion or rounding."""
        return ("-" if self.negative else "") + f"0x{self.significand:x}p{self.exponent:+d}"

    def fraction(self) -> Fraction:
        """Build an independent exact rational using Python big integers."""
        signed = -self.significand if self.negative else self.significand
        if self.exponent >= 0:
            return Fraction(signed << self.exponent, 1)
        return Fraction(signed, 1 << -self.exponent)


@dataclass(frozen=True)
class Case:
    """One deterministic test vector and its independently derived result."""

    family: Family
    method: int
    reference: Dyadic
    observation: Dyadic
    tolerance: Dyadic
    label: str

    def request(self) -> str:
        return (
            f"{self.family.key} {self.method} {self.reference.token()} "
            f"{self.observation.token()} {self.tolerance.token()}\n"
        )

    def expected(self) -> tuple[int, int]:
        """Apply B1a mathematical semantics using Fraction, not the C kernel.

        Every test case is a finite, *exactly representable* binary value.
        Structural invalidity (negative/extraneous tolerance) takes
        precedence over the relative-reference zero-domain check.
        """
        r = self.reference.fraction()
        o = self.observation.fraction()
        t = self.tolerance.fraction()

        if self.method == 1:
            if t != 0:
                return INVALID_ARGUMENT, 0
            return SUCCESS, int(r == o)
        if self.method not in (2, 3) or t < 0:
            return INVALID_ARGUMENT, 0
        if self.method == 3 and r == 0:
            return OUTSIDE_DOMAIN, 0

        exact_difference = abs(o - r)
        exact_threshold = t if self.method == 2 else t * abs(r)
        return SUCCESS, int(exact_difference <= exact_threshold)


def zero(negative: bool = False) -> Dyadic:
    """Signed zero. The sign is irrelevant to exact mathematical equality."""
    return Dyadic(0, 0, negative)


def integer(value: int) -> Dyadic:
    """An exactly representable small integer."""
    return Dyadic(abs(value), 0, value < 0)


def power_of_two(exponent: int, negative: bool = False) -> Dyadic:
    return Dyadic(1, exponent, negative)


def random_value(fmt: Family, rng: random.Random) -> Dyadic:
    """Sample normal, subnormal, zero, extreme and small-exponent dyadics.

    Each result is representable by the selected native floating-point type.
    For normal values, the significand is p bits with the implicit/explicit
    leading bit set. Subnormals have 1..p-1 significant bits and a fixed
    quantum exponent. Exponents are independent of the C implementation.
    """
    category = rng.randrange(10)
    negative = bool(rng.getrandbits(1))
    quantum = fmt.min_quantum_exponent
    if category == 0:
        return zero(negative)
    if category <= 2:
        # Nonzero subnormal in [2^quantum, (2^(p-1)-1)*2^quantum].
        return Dyadic(rng.randrange(1, 1 << (fmt.p - 1)), quantum, negative)
    if category == 3:
        return Dyadic(fmt.max_coefficient, fmt.emax - fmt.p + 1, negative)
    if category == 4:
        return Dyadic(1 << (fmt.p - 1), 1 - fmt.bias - (fmt.p - 1), negative)
    if category == 5:
        return power_of_two(rng.randrange(-8, 9), negative)
    if category == 6:
        return Dyadic(rng.randrange(1 << (fmt.p - 1), 1 << fmt.p),
                      rng.randrange(-10, 10) - fmt.p + 1, negative)
    return Dyadic(rng.randrange(1 << (fmt.p - 1), 1 << fmt.p),
                  rng.randrange(1 - fmt.bias, fmt.emax + 1) - fmt.p + 1,
                  negative)


def directed_cases(fmt: Family) -> Iterator[Case]:
    """Mathematically derived adversarial regressions and exact boundaries.

    These make rare boundary conditions mandatory even when PRNG sampling
    would otherwise almost never reach them.
    """
    one = integer(1)
    two = integer(2)
    minus_one = integer(-1)
    tiny = power_of_two(fmt.min_quantum_exponent)
    big = power_of_two(fmt.p)
    max_finite = Dyadic(fmt.max_coefficient, fmt.emax - fmt.p + 1)

    yield Case(fmt, 1, zero(), zero(True), zero(), "signed-zero-exact")
    yield Case(fmt, 1, one, one, zero(), "equal-exact")
    yield Case(fmt, 1, one, two, zero(), "unequal-exact")
    yield Case(fmt, 1, one, two, one, "exact-invalid-tolerance")
    yield Case(fmt, 2, big, minus_one, big, "rounded-subtraction-false-pass")
    yield Case(fmt, 3, big, minus_one, one, "rounded-relative-false-pass")
    yield Case(fmt, 2, one, two, one, "absolute-exact-equality-threshold")
    yield Case(fmt, 3, two, one, Dyadic(1, -1), "relative-equality-threshold")
    yield Case(fmt, 3, one, two, Dyadic(1, -1), "relative-asymmetry-fail")
    yield Case(fmt, 3, zero(), zero(), zero(), "relative-zero-reference")
    yield Case(fmt, 2, zero(), tiny, tiny, "smallest-subnormal-inclusion")
    yield Case(fmt, 2, zero(), tiny, zero(), "smallest-subnormal-exclusion")
    yield Case(fmt, 3, max_finite, Dyadic(max_finite.significand,
                                        max_finite.exponent, True),
               two, "max-relative-product-equality")
    yield Case(fmt, 2, max_finite, Dyadic(max_finite.significand,
                                        max_finite.exponent, True),
               max_finite, "exceeds-finite-difference")
    yield Case(fmt, 3, tiny, zero(), one, "subnormal-relative-equality")
    yield Case(fmt, 3, tiny, zero(), Dyadic(1, -1), "subnormal-relative-fail")
    yield Case(fmt, 2, one, one, Dyadic(1, -1, True), "negative-tolerance")
    yield Case(fmt, 3, zero(), one, Dyadic(1, -1, True), "invalid-before-domain")


def random_cases(fmt: Family, rng: random.Random, count: int) -> Iterator[Case]:
    """Generate full-exponent, correlated, and near-threshold test families."""
    for i in range(count):
        method = 1 + (i % 3)
        reference = random_value(fmt, rng)
        observation = random_value(fmt, rng)
        tolerance = random_value(fmt, rng)
        category = i % 12
        if category == 0:
            observation = reference
        elif category == 1:
            tolerance = zero()
        elif category == 2:
            tolerance = integer(1)
        elif category == 3:
            reference = zero()
        elif category == 4:
            reference = power_of_two(fmt.p)
            observation = integer(-1)
            tolerance = reference if method == 2 else integer(1)
        elif category == 5:
            observation = zero()
            tolerance = Dyadic(reference.significand, reference.exponent)
        elif category == 6:
            tolerance = Dyadic(tolerance.significand, tolerance.exponent)
        elif category == 7:
            observation = Dyadic(reference.significand, reference.exponent,
                                 not reference.negative)
            tolerance = integer(2)
        elif category == 8:
            reference = power_of_two(fmt.min_quantum_exponent)
            observation = zero()
            tolerance = reference
        elif category == 9:
            reference = integer(1)
            observation = integer(2)
            tolerance = integer(1)
        # Leave category 10/11 fully unconstrained except for method-specific
        # well-formedness, so statistical coverage spans unusual input ratios.
        if method == 1 and i % 7 != 0:
            tolerance = zero()
        if method != 1 and i % 9 != 0:
            tolerance = Dyadic(tolerance.significand, tolerance.exponent)
        yield Case(fmt, method, reference, observation, tolerance,
                   f"random-{i}-category-{category}")


def execute_driver(driver: Path, cases: list[Case], timeout: int) -> None:
    """Run a whole deterministic corpus, matching every response in order.

    A subprocess failure, too-few or too-many output lines, a wrong status,
    or an incorrect Boolean is fatal. Print a complete replay vector before
    exit so a failing random case can become a permanent deterministic test.
    """
    payload = "".join(case.request() for case in cases)
    result = subprocess.run([str(driver.resolve())], input=payload, text=True,
                            capture_output=True, timeout=timeout, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"driver exited {result.returncode}: {result.stderr}")
    outputs = result.stdout.splitlines()
    if len(outputs) != len(cases):
        raise RuntimeError(f"expected {len(cases)} replies, got {len(outputs)}; "
                           f"driver stderr: {result.stderr}")
    for i, (case, actual_line) in enumerate(zip(cases, outputs, strict=True)):
        expected = case.expected()
        try:
            fields = actual_line.split()
            if len(fields) != 2:
                raise ValueError("expected two integers")
            actual = (int(fields[0]), int(fields[1]))
        except ValueError as exc:
            raise RuntimeError(f"malformed driver reply {i}: {actual_line!r}") from exc
        if actual != expected:
            raise AssertionError(
                f"ORACLE MISMATCH at index={i} kind={case.label}\n"
                f"request: {case.request().rstrip()}\n"
                f"expected: {expected}, actual: {actual}\n"
                "This is a deterministic regression vector: retain it."
            )


def main() -> int:
    """CLI entry point; CLI options are recorded in stdout for reproducibility."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--driver", required=True, type=Path,
                        help="compiled dyadic_oracle_driver executable")
    parser.add_argument("--per-family", type=int, default=2000,
                        help="random cases per family (directed cases added)")
    parser.add_argument("--seed", type=lambda s: int(s, 0), default=0xBB7C2026,
                        help="Python PRNG seed (supports hexadecimal with 0x)")
    parser.add_argument("--batch-size", type=int, default=5000,
                        help="stream batches to bound Python memory consumption")
    parser.add_argument("--timeout", type=int, default=180,
                        help="driver runtime timeout in seconds")
    args = parser.parse_args()
    if args.per_family < 0 or args.timeout <= 0 or args.batch_size <= 0:
        parser.error("case count must be nonnegative; batch size and timeout positive")
    if not args.driver.is_file():
        parser.error(f"driver not found: {args.driver}")

    rng = random.Random(args.seed)
    overall = 0
    print(f"seed={args.seed:#x} random_per_family={args.per_family} "
          f"batch_size={args.batch_size} python={sys.version.split()[0]}",
          flush=True)

    # The count may be hundreds of thousands on a developer workstation.
    # Streaming deterministic batches prevents test memory consumption from
    # scaling with the total requested campaign size. `islice` advances the
    # same seeded generator without altering its order or replay semantics.
    for fmt in FAMILIES:
        directed = list(directed_cases(fmt))
        total_family = len(directed) + args.per_family
        print(f"family={fmt.key} p={fmt.p} cases={total_family}", flush=True)
        execute_driver(args.driver, directed, args.timeout)
        generated = random_cases(fmt, rng, args.per_family)
        remaining = args.per_family
        while remaining:
            chunk = list(islice(generated, min(remaining, args.batch_size)))
            if not chunk:
                raise RuntimeError("random case iterator ended prematurely")
            execute_driver(args.driver, chunk, args.timeout)
            remaining -= len(chunk)
        overall += total_family

    print(f"PASS: {overall} exact-rational differential comparisons", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (AssertionError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
