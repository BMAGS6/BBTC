#!/usr/bin/env python3
"""Independent, exact-rational VV0.1-B2b M-01..M-04 reference trace.

This developer-only program uses Python integers/Fraction; it does not import,
execute, parse output from, or reproduce values supplied by a BBTC binary.
The mathematical reference is evaluated from an elementary Chebyshev polynomial
identity, while an independently written step trace models the relevant
source-level arithmetic. These agree only if each asserted identity holds.

Passing this script is not qualification of any compiled C evaluation profile.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction


@dataclass(frozen=True)
class Case:
    identity: str
    temperature: Fraction
    coefficients: tuple[Fraction, ...]
    expected: tuple[Fraction, Fraction, Fraction]


T_MIN = Fraction(256)
T_MAX = Fraction(512)
ZERO = Fraction(0)
TWO = Fraction(2)
FOUR = Fraction(4)


def power_of_two(exponent: int) -> Fraction:
    return Fraction(1 << exponent) if exponent >= 0 else Fraction(1, 1 << -exponent)


# Independent, fixed analytical expectations; no values are learned from BBTC.
CASES = (
    Case("M-01", Fraction(384), (ZERO,), (ZERO, ZERO, ZERO)),
    Case("M-02", Fraction(384), (power_of_two(-10), power_of_two(-12)),
         (power_of_two(-10), power_of_two(-19), ZERO)),
    Case("M-03", Fraction(384), (ZERO, ZERO, power_of_two(-12)),
         (-power_of_two(-12), ZERO, power_of_two(-24))),
    Case("M-04", Fraction(320), (ZERO, ZERO, power_of_two(-12)),
         (-power_of_two(-13), -power_of_two(-18), power_of_two(-24))),
)


@dataclass(frozen=True)
class BinaryProfile:
    name: str
    significand_bits: int
    min_normal_exponent: int
    max_finite_exponent: int


# Candidate formats from the captured x86-64 profiles, not universal C claims.
PROFILES = (
    BinaryProfile("float/binary32", 24, -126, 127),
    BinaryProfile("double/binary64", 53, -1022, 1023),
    BinaryProfile("long_double/x87-extended", 64, -16382, 16383),
)


def closed_form(case: Case) -> tuple[Fraction, Fraction, Fraction]:
    """Compute independently from T0=1, T1=x, T2=2*x^2-1."""
    c = (*case.coefficients, *(ZERO for _ in range(3 - len(case.coefficients))))
    x = TWO * (case.temperature - T_MIN) / (T_MAX - T_MIN) - 1
    s = TWO / (T_MAX - T_MIN)
    return (
        c[0] + c[1] * x + c[2] * (TWO * x * x - 1),
        s * (c[1] + FOUR * c[2] * x),
        FOUR * c[2] * s * s,
    )


def trace_source_arithmetic(case: Case) -> tuple[tuple[Fraction, ...], list[tuple[str, Fraction]]]:
    """Trace scalar C expression operations without executing C or using floats.

    Record each result of a relevant individual arithmetic operation, including
    intermediate subtraction/multiplication/division and derivative scaling.
    The reference computation above uses a separate closed-form identity.
    """
    steps: list[tuple[str, Fraction]] = []

    def record(label: str, value: Fraction) -> Fraction:
        steps.append((label, value))
        return value

    record("input.Tmin", T_MIN)
    record("input.Tmax", T_MAX)
    record("input.T", case.temperature)
    for k, value in enumerate(case.coefficients):
        record(f"input.c{k}", value)

    span = record("normalization.span", T_MAX - T_MIN)
    delta = record("normalization.delta", case.temperature - T_MIN)
    ratio = record("normalization.delta/span", delta / span)
    two_ratio = record("normalization.two_ratio", TWO * ratio)
    x = record("normalization.x", two_ratio - 1)
    scale = record("normalization.scale", TWO / span)

    if not (-1 <= x <= 1):
        raise AssertionError(f"{case.identity}: unexpectedly active coordinate clamp")

    value_next = value_next_next = ZERO
    first_next = first_next_next = ZERO
    second_next = second_next_next = ZERO

    for k in range(len(case.coefficients) - 1, 0, -1):
        prefix = f"recurrence.k{k}"

        vx = record(f"{prefix}.value.2x", TWO * x)
        vt = record(f"{prefix}.value.product", vx * value_next)
        vs = record(f"{prefix}.value.sub", vt - value_next_next)
        value = record(f"{prefix}.value.add_c", vs + case.coefficients[k])

        fa = record(f"{prefix}.first.2_value", TWO * value_next)
        fx = record(f"{prefix}.first.2x", TWO * x)
        fb = record(f"{prefix}.first.product", fx * first_next)
        fs = record(f"{prefix}.first.sum", fa + fb)
        first = record(f"{prefix}.first.sub", fs - first_next_next)

        sa = record(f"{prefix}.second.4_first", FOUR * first_next)
        sx = record(f"{prefix}.second.2x", TWO * x)
        sb = record(f"{prefix}.second.product", sx * second_next)
        ss = record(f"{prefix}.second.sum", sa + sb)
        second = record(f"{prefix}.second.sub", ss - second_next_next)

        value_next_next, value_next = value_next, value
        first_next_next, first_next = first_next, first
        second_next_next, second_next = second_next, second

    b_product = record("reconstruction.B.product", x * value_next)
    b_sub = record("reconstruction.B.subtract", b_product - value_next_next)
    b = record("reconstruction.B", b_sub + case.coefficients[0])

    d1_product = record("reconstruction.first.product", x * first_next)
    d1_sum = record("reconstruction.first.add", value_next + d1_product)
    d1 = record("reconstruction.first", d1_sum - first_next_next)

    d2_double = record("reconstruction.second.2_first", TWO * first_next)
    d2_product = record("reconstruction.second.product", x * second_next)
    d2_sum = record("reconstruction.second.add", d2_double + d2_product)
    d2 = record("reconstruction.second", d2_sum - second_next_next)

    if d1 != ZERO:
        out_d1 = record("output.first.scale", d1 * scale)
    else:
        out_d1 = record("output.first.zero_branch", ZERO)

    if d2 != ZERO:
        d2_once = record("output.second.first_scale", d2 * scale)
        out_d2 = record("output.second.second_scale", d2_once * scale)
    else:
        out_d2 = record("output.second.zero_branch", ZERO)

    return (b, out_d1, out_d2), steps


def is_exact_normal(value: Fraction, profile: BinaryProfile) -> bool:
    """True for zero or an exactly representable *normal* finite binary value."""
    if value == ZERO:
        return True
    numerator = abs(value.numerator)
    denominator = value.denominator
    if denominator & (denominator - 1):
        return False  # Not dyadic.

    # Remove trailing zero bits so the odd integer is the significand.
    trailing_zero_bits = (numerator & -numerator).bit_length() - 1
    odd = numerator >> trailing_zero_bits
    exponent = trailing_zero_bits - (denominator.bit_length() - 1)
    top_exponent = exponent + odd.bit_length() - 1
    return (
        odd.bit_length() <= profile.significand_bits
        and profile.min_normal_exponent <= top_exponent <= profile.max_finite_exponent
    )


def verify_case(case: Case, *, expectation: tuple[Fraction, Fraction, Fraction] | None = None,
                verbose: bool = False) -> int:
    expected = case.expected if expectation is None else expectation
    independent = closed_form(case)
    trace_result, steps = trace_source_arithmetic(case)

    if independent != expected:
        raise AssertionError(f"{case.identity}: analytical reference disagrees with expected literals: "
                             f"{independent!r} != {expected!r}")
    if trace_result != expected:
        raise AssertionError(f"{case.identity}: source-operation trace disagrees with reference: "
                             f"{trace_result!r} != {expected!r}")

    for profile in PROFILES:
        for label, value in steps:
            if not is_exact_normal(value, profile):
                raise AssertionError(f"{case.identity}/{profile.name}: {label}="
                                     f"{value} is not exactly normal/representable")

    if verbose:
        print(f"{case.identity}: expected B,dB/dT,d²B/dT² = "
              + ", ".join(str(v) for v in expected))
        for label, value in steps:
            print(f"  {label:<42} {value}")

    return len(steps)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verbose", action="store_true", help="show exact intermediate fractions")
    args = parser.parse_args()

    step_count = sum(verify_case(case, verbose=args.verbose) for case in CASES)

    # In-memory negative control: the oracle must reject a corrupted expected
    # literal, not quietly rebuild its reference from the same wrong value.
    for case in CASES:
        corrupted = (case.expected[0] + power_of_two(-30), *case.expected[1:])
        try:
            verify_case(case, expectation=corrupted)
        except AssertionError:
            pass
        else:
            raise AssertionError(f"{case.identity}: corrupted-reference negative control survived")

    print("PASS: independent mathematical identities and source-operation traces agree")
    print(f"PASS: {len(CASES)} cases, {len(CASES) * len(PROFILES)} native-format assignments, "
          f"{len(CASES) * len(PROFILES) * 3} mathematical output identities")
    print(f"PASS: {step_count} recorded expression intermediates, checked in each native format")
    print(f"PASS: {len(CASES)} deliberately corrupted expected-literal controls rejected")
    print("SCOPE: Python rational proof only; compiler, ABI and live fenv not qualified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
