#!/usr/bin/env python3
"""VV0.1-B1b.3d: independent end-to-end rational oracle for comparison.h.

The tested C code is an *opaque black box*: Python submits native scalar
inputs through a tiny transport executable and independently predicts the
complete operation-status, exact finding, diagnostic classification, and
correctly rounded native metric. It does NOT link to, call, or import the C
exact-dyadic, metric, rounding, or preparation algorithms, nor the previous
component-oracle implementations.

Reference arithmetic is arbitrary-precision fractions.Fraction and integer
quotient/remainder, never binary64 Python float arithmetic. In particular,
x87 long-double values with exponents far outside binary64 are preserved as
rational numbers throughout. Both the input and output protocols use C hex
floating syntax rather than lossy decimal conversion.

This differential test is repeatable, with the seed and case counts printed.
Finite randomized testing is not a mathematical proof, nor does it establish
physical agreement of any BBTC ballistic model with experimental evidence.
"""
from __future__ import annotations

import argparse
from itertools import chain
from dataclasses import dataclass
from fractions import Fraction
import random
import re
import subprocess


@dataclass(frozen=True)
class Family:
    """A certified native binary-family descriptor (not an arbitrary ABI)."""

    key: str
    precision: int
    min_exp: int  # C's FLT_MIN_EXP/DBL_MIN_EXP/LDBL_MIN_EXP
    max_exp: int  # C's FLT_MAX_EXP/DBL_MAX_EXP/LDBL_MAX_EXP

    @property
    def min_quantum(self) -> int:
        return self.min_exp - self.precision

    @property
    def max_finite(self) -> Fraction:
        return Fraction((1 << self.precision) - 1) * power_of_two(
            self.max_exp - self.precision
        )


FAMILIES = (
    Family('F', 24, -125, 128),
    Family('D', 53, -1021, 1024),
    Family('L', 64, -16381, 16384),
)

HEX_NATIVE = re.compile(
    r'(?P<sign>[+-]?)0x(?P<int>[0-9a-f]+)'
    r'(?:\.(?P<frac>[0-9a-f]*))?p(?P<exp>[+-]?\d+)\Z'
)


@dataclass(frozen=True)
class Case:
    """Exactly one opaque C API invocation; no expected result is stored."""

    family: Family
    method: int
    null_policy: bool
    null_result: bool
    rounding: int
    flags: int
    reference: str
    observation: str
    tolerance: str

    def record(self) -> str:
        return (f'{self.family.key} {self.method} {int(self.null_policy)} '
                f'{int(self.null_result)} {self.rounding} {self.flags} '
                f'{self.reference} {self.observation} {self.tolerance}\n')


def power_of_two(exponent: int) -> Fraction:
    """Construct an exact power of two, including negative exponents."""
    if exponent >= 0:
        return Fraction(1 << exponent)
    return Fraction(1, 1 << -exponent)


def native_hex(significand: int, exponent: int, negative: bool = False) -> str:
    """Encode an exactly representable native-family scalar without rounding.

    The caller supplies a native-valid significand/exponent pair. Hex notation
    may have more than one digit left of the radix point: strtof/strtod/strtold
    interpret it as an exact binary integer times a power of two.
    """
    return f'{"-" if negative else ""}0x{significand:x}p{exponent:+d}'


def decode_native(token: str) -> Fraction | str:
    """Decode hexadecimal scalar tokens exactly; classify NaN/Inf lexically.

    A nonfinite token returns a discriminant rather than being inserted into
    Fraction: the contract demands NaN-first, then infinity, before any finite
    comparison, tolerance sign inspection, or relative-zero domain check.
    """
    lower = token.lower()
    if lower in ('nan', '+nan', '-nan'):
        return 'nan'
    if lower in ('inf', '+inf', '-inf', 'infinity', '+infinity', '-infinity'):
        return 'inf'
    match = HEX_NATIVE.fullmatch(lower)
    if match is None:
        raise ValueError(f'not a native hexadecimal scalar: {token!r}')
    frac_digits = match['frac'] or ''
    integral = int(match['int'] + frac_digits, 16)
    exponent = int(match['exp']) - 4 * len(frac_digits)
    result = Fraction(integral) * power_of_two(exponent)
    return -result if match['sign'] == '-' else result


def round_binary_nearest_even(value: Fraction, family: Family) -> tuple[int, Fraction]:
    """Independently map a nonnegative exact metric to B1b.3a metric semantics.

    This formulation uses Python's single arbitrary-precision quotient and
    remainder on a dyadic-scaled Fraction; it does not mimic the C bit-grid
    comparator or reuse the earlier rounding oracle's code.

    Classification observes the mathematical magnitude BEFORE rounding:
    1 exact finite, 2 inexact finite, 3 exceeds max finite, 4 positive rounds
    to zero. An unresolvable result (state 5) is not expected for admitted
    finite inputs on our qualified platform.
    """
    assert value >= 0
    if value == 0:
        return 1, Fraction(0)
    if value > family.max_finite:
        return 3, Fraction(0)

    # Identify floor(log2(value)) using exact integer bit lengths, correcting
    # a possibly one-bit-high initial estimate with a rational comparison.
    binade = value.numerator.bit_length() - value.denominator.bit_length()
    if value < power_of_two(binade):
        binade -= 1

    quantum_exp = max(family.min_quantum, binade - family.precision + 1)
    units = value / power_of_two(quantum_exp)
    quotient, remainder = divmod(units.numerator, units.denominator)
    comparison = 2 * remainder - units.denominator
    if comparison > 0 or (comparison == 0 and (quotient & 1) != 0):
        quotient += 1

    if quotient == 0:
        return 4, Fraction(0)
    rounded_value = Fraction(quotient) * power_of_two(quantum_exp)
    return (1 if remainder == 0 else 2), rounded_value


def expected(case: Case) -> tuple[int, int, int, Fraction]:
    """Pure mathematical B1 policy interpreter, independent from the C API.

    The validation order directly follows the accepted B1a written contract.
    0=SUCCESS, 1=INVALID_ARGUMENT, 2=NAN_INPUT, 3=NONFINITE_INPUT,
    4=OUTSIDE_DOMAIN; supported inputs are not expected to fail numerically.
    Findings: 0=NOT_EVALUATED, 1=PASS, 2=FAIL.
    """
    cleared = (0, 0, 0, Fraction(0))
    if case.null_result or case.null_policy or case.method not in (1, 2, 3):
        return (1, *cleared[1:])

    r = decode_native(case.reference)
    o = decode_native(case.observation)
    t = decode_native(case.tolerance)
    if 'nan' in (r, o, t):
        return (2, *cleared[1:])
    if 'inf' in (r, o, t):
        return (3, *cleared[1:])
    assert isinstance(r, Fraction)
    assert isinstance(o, Fraction)
    assert isinstance(t, Fraction)
    if (case.method == 1 and t != 0) or (case.method != 1 and t < 0):
        return (1, *cleared[1:])
    if case.method == 3 and r == 0:
        return (4, *cleared[1:])

    error = abs(o - r)
    if case.method == 1:
        return (0, 1 if error == 0 else 2, 0, Fraction(0))
    if case.method == 2:
        is_pass = error <= t
        magnitude = error
    else:
        # Compare exact products: do NOT decide via a rounded relative metric.
        is_pass = error <= t * abs(r)
        magnitude = error / abs(r)
    state, rounded = round_binary_nearest_even(magnitude, case.family)
    return (0, 1 if is_pass else 2, state, rounded)


def directed_cases(family: Family):
    """Acceptance-critical boundary corpus, also varied across caller fenv.

    Adversarial cases include exact threshold disagreement with a native
    rounded metric, opposite-sign finite maxima, tiny normal/subnormal input
    differences, relative overflow, method/pointer precedence, NaN ahead of
    infinity, and the signed-zero and zero-reference domain rules.
    """
    p = family.precision
    max_finite = native_hex((1 << p) - 1, family.max_exp - p)
    least = native_hex(1, family.min_quantum)
    min_normal = native_hex(1 << (p - 1), family.min_quantum)
    large = native_hex(1, p)
    near_one = native_hex((1 << (p - 1)) + 1, -p + 1)
    values = [
        (1, '0x0p+0', '-0x0p+0', '-0x0p+0'),
        (1, '-0x0p+0', '0x0p+0', '0x0p+0'),
        (1, '0x1p+0', '0x2p+0', '0x0p+0'),
        (1, near_one, near_one, '0x0p+0'),
        (1, least, '0x0p+0', '0x0p+0'),
        (1, '-0x1p+0', '-0x1p+0', '0x0p+0'),
        (2, '0x2p+0', '0x3p+0', '0x1p+0'),
        (2, '0x2p+0', '0x3p+0', '0x0p+0'),
        (2, large, '-0x1p+0', large),    # Exact 2^p+1, rounded to 2^p
        (2, max_finite, '-' + max_finite, max_finite),
        (2, least, '0x0p+0', least),
        (2, min_normal, least, min_normal),
        (3, large, '-0x1p+0', '0x1p+0'), # Exact relative error > 1
        (3, '0x3p+0', '0x4p+0', '0x1p-1'),
        (3, '-0x3p+0', '-0x4p+0', '0x1p-1'),
        (3, least, max_finite, max_finite), # Relative overflow
        (3, '0x0p+0', '0x0p+0', '0x0p+0'),
        (3, '-0x0p+0', '0x1p+0', '0x1p+0'),
        (3, '-0x1p+0', '0x0p+0', '0x1p+0'),
        (2, 'nan', 'inf', '0x1p+0'),
        (2, 'inf', 'nan', '-0x1p+0'),
        (2, 'inf', '0x0p+0', 'nan'),
        (2, '0x0p+0', 'inf', '0x1p+0'),
        (2, '0x0p+0', '0x1p+0', '-0x1p+0'),
        (1, '0x1p+0', '0x1p+0', '0x1p+0'),
        (1, '0x1p+0', '0x1p+0', 'nan'),
        (2, '0x1p+0', '0x2p+0', 'inf'),
        (2, '0x1p+0', '0x2p+0', 'nan'),
        (3, 'nan', 'inf', '0x1p+0'),
        (0, 'nan', 'inf', 'nan'),
        (255, 'nan', 'inf', 'nan'),
    ]
    for index, (method, reference, observation, tolerance) in enumerate(values):
        # Exercise every supported caller rounding mode and exception-preset
        # category, including failure returns; keep the mandatory corpus small.
        for rounding in range(4):
            for flags in range(4):
                yield Case(family, method, False, False, rounding, flags,
                           reference, observation, tolerance)
        if index % 5 == 0:
            yield Case(family, method, True, False, 1, 2,
                       reference, observation, tolerance)
            yield Case(family, method, False, True, 2, 3,
                       reference, observation, tolerance)


def random_native(family: Family, rng: random.Random) -> str:
    """Draw an exactly representable value from the entire native exponent span."""
    p = family.precision
    qmin = family.min_quantum
    choices = [
        '0x0p+0', '-0x0p+0', '0x1p+0', '-0x1p+0',
        '0x2p+0', '-0x2p+0', '0x3p+0', '-0x3p+0',
        native_hex(1, qmin),
        native_hex(1, qmin, True),
        native_hex((1 << p) - 1, family.max_exp - p),
        native_hex((1 << p) - 1, family.max_exp - p, True),
        native_hex(1 << (p - 1), qmin),
        native_hex(1 << (p - 1), qmin, True),
    ]
    if rng.randrange(4) == 0:
        return rng.choice(choices)
    if rng.randrange(5) == 0:
        significand = rng.randint(1, (1 << (p - 1)) - 1)
        exponent = qmin
    else:
        significand = rng.randint(1 << (p - 1), (1 << p) - 1)
        exponent = rng.randint(qmin, family.max_exp - p)
    return native_hex(significand, exponent, rng.randrange(2) == 0)


def random_cases(family: Family, rng: random.Random, count: int):
    """Supplement directed boundary cases with seeded method/input mixtures."""
    for _ in range(count):
        method = rng.choices([1, 2, 3, 0, 255], [20, 35, 35, 5, 5])[0]
        reference = random_native(family, rng)
        observation = (reference if rng.randrange(8) == 0
                       else random_native(family, rng))
        tolerance = random_native(family, rng)
        if rng.randrange(3) == 0:
            tolerance = tolerance.lstrip('-')
        if rng.randrange(12) == 0:
            tolerance = '-0x0p+0'
        if rng.randrange(80) == 0:
            reference = rng.choice(['nan', 'inf', '-inf'])
        if rng.randrange(80) == 0:
            observation = rng.choice(['nan', 'inf', '-inf'])
        if rng.randrange(80) == 0:
            tolerance = rng.choice(['nan', 'inf', '-inf'])
        yield Case(family, method, rng.randrange(40) == 0,
                   rng.randrange(40) == 0, rng.randrange(4), rng.randrange(4),
                   reference, observation, tolerance)


def verify_batch(driver: str, cases: list[Case], start_index: int) -> None:
    """Invoke C as a black box, compare every protocol field against Python."""
    if not cases:
        return
    payload = ''.join(case.record() for case in cases)
    process = subprocess.run([driver], input=payload, text=True,
                             capture_output=True, check=False)
    if process.returncode != 0:
        raise AssertionError(
            f'C driver returned {process.returncode}: {process.stderr}\n'
            f'first_case={cases[0]}')
    replies = process.stdout.splitlines()
    if len(replies) != len(cases):
        raise AssertionError(
            f'reply count={len(replies)}, expected={len(cases)}, '
            f'stderr={process.stderr!r}')
    for offset, (case, reply) in enumerate(zip(cases, replies, strict=True)):
        fields = reply.split()
        if len(fields) != 5:
            raise AssertionError(f'bad C response {reply!r} for case={case}')
        status, finding, metric_state, metric_token, env_preserved = fields
        result = (int(status), int(finding), int(metric_state),
                  decode_native(metric_token))
        # Every contractual zero (including failure and no-metric sentinels)
        # must be POSITIVE zero; Fraction alone intentionally ignores sign.
        if result[3] == 0 and metric_token.startswith('-'):
            raise AssertionError(
                f'case={start_index + offset} negative-zero diagnostic: {reply!r}')
        expected_result = expected(case)
        if result != expected_result or env_preserved != '1':
            raise AssertionError(
                f'case={start_index + offset} family={case.family.key} '
                f'method={case.method} null_policy={case.null_policy} '
                f'null_result={case.null_result} '
                f'rounding={case.rounding} flags={case.flags} '
                f'r={case.reference} o={case.observation} t={case.tolerance}\n'
                f'expected={expected_result}; actual={result}; '
                f'env_preserved={env_preserved}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', required=True)
    parser.add_argument('--per-family', type=int, default=2000)
    parser.add_argument('--seed', type=lambda x: int(x, 0), default=0xB1B3D)
    parser.add_argument('--batch-size', type=int, default=1000)
    args = parser.parse_args()
    if args.per_family < 0 or args.batch_size <= 0:
        parser.error('per-family must be >=0 and batch-size must be positive')
    rng = random.Random(args.seed)
    total = 0
    for family in FAMILIES:
        buffer: list[Case] = []
        family_count = 0
        for case in chain(directed_cases(family), random_cases(family, rng, args.per_family)):
            buffer.append(case)
            if len(buffer) >= args.batch_size:
                verify_batch(args.driver, buffer, total)
                total += len(buffer)
                family_count += len(buffer)
                buffer.clear()
        if buffer:
            verify_batch(args.driver, buffer, total)
            total += len(buffer)
            family_count += len(buffer)
        print(f'family={family.key} p={family.precision} cases={family_count}')
    print(f'PASS: {total} complete comparison API rational-oracle cases '
          f'seed={args.seed:#x} random_per_family={args.per_family}')


if __name__ == '__main__':
    main()
