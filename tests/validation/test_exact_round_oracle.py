#!/usr/bin/env python3
"""Independent B1b.3c-2 binary-rounding oracle using arbitrary-precision Fraction.

Unlike the production C algorithm, this oracle does not search a floating
significand with repeated integer comparisons. It constructs the full rational
number using Python's arbitrary-precision integers, computes a single exact
quotient and remainder, and applies nearest-even using divmod. Expected native
values are represented as Fraction, including x87 long double: Python float is
NEVER used for a long-double reference or transport.

Results are deterministic with a printed seed. Randomized tests are augmented
with mandatory ties, subnormal, overflow, and binade-transition cases. A
successful finite sample is not a proof of correctness on all platforms.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import random
import re
import subprocess


@dataclass(frozen=True)
class Family:
    key: str
    p: int
    min_exp: int
    max_exp: int


FAMILIES = (
    Family('F', 24, -125, 128),
    Family('D', 53, -1021, 1024),
    Family('L', 64, -16381, 16384),
)


def pow2(exponent: int) -> Fraction:
    """Produce 2**exponent without any floating-point conversion."""
    return Fraction(1 << exponent, 1) if exponent >= 0 else Fraction(1, 1 << -exponent)


def decode_hex(text: str) -> Fraction:
    """Parse C %a/%La hexadecimal output exactly, including x87 mantissas."""
    match = re.fullmatch(r'([+-]?)0x([0-9a-f]+)(?:\.([0-9a-f]*))?p([+-]?\d+)', text.lower())
    if not match:
        raise AssertionError(f'malformed hexadecimal float {text!r}')
    sign, integral, fraction_digits, exp_text = match.groups()
    decimals = fraction_digits or ''
    number = int(integral + decimals, 16)
    if sign == '-':
        number = -number
    return number * pow2(int(exp_text) - 4 * len(decimals))


def from_fields(n: int, e: int, d: int, f: int) -> Fraction:
    if not d:
        raise ValueError('invalid denominator')
    return Fraction(n, d) * pow2(e - f)


def oracle(value: Fraction, fmt: Family):
    """Round by one exact quotient/remainder rather than C's bit-grid search."""
    if value < 0:
        raise ValueError('nonnegative metrics only')
    if not value:
        return 1, Fraction(0)

    maximum = ((1 << fmt.p) - 1) * pow2(fmt.max_exp - fmt.p)
    if value > maximum:
        return 3, Fraction(0)

    # Estimate binade by integer length and adjust using exact Fraction.
    exponent = value.numerator.bit_length() - value.denominator.bit_length()
    if value < pow2(exponent):
        exponent -= 1
    quantum = max(fmt.min_exp - fmt.p, exponent - fmt.p + 1)

    scaled = value / pow2(quantum)
    q, remainder = divmod(scaled.numerator, scaled.denominator)
    halfway = remainder * 2
    rounded = q + int(halfway > scaled.denominator or
                      (halfway == scaled.denominator and q % 2 == 1))
    if rounded == 0:
        return 4, Fraction(0)
    if remainder == 0:
        return 1, rounded * pow2(quantum)
    return 2, rounded * pow2(quantum)


def vectors(fmt: Family, rng: random.Random, count: int):
    """Cover mathematical boundaries explicitly, then explore broad ratios."""
    p = fmt.p
    q = fmt.min_exp - p
    mx = (1 << p) - 1
    samples = [
        (0, 0, 1, 0),
        (1, 0, 1, 0),
        (1, 0, 3, 0),
        (2, 0, 3, 0),
        ((1 << p) + 1, -p, 1, 0),          # tie: even lower at 1
        ((1 << p) + 3, -p, 1, 0),          # tie: odd lower, round upward
        ((1 << (p + 1)) - 1, -p, 1, 0),    # midpoint at 2.0 binade carry
        (1, q - 1, 1, 0),                 # midpoint to zero: ties even
        (3, q - 1, 1, 0),                 # midpoint 1/2 subnormal quantum
        ((1 << p) - 1, q - 1, 1, 0),     # last subnormal/first normal tie
        (mx, fmt.max_exp - p, 1, 0),     # exact largest finite
        (2 * mx + 1, fmt.max_exp - p - 1, 1, 0),  # just above finite max
        (1, fmt.max_exp, 1, 0),          # mathematical overflow
        (1, q - 2, 1, 0),                # strictly less than halfway zero
        (3, q - 2, 1, 0),                # positive rounds to quantum
        (1, q, 1, 0),                    # exact least subnormal
        (1, 0, 3, 0),                   # non-dyadic rational
        (UINT64, -64, UINT64 - 2, -64), # non-dyadic near 1 (wide denominator)
    ]
    yield from samples
    bit_counts = [0, 1, 2, 8, p - 1, p, p + 1, 65, 80, 128, 256,
                  1024, 4096, 15000, 32800]
    exp_choices = [-16508, -16445, -1022, -149, -1, 0, 1, 127,
                   1023, 16320]
    for _ in range(count):
        bits = rng.choice(bit_counts)
        n = rng.getrandbits(bits)
        d = rng.getrandbits(rng.randint(1, 64)) or 1
        e = rng.choice(exp_choices) if rng.randrange(3) == 0 else rng.randint(-16508, 16320)
        f = rng.choice(exp_choices) if rng.randrange(3) == 0 else rng.randint(-16508, 16320)
        yield n, e, d, f


UINT64 = (1 << 64) - 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', required=True)
    parser.add_argument('--per-family', type=int, default=1000)
    parser.add_argument('--seed', type=lambda s: int(s, 0), default=0xC023C2)
    args = parser.parse_args()
    if args.per_family < 0:
        parser.error('--per-family must be nonnegative')
    rng = random.Random(args.seed)
    corpus = []
    for fmt in FAMILIES:
        for n, e, d, f in vectors(fmt, rng, args.per_family):
            expected = oracle(from_fields(n, e, d, f), fmt)
            corpus.append((fmt, n, e, d, f, expected))

    commands = ''.join(f'{fmt.key} {n:x} {e} {d:x} {f}\n'
                       for fmt, n, e, d, f, _ in corpus)
    result = subprocess.run([args.driver], input=commands, text=True,
                            capture_output=True, check=True)
    replies = result.stdout.splitlines()
    if len(replies) != len(corpus):
        raise AssertionError(f'wrong reply count: {len(replies)} expected {len(corpus)} '
                             f'stderr={result.stderr}')

    for idx, (case, reply) in enumerate(zip(corpus, replies, strict=True)):
        fmt, n, e, d, f, expected = case
        parts = reply.split()
        if len(parts) != 3:
            raise AssertionError(f'case={idx} malformed reply {reply!r}')
        status, state, token = parts
        actual = (int(state), decode_hex(token))
        if int(status) != 0 or actual != expected:
            raise AssertionError(
                f'case={idx} fmt={fmt.key} N={n:x} E={e} d={d:x} F={f} '
                f'status={status} expected={expected} actual={actual}'
            )
    print(f'PASS: {len(corpus)} exact-rational native-rounding comparisons '
          f'seed={args.seed:#x} random_per_family={args.per_family}')


if __name__ == '__main__':
    main()
