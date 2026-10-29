#!/usr/bin/env python3
"""Exact metric differential oracle, not a second copy of C big-integer math.

This harness chooses finite, exactly representable native binary inputs using
format-aware integers; the existing B1b.2 input generator supplies independent
field-oriented cases. The expected result is evaluated *only* with Python's
arbitrary-precision fractions.Fraction. The C output is interpreted as its
published exact rational representation; no Python float conversion occurs.

A passing random corpus is evidence of sampled correctness, not a proof or a
claim about physical trajectory accuracy. A fixed seed permits exact replay.
"""
from __future__ import annotations

import argparse
from fractions import Fraction
import random
import subprocess

from test_exact_dyadic_oracle import FAMILIES, Dyadic, integer, power_of_two, random_value, zero


def cases(fmt, rng, count):
    """Mandatory extremes, cancellation, signed zero, non-dyadic ratios."""
    quantum = fmt.min_quantum_exponent
    biggest = Dyadic(fmt.max_coefficient, fmt.emax - fmt.p + 1)
    tiny = power_of_two(quantum)
    directed = [
        (zero(), zero(True)),
        (integer(1), integer(3)),
        (integer(-3), integer(1)),
        (integer(3), integer(4)),
        (integer(3), integer(2)),
        (zero(), biggest),
        (biggest, Dyadic(biggest.significand, biggest.exponent, True)),
        (tiny, zero()),
        (biggest, tiny),
        (tiny, biggest),
        (power_of_two(fmt.p), integer(-1)),
        (integer(-3), integer(-2)),
    ]
    yield from directed
    for _ in range(count):
        yield random_value(fmt, rng), random_value(fmt, rng)


def fraction_from_message(tokens):
    """Decode an exact unsigned integer/power-of-two ratio without rounding."""
    if len(tokens) != 5:
        raise AssertionError(f"Malformed driver record: {tokens!r}")
    status, numerator_hex, exponent, denominator_hex, denominator_exponent = tokens
    n, d = int(numerator_hex, 16), int(denominator_hex, 16)
    e = int(exponent) - int(denominator_exponent)
    if d == 0:
        return int(status), None
    return int(status), Fraction(n << e, d) if e >= 0 else Fraction(n, d << -e)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', required=True)
    parser.add_argument('--per-family', type=int, default=2000)
    parser.add_argument('--seed', type=lambda s: int(s, 0), default=0xC023C1)
    args = parser.parse_args()
    if args.per_family < 0:
        parser.error('--per-family must be nonnegative')
    rng = random.Random(args.seed)

    vectors = []
    for fmt in FAMILIES:
        for r, o in cases(fmt, rng, args.per_family):
            for method in ('A', 'R'):
                expected = None
                status = 0
                if method == 'R' and r.significand == 0:
                    status = 4  # OUTSIDE_DOMAIN, intentionally zero output
                else:
                    expected = abs(o.fraction() - r.fraction())
                    if method == 'R':
                        expected /= abs(r.fraction())
                line = f'{fmt.key} {method} {r.token()} {o.token()}\n'
                vectors.append((fmt.key, method, r, o, status, expected, line))

    commands = ''.join(item[6] for item in vectors)
    result = subprocess.run([args.driver], input=commands, text=True,
                            capture_output=True, check=True)
    replies = result.stdout.splitlines()
    if len(replies) != len(vectors):
        raise AssertionError(f'Expected {len(vectors)} replies; got {len(replies)}; stderr={result.stderr}')
    for index, (vector, reply) in enumerate(zip(vectors, replies, strict=True)):
        family, mode, r, o, expected_status, expected_value, _ = vector
        status, value = fraction_from_message(reply.split())
        if status != expected_status or value != expected_value:
            raise AssertionError(
                f'case={index} family={family} mode={mode} '
                f'r={r.token()} o={o.token()} '
                f'expected={(expected_status, expected_value)} '
                f'got={(status, value)}'
            )
    print(f'PASS: {len(vectors)} exact metric comparisons '
          f'seed={args.seed:#x} random_per_family={args.per_family}')


if __name__ == '__main__':
    main()
