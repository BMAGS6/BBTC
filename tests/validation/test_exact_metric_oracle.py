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
import re
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


_HEX_UNSIGNED = re.compile(r'[0-9a-fA-F]+\Z')
_DECIMAL_SIGNED = re.compile(r'[+-]?[0-9]+\Z')


def verify_metric_record(tokens, fmt, method, reference, expected_status,
                         expected_value):
    """Validate an entire driver record, not merely its rational quotient.

    The output protocol is STATUS N_HEX E D_HEX F.  A quotient is insufficient
    to prove the representation contract: N=2,D=2 and N=1,D=1 denote the
    same value, but the metric builder MUST use D=1,F=0 in absolute mode and
    the reference's exactly normalized significand/exponent in relative mode.

    On failure, every field of the C metric structure is required to be zero.
    The transport cannot expose padding bytes, the `used` count or zero-valued
    unused limbs; those are checked by the native cmocka representation tests.

    Raises AssertionError on invalid syntax, an invalid representation, a
    mismatched status or a numerically incorrect result.  This routine does
    not call any C kernel and uses only Python arbitrary-precision arithmetic.
    """
    if len(tokens) != 5:
        raise AssertionError(f'expected five metric fields, received {tokens!r}')
    status_text, numerator_text, exponent_text, denominator_text, den_exp_text = tokens
    if not (_DECIMAL_SIGNED.fullmatch(status_text) and
            _HEX_UNSIGNED.fullmatch(numerator_text) and
            _DECIMAL_SIGNED.fullmatch(exponent_text) and
            _HEX_UNSIGNED.fullmatch(denominator_text) and
            _DECIMAL_SIGNED.fullmatch(den_exp_text)):
        raise AssertionError(f'noncanonical metric transport syntax: {tokens!r}')

    status = int(status_text)
    numerator = int(numerator_text, 16)
    numerator_exponent = int(exponent_text)
    denominator = int(denominator_text, 16)
    denominator_exponent = int(den_exp_text)

    if status != expected_status:
        raise AssertionError(f'wrong status: {status} != {expected_status}')
    if status != 0:
        # No failed call may leak even a mathematically harmless stale field.
        if expected_value is not None or any((numerator, numerator_exponent,
                                              denominator, denominator_exponent)):
            raise AssertionError(f'failure must clear every exported field: {tokens!r}')
        return

    if expected_value is None or denominator == 0:
        raise AssertionError(f'success requires a nonzero denominator: {tokens!r}')
    if numerator == 0 and numerator_exponent != 0:
        raise AssertionError(f'zero numerator requires exponent zero: {tokens!r}')
    if method == 'A':
        if (denominator, denominator_exponent) != (1, 0):
            raise AssertionError(f'absolute denominator must equal (1,0): {tokens!r}')
    elif method == 'R':
        # Normalize the *independently generated* positive native reference
        # exactly as a p-bit binary significand.  This uses integer bit_length
        # and shifts rather than copying the C frexp/scalbn path.
        if reference.significand == 0:
            raise AssertionError('relative reference zero cannot succeed')
        bits = reference.significand.bit_length()
        if bits > fmt.p:
            raise AssertionError('oracle generated a non-native significand')
        shift = fmt.p - bits
        required = (reference.significand << shift, reference.exponent - shift)
        if (denominator, denominator_exponent) != required:
            raise AssertionError(f'relative denominator {denominator, denominator_exponent}'
                                 f' != reference-normalized {required}')
    else:
        raise AssertionError(f'unknown method {method!r}')

    offset = numerator_exponent - denominator_exponent
    actual_value = (Fraction(numerator << offset, denominator) if offset >= 0
                    else Fraction(numerator, denominator << -offset))
    if actual_value != expected_value:
        raise AssertionError(f'wrong exact metric: {actual_value} != {expected_value}')


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
        try:
            verify_metric_record(reply.split(), next(f for f in FAMILIES if f.key == family),
                                 mode, r, expected_status, expected_value)
        except AssertionError as exc:
            raise AssertionError(
                f'case={index} family={family} mode={mode} '
                f'r={r.token()} o={o.token()} '
                f'expected={(expected_status, expected_value)}; '
                f'driver={reply!r}: {exc}'
            ) from exc
    print(f'PASS: {len(vectors)} exact metric comparisons '
          f'seed={args.seed:#x} random_per_family={args.per_family}')


if __name__ == '__main__':
    main()
