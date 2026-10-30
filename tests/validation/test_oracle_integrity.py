#!/usr/bin/env python3
"""VV0.1-B1b.3d-3c-2: fail-closed tests of the metric-oracle response parser.

The oracle is itself software that can be wrong.  These tests deliberately
supply malformed or semantically invalid records to the pure-Python response
validator; every such mutation MUST be rejected before numerical campaigns
are credited as evidence.  No C library, compiler, real test driver, network
access or external dependency is needed.  This makes the test usable in the
acceptance evidence script on every already-qualified Python installation.

Coverage boundaries: this tests the five-field exact-metric record contract,
not the correctness of Python's Fraction implementation, the trustworthiness
of host toolchains, C structure padding, or all conceivable failure modes.
"""
from __future__ import annotations

from fractions import Fraction
from pathlib import Path
import sys
import unittest

# Resolve a sibling module only.  Do not modify the user's working directory
# or import production artifacts as reference implementations.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_exact_dyadic_oracle import FAMILIES, integer, zero  # noqa: E402
from test_exact_metric_oracle import verify_metric_record  # noqa: E402


class MetricOracleProtocolTests(unittest.TestCase):
    """Check that the oracle rejects fake successes and stale failed output."""

    def setUp(self) -> None:
        """Select binary32 so all expected exact fields can be hand-checked."""
        self.fmt = FAMILIES[0]
        self.reference = integer(3)
        self.absolute = ('0', '1', '0', '1', '0')

    def check(self, record: tuple[str, ...], *, method: str = 'A',
              reference=None, status: int = 0, value=Fraction(1)) -> None:
        """Call the pure oracle validator with a reproducible fake C reply."""
        verify_metric_record(record, self.fmt, method,
                             self.reference if reference is None else reference,
                             status, value)

    def test_valid_absolute(self) -> None:
        """Exact magnitude one has a valid absolute rational encoding."""
        self.check(self.absolute)

    def test_valid_relative_non_dyadic(self) -> None:
        """The normalized binary32 representation of three encodes 1/3."""
        m = 3 << 22
        self.check(('0', '1', '0', f'{m:x}', '-22'), method='R',
                   value=Fraction(1, 3))

    def test_valid_canonical_failure(self) -> None:
        """A zero-reference relative rejection has all-zero metric fields."""
        self.check(('4', '0', '0', '0', '0'), method='R',
                   reference=zero(), status=4, value=None)

    def test_reject_failure_stale_numerator(self) -> None:
        """A failure must not leak a nonzero numerator despite D=0."""
        with self.assertRaises(AssertionError):
            self.check(('4', '1', '0', '0', '0'), method='R',
                       reference=zero(), status=4, value=None)

    def test_reject_failure_stale_exponent(self) -> None:
        """A failure must not leak even a nonzero exponent with N=0."""
        with self.assertRaises(AssertionError):
            self.check(('4', '0', '-7', '0', '0'), method='R',
                       reference=zero(), status=4, value=None)

    def test_reject_failure_stale_denominator_exponent(self) -> None:
        """Denominator exponent must also clear on every failure."""
        with self.assertRaises(AssertionError):
            self.check(('4', '0', '0', '0', '19'), method='R',
                       reference=zero(), status=4, value=None)

    def test_reject_wrong_status(self) -> None:
        """The oracle must never accept an unexpected success status."""
        with self.assertRaises(AssertionError):
            self.check(('0', '0', '0', '0', '0'), method='R',
                       reference=zero(), status=4, value=None)

    def test_reject_success_zero_denominator(self) -> None:
        """A successful record cannot contain D=0, even for N=0."""
        with self.assertRaises(AssertionError):
            self.check(('0', '0', '0', '0', '0'), value=Fraction(0))

    def test_reject_absolute_equivalent_denominator(self) -> None:
        """A numerically equivalent ratio is not a valid absolute encoding."""
        with self.assertRaises(AssertionError):
            self.check(('0', '2', '0', '2', '0'))

    def test_reject_relative_equivalent_denominator(self) -> None:
        """Relative denominator must be the native reference, not a reduction."""
        with self.assertRaises(AssertionError):
            self.check(('0', '1', '0', '3', '0'), method='R',
                       value=Fraction(1, 3))

    def test_reject_zero_noncanonical_exponent(self) -> None:
        """An exact zero numerator MUST have canonical E=0."""
        with self.assertRaises(AssertionError):
            self.check(('0', '0', '5', '1', '0'), value=Fraction(0))

    def test_accept_canonical_successful_zero(self) -> None:
        """A fully canonical zero magnitude is a valid successful output."""
        self.check(('0', '0', '0', '1', '0'), value=Fraction(0))

    def test_reject_bad_hex_and_negative_numerator(self) -> None:
        """Malformed and negative unsigned hexadecimal fields fail closed."""
        for token in ('-1', '0x1', 'NaN', 'zz', ''):
            with self.subTest(token=token), self.assertRaises(AssertionError):
                self.check(('0', token, '0', '1', '0'))

    def test_reject_bad_field_count(self) -> None:
        """A partial or extended response line cannot be silently accepted."""
        with self.assertRaises(AssertionError):
            self.check(('0', '1', '0', '1'))
        with self.assertRaises(AssertionError):
            self.check(('0', '1', '0', '1', '0', 'unexpected'))

    def test_reject_wrong_numerical_result(self) -> None:
        """Correct field shape never excuses an incorrect rational result."""
        with self.assertRaises(AssertionError):
            self.check(('0', '2', '0', '1', '0'))


if __name__ == '__main__':
    unittest.main()
