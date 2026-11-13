#!/usr/bin/env python3
"""Run a bounded, reproducible source-mutation challenge against VV0.1-B2b.

This is a DEVELOPER-ONLY numerical-verification tool, not a production BBTC
component. It never edits the user's checkout. Instead, it archives one
committed Git revision into an automatically removed temporary directory,
compiles a clean reference-runner baseline, then builds one deliberately
mutated producer variant at a time.

A mutant is considered *detected* only when:
    1. its source mutation matched exactly one evaluated expression;
    2. its production object and reference runner rebuilt successfully; and
    3. the actual cmocka `test_complete_reference_matrix` test failed.

Compiler errors, missing tests, process crashes and setup failures are NOT
counted as successful detections. A deliberately out-of-scope left-clamp
mutation is expected to survive the four frozen dyadic reference cases; this
is evidence of the test matrix's declared boundary, not a defect in the suite.

Neither mutation detection nor a full green campaign constitutes general
compiler qualification, arbitrary-input correctness, or model validation.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import difflib
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile


PRODUCER = Path("src/internal/first_order_virial_gas_model.c")
RUNNER_TARGET = "bbtc_vv_test_virial_reference_matrix"
RUNNER_RELATIVE = Path("vv-validation-tests") / RUNNER_TARGET
FAMILIES = {"float": "f", "double": "", "long_double": "L"}
STRICT_FLAGS = "-O1 -g -frounding-math -ffp-contract=off -fno-fast-math"

# Anchor each mutation to a source-level expression, NOT to a line number.
# Formatting may change without invalidating the mathematical meaning; if an
# expected expression is no longer unambiguously identifiable, abort rather
# than silently mutating an unrelated part of the implementation.
@dataclass(frozen=True)
class Mutation:
    """One fault hypothesis, its single-expression edit and expected outcome.

    `pattern` and `replacement` use `{suffix}` for the native literal suffix
    and standard Python regex named groups. Each family is mutated ALONE so
    a failure in one precision cannot mask a missing test in another.
    """

    identity: str
    explanation: str
    pattern: str
    replacement: str
    expected: str  # DETECTED or SURVIVED; never inferred from observed output.


MUTATIONS = (
    Mutation(
        "normalization_offset",
        "Replace x = 2*(T-Tmin)/range - 1 with the wrong +1 offset.",
        r"(?P<head>\)\s*)-\s*1\.0{suffix}\s*;",
        r"\g<head>+ 1.0{suffix};",
        "DETECTED",
    ),
    Mutation(
        "temperature_scale",
        "Halve dx/dT; both nonzero temperature derivatives should expose it.",
        r"(?P<head>temperature_scale_per_k\s*=\s*)2\.0{suffix}"
        r"(?P<tail>\s*/\s*temperature_range_k\s*;)",
        r"\g<head>1.0{suffix}\g<tail>",
        "DETECTED",
    ),
    Mutation(
        "reconstruction_sign",
        "Reverse the b2 sign in the reconstructed Chebyshev value.",
        r"(?P<head>polynomial_value\s*=\s*normalized_temperature\s*\*\s*"
        r"value_next\s*)-\s*(?P<tail>value_next_next)",
        r"\g<head>+ \g<tail>",
        "DETECTED",
    ),
    Mutation(
        "second_derivative_factor",
        "Halve the analytically required factor of two in d2B/dx2.",
        r"(?P<head>second_derivative_wrt_x\s*=\s*)2\.0{suffix}"
        r"(?P<tail>\s*\*\s*first_next)",
        r"\g<head>1.0{suffix}\g<tail>",
        "DETECTED",
    ),
    Mutation(
        "first_derivative_guard",
        "Invert the zero derivative branch, suppressing required dB/dT values.",
        r"(?P<head>if\s*\(\s*first_derivative_wrt_x\s*)!=\s*0\.0{suffix}"
        r"(?P<tail>\s*\))",
        r"\g<head>== 0.0{suffix}\g<tail>",
        "DETECTED",
    ),
    Mutation(
        "inactive_left_clamp_control",
        "Change only the x<-1 clamp threshold; frozen in-domain cases never take it.",
        r"(?P<head>if\s*\(\s*normalized_temperature\s*<\s*)-1\.0{suffix}"
        r"(?P<tail>\s*\))",
        r"\g<head>-2.0{suffix}\g<tail>",
        "SURVIVED",
    ),
)


class CampaignError(RuntimeError):
    """An infrastructure, identity or contract problem, NOT a detected mutant."""


def sha256(data: bytes) -> str:
    """Return a stable artifact identity, including for mutated source bytes."""
    return hashlib.sha256(data).hexdigest()


def run_command(args: list[str], log: Path, *, cwd: Path,
                timeout: int = 180) -> subprocess.CompletedProcess[str]:
    """Execute an explicit argument vector and retain all diagnostic output.

    No shell is used. Nonzero exits are returned to the caller for explicit
    classification rather than being mistaken for successful mutant kills.
    """
    try:
        result = subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                                errors="replace", timeout=timeout, check=False)
        log.write_text(f"$ {args!r}\nexit={result.returncode}\n"
                       f"--- stdout ---\n{result.stdout}\n"
                       f"--- stderr ---\n{result.stderr}\n", encoding="utf-8")
        return result
    except subprocess.TimeoutExpired as exc:
        log.write_text(f"$ {args!r}\nTIMEOUT after {timeout}s\n", encoding="utf-8")
        raise CampaignError(f"Timed out running {args[0]}; see {log}") from exc


def repository_root(script: Path) -> Path:
    """Locate the checkout containing this script without trusting cwd."""
    return script.resolve().parents[2]


def committed_head(repo: Path) -> str:
    """Read the immutable source identity; uncommitted drafts are excluded."""
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo,
                            capture_output=True, text=True, check=True)
    return result.stdout.strip()


def evaluator_span(source: str, family: str) -> tuple[int, int]:
    """Isolate one public temperature-law EVALUATOR, excluding validation code.

    Multiple family functions share similar text. Restricting edits to this
    exact range prevents corrupting another precision or a model validator.
    """
    begin = f"bbtc_ib_first_order_virial_temperature_law_evaluate_{family}("
    end = f"bbtc_ib_first_order_virial_gas_model_validate_{family}("
    if source.count(begin) != 1:
        raise CampaignError(f"Expected one evaluator definition: {family}")
    first = source.index(begin)
    try:
        last = source.index(end, first)
    except ValueError as exc:
        raise CampaignError(f"Missing evaluator boundary: {family}") from exc
    return first, last


def apply_mutation(original: str, family: str, spec: Mutation) -> str:
    """Apply exactly one fault to exactly one native evaluator.

    `re.subn` is deliberately strict. A changed source layout cannot silently
    produce a no-op mutation or transform two expressions at once. All other
    native functions remain byte-for-byte equal to the original source.
    """
    first, last = evaluator_span(original, family)
    suffix = FAMILIES[family]
    body = original[first:last]
    changed, count = re.subn(spec.pattern.format(suffix=suffix),
                             spec.replacement.format(suffix=suffix), body)
    if count != 1 or body == changed:
        raise CampaignError(f"{family}/{spec.identity}: expected exactly one edit; got {count}")
    return original[:first] + changed + original[last:]


def test_mutation_anchors() -> None:
    """Self-test all 18 lexical edits without a compiler or source checkout."""
    samples = {
        "normalization_offset": ") - 1.0{suffix};",
        "temperature_scale": "temperature_scale_per_k = 2.0{suffix} / temperature_range_k;",
        "reconstruction_sign": "polynomial_value = normalized_temperature * value_next\n"
                               "        - value_next_next + coefficients[0];",
        "second_derivative_factor": "second_derivative_wrt_x = 2.0{suffix} * first_next;",
        "first_derivative_guard": "if (first_derivative_wrt_x != 0.0{suffix})",
        "inactive_left_clamp_control": "if (normalized_temperature < -1.0{suffix})",
    }
    for family, suffix in FAMILIES.items():
        for spec in MUTATIONS:
            snippet = samples[spec.identity].format(suffix=suffix)
            updated, count = re.subn(spec.pattern.format(suffix=suffix),
                                      spec.replacement.format(suffix=suffix), snippet)
            if count != 1 or updated == snippet:
                raise CampaignError(f"Self-test failed: {family}/{spec.identity}")
    print("PASS: 18 native-family mutation anchors and replacements validated")


def write_summary(path: Path, record: dict) -> None:
    """Write JSON after every experiment so interrupted campaigns remain auditable."""
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def classify_test(result: subprocess.CompletedProcess[str], expected: str) -> str:
    """Accept only a named cmocka assertion failure as detection.

    A general nonzero return could mean a crash, dynamic-link failure, setup
    failure, ASan finding, or unrelated test failure. Those are not legitimate
    evidence that the expected numerical reference caught this mutation.
    """
    transcript = result.stdout + "\n" + result.stderr
    named_failure = re.search(
        r"\[\s*FAILED\s*\]\s*test_complete_reference_matrix\b", transcript
    ) is not None
    if expected == "DETECTED" and result.returncode != 0 and named_failure:
        return "DETECTED"
    if expected == "SURVIVED" and result.returncode == 0 and (
        "test_complete_reference_matrix" in transcript
    ):
        return "SURVIVED"
    return "UNEXPECTED"


def campaign(args: argparse.Namespace) -> int:
    """Run one GCC/Clang profile against a clean, committed source archive.

    The source and build directories live under TemporaryDirectory and are
    removed on normal exit and exceptions. Only textual logs, mutation diffs,
    digest records and the summary persist in the requested evidence folder.
    """
    repo = repository_root(Path(__file__))
    head = committed_head(repo)
    if head != args.expected_head:
        raise CampaignError(f"HEAD {head} does not match expected {args.expected_head}")

    producer_result = subprocess.run(
        ["git", "show", f"{head}:{PRODUCER.as_posix()}"], cwd=repo,
        capture_output=True, check=True
    )
    pristine_bytes = producer_result.stdout
    original = pristine_bytes.decode("utf-8")

    # Validate every mutation against the actual committed source BEFORE
    # creating a build or output directory. Every edit must be precise.
    edits = [(family, spec, apply_mutation(original, family, spec))
             for family in FAMILIES for spec in MUTATIONS]
    if args.plan:
        print(f"HEAD: {head}\nProducer SHA-256: {sha256(pristine_bytes)}")
        for family, spec, _ in edits:
            print(f"{family:12} {spec.identity:29} expected={spec.expected}")
        print("PASS: All 18 mutations match exactly one committed-source expression")
        return 0

    compiler = shutil.which(args.compiler)
    if compiler is None:
        raise CampaignError(f"C compiler not found: {args.compiler}")
    for tool in ("git", "cmake", "ninja", "pkg-config"):
        if shutil.which(tool) is None:
            raise CampaignError(f"Required build tool not found: {tool}")

    if args.output is None:
        raise CampaignError("--output is required for an evidence-producing campaign")
    output = args.output.expanduser().resolve()
    if output.is_relative_to(repo.resolve()):
        raise CampaignError("Evidence output must be outside the repository")
    if output.exists():
        raise CampaignError(f"Evidence directory already exists: {output}")
    output.mkdir(parents=True)

    summary = {
        "record_kind": "VV0.1-B2b.4b Stage 1 controlled source mutations",
        "status": "IN_PROGRESS", "source_commit": head,
        "producer_sha256": sha256(pristine_bytes),
        "compiler": compiler,
        "compiler_version": subprocess.run([compiler, "--version"], capture_output=True,
                                           text=True, check=True).stdout.splitlines()[0],
        "flags": STRICT_FLAGS,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "M-01..M-04 source-mutant detection; no compiler qualification",
        "experiments": [],
    }
    summary_file = output / "summary.json"
    write_summary(summary_file, summary)

    try:
        with tempfile.TemporaryDirectory(prefix="bbtc_vv_mutants_") as temporary:
            sandbox = Path(temporary)
            source_root = sandbox / "source"
            build_root = sandbox / "build"
            source_root.mkdir()
            archive = sandbox / "committed_source.tar"
            with archive.open("wb") as stream:
                subprocess.run(["git", "archive", "--format=tar", head],
                               cwd=repo, stdout=stream, check=True)
            # git archive is generated from the user's trusted committed tree.
            with tarfile.open(archive) as tar:
                tar.extractall(source_root, filter="data")
            source_file = source_root / PRODUCER
            if sha256(source_file.read_bytes()) != sha256(pristine_bytes):
                raise CampaignError("Archived producer bytes differ from committed blob")

            configure = [
                "cmake", "-S", str(source_root), "-B", str(build_root), "-G", "Ninja",
                f"-DCMAKE_C_COMPILER={compiler}", "-DCMAKE_BUILD_TYPE=Debug",
                f"-DCMAKE_C_FLAGS={STRICT_FLAGS}",
                "-DBBTC_BUILD_TESTS=OFF", "-DBBTC_BUILD_VVUQ=ON",
                "-DBBTC_VVUQ_BUILD_TESTS=ON", "-DBBTC_VVUQ_ORACLE_TESTS=OFF",
            ]
            if run_command(configure, output / "configure.log", cwd=repo).returncode != 0:
                raise CampaignError("Baseline CMake configuration failed")

            build = ["cmake", "--build", str(build_root), "--target", RUNNER_TARGET]
            if run_command(build, output / "baseline-build.log", cwd=repo).returncode != 0:
                raise CampaignError("Unmutated reference runner failed to compile")
            executable = build_root / RUNNER_RELATIVE
            if not executable.is_file():
                raise CampaignError(f"Reference runner missing: {executable}")
            baseline = run_command([str(executable)], output / "baseline-test.log",
                                   cwd=repo, timeout=45)
            if baseline.returncode != 0 or "test_complete_reference_matrix" not in (
                baseline.stdout + baseline.stderr
            ):
                raise CampaignError("Unmutated reference matrix is not green")
            summary["baseline_runner_sha256"] = sha256(executable.read_bytes())
            summary["baseline_status"] = "PASS"
            write_summary(summary_file, summary)

            # Each experiment starts from the original source. Explicitly
            # delete its old production object to force recompilation, even
            # on filesystems with unusual timestamp granularity.
            object_file = (build_root / "CMakeFiles" / "bbtc.dir" / "src" /
                           "internal" / "first_order_virial_gas_model.c.o")
            for family, spec, mutant in edits:
                name = f"{family}__{spec.identity}"
                work = output / name
                work.mkdir()
                source_file.write_text(mutant, encoding="utf-8")
                object_file.unlink(missing_ok=True)
                executable.unlink(missing_ok=True)

                diff = "".join(difflib.unified_diff(
                    original.splitlines(keepends=True), mutant.splitlines(keepends=True),
                    fromfile=f"a/{PRODUCER.as_posix()}",
                    tofile=f"b/{PRODUCER.as_posix()}"
                ))
                (work / "mutant.patch").write_text(diff, encoding="utf-8")
                entry = {
                    "id": name, "native_family": family,
                    "mutation": spec.identity,
                    "description": spec.explanation,
                    "expected": spec.expected,
                    "mutant_source_sha256": sha256(mutant.encode("utf-8")),
                }
                built = run_command(build, work / "build.log", cwd=repo)
                if built.returncode != 0 or not executable.is_file():
                    entry["observed"] = "INFRASTRUCTURE_FAILURE"
                    summary["experiments"].append(entry)
                    raise CampaignError(f"Mutant did not compile: {name}")
                if "first_order_virial_gas_model.c.o" not in (
                    built.stdout + built.stderr
                ):
                    entry["observed"] = "INFRASTRUCTURE_FAILURE"
                    summary["experiments"].append(entry)
                    raise CampaignError(f"Mutated producer was not recompiled: {name}")
                entry["runner_sha256"] = sha256(executable.read_bytes())
                result = run_command([str(executable)], work / "test.log", cwd=repo,
                                     timeout=45)
                entry["test_returncode"] = result.returncode
                entry["observed"] = classify_test(result, spec.expected)
                summary["experiments"].append(entry)
                write_summary(summary_file, summary)
                print(f"{entry['observed']:10} {name} (expected {spec.expected})", flush=True)
                if entry["observed"] != spec.expected:
                    raise CampaignError(f"Unanticipated mutant result: {name}; inspect {work}")

        summary["status"] = "PASS: all expected detections and scoped survivors observed"
        write_summary(summary_file, summary)
        print(f"PASS: 15 expected detections, 3 expected survivors; evidence at {output}")
        return 0
    except (CampaignError, OSError, subprocess.CalledProcessError) as exc:
        summary["status"] = f"INCOMPLETE_OR_FAILED: {exc}"
        write_summary(summary_file, summary)
        print(f"FAIL: {exc}; evidence at {output}", file=sys.stderr)
        return 1


def main() -> int:
    """Parse explicit, reproducible command-line inputs and run one campaign."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-head", required=False,
                        help="Required full 40-character committed source hash")
    parser.add_argument("--compiler", default="gcc", help="C compiler executable")
    parser.add_argument("--output", type=Path, help="New evidence directory outside the Git tree")
    parser.add_argument("--plan", action="store_true",
                        help="Validate all mutations against HEAD without compiling")
    parser.add_argument("--self-test", action="store_true",
                        help="Check mutation regexes without requiring a repository")
    args = parser.parse_args()
    if args.self_test:
        test_mutation_anchors()
        return 0
    if not args.expected_head or not re.fullmatch(r"[a-f0-9]{40}", args.expected_head):
        parser.error("--expected-head requires a complete 40-character Git SHA")
    try:
        return campaign(args)
    except (CampaignError, OSError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
