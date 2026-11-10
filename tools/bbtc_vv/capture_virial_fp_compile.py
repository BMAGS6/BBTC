#!/usr/bin/env python3
"""Capture (but do not certify) B2b.3c.2 producer compile-command evidence.

This optional, developer-only helper reads a CMake ``compile_commands.json``
produced with ``CMAKE_EXPORT_COMPILE_COMMANDS=ON``. It does not rebuild any
object, alter BBTC, or grant qualification. Its output records the actual
compiler invocation for each numerical producer/adapter/comparator/runner
translation unit, source and object SHA-256 identities where available, and a
conservative strict-floating-point flag screen.

Important limitations
---------------------
A command listing flags is not equivalent to a semantic audit of those flags,
the target architecture, the actual executable, or the generated instructions.
In particular, toolchain wrappers, environment variables, compiler defaults,
response files, link-time optimization, and target-specific flags can change
what is ultimately executed. All good-looking results are explicitly named
``FLAGS_SCREENED_UNQUALIFIED`` and require B2b.3c.3 manual review.

This program depends only on the Python standard library. It is NOT invoked by
ordinary production builds or CTest; Python is not a mandatory build tool.
No project file is written: by default evidence is placed in the supplied
CMake build directory (outside the BBTC source worktree).
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shlex
import subprocess
import sys


# Every source is tied to one distinct compilation. An adapter being compiled
# strictly does NOT prove the independently compiled production object is.
SOURCES = {
    "production_virial": "src/internal/first_order_virial_gas_model.c",
    "observation_adapter": "tools/bbtc_vv/virial_adapter.c",
    "b1_comparison": "tools/bbtc_vv/comparison_api.c",
    "reference_matrix": "tests/validation/test_virial_reference_matrix.c",
    "fp_preflight": "tools/bbtc_vv/virial_fp_profile.c",
    # B2b.3c.3a: the register-observation module must also have a distinct
    # compiler invocation and object identity in new qualification evidence.
    "runtime_control": "tools/bbtc_vv/virial_fp_control.c",
}

# These are policy checks for *observed command arguments*. They deliberately
# do not infer flags from a nearby target or the CMake cache.
REQUIRED = ("-frounding-math", "-ffp-contract=off", "-fno-fast-math")
UNSAFE = {
    "-ffast-math", "-Ofast", "-funsafe-math-optimizations",
    "-fassociative-math", "-ffinite-math-only", "-fno-rounding-math",
    "-ffp-contract=fast", "-ffp-contract=on", "-freciprocal-math",
}


def digest_file(path: Path) -> str | None:
    """Return SHA-256 of readable on-disk bytes or None when unavailable.

    Hashing the *actual object file* helps a later reviewer associate the
    captured compiler command with a concrete build output. It cannot prove
    that an object really came from the command or wasn't later overwritten.
    """
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def arguments(entry: dict) -> list[str]:
    """Read either standard compile database argument representation.

    ``arguments`` is preferred to ``command``; the latter must be tokenized
    using shell quoting rules. No shell command is executed by this helper.
    A compiler response file (``@flags.rsp``) is not expanded or trusted.
    """
    value = entry.get("arguments")
    if isinstance(value, list) and all(isinstance(v, str) for v in value):
        return value
    command = entry.get("command")
    return shlex.split(command) if isinstance(command, str) else []


def input_path(entry: dict) -> Path:
    """Normalize a compilation unit against its declared working directory."""
    cwd = Path(entry.get("directory", ".")).resolve()
    path = Path(entry.get("file", ""))
    return (path if path.is_absolute() else cwd / path).resolve()


def output_path(entry: dict, argv: list[str]) -> Path | None:
    """Find the object named by -o; do not guess a generator-dependent path."""
    output = entry.get("output")
    if not isinstance(output, str):
        for i, token in enumerate(argv):
            if token == "-o" and i + 1 < len(argv):
                output = argv[i + 1]
                break
            if token.startswith("-o") and len(token) > 2:
                output = token[2:]
                break
    if not isinstance(output, str):
        return None
    cwd = Path(entry.get("directory", ".")).resolve()
    path = Path(output)
    return (path if path.is_absolute() else cwd / path).resolve()


def screen_flags(argv: list[str]) -> tuple[list[str], list[str]]:
    """Return missing required options and conservative suspicious options.

    The screen rejects any *appearance* of an unsafe option even if a later
    flag might cancel it. This intentionally produces false negatives rather
    than silently classifying ambiguous override sequences as qualified.
    Compiler response files are always unresolved and are reported separately.
    """
    missing = [flag for flag in REQUIRED if flag not in argv]
    unsafe = sorted(set(arg for arg in argv if arg in UNSAFE or arg.startswith("@")))
    return missing, unsafe



def toolchain_identity(argv: list[str]) -> dict:
    """Report the executable's claimed version and target without trusting it.

    The first argument may be a wrapper (e.g. ccache). In that event the
    command is still recorded verbatim, but the reported version/triple may
    describe the wrapper rather than the actual compiler. A reviewer must
    resolve that distinction; this helper never confers qualification.
    """
    if not argv:
        return {"invoked": None, "version_first_line": None, "target": None}

    def read_option(option: str) -> str | None:
        try:
            completed = subprocess.run(
                [argv[0], option], text=True, capture_output=True,
                timeout=6, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        if completed.returncode != 0:
            return None
        return completed.stdout.splitlines()[0] if completed.stdout.splitlines() else None

    return {
        "invoked": argv[0],
        "version_first_line": read_option("--version"),
        "target": read_option("-dumpmachine"),
    }



def source_git_identity(source_root: Path) -> dict:
    """Record Git source identity without modifying or trusting the worktree.

    A dirty worktree is not itself a qualification rejection here: these tools
    are commonly run before a patch is committed. The later reviewer must
    reconcile the exact source SHA-256 values with the final Git checkpoint.
    """
    def git(*args: str) -> str | None:
        try:
            result = subprocess.run(
                ["git", "-C", str(source_root), *args],
                text=True, capture_output=True, timeout=6, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        return result.stdout.strip() if result.returncode == 0 else None

    return {
        "head": git("rev-parse", "HEAD"),
        "worktree_porcelain": git("status", "--porcelain"),
    }


def capture(source_root: Path, build_root: Path) -> dict:
    """Read six independent translation-unit commands without executing them.

    Duplicate entries for one source are unresolved because the tool cannot
    infer which object belongs to the link under review. Missing entries are
    unresolved for the same reason. All findings remain *unqualified* even
    when the narrow command-line flag screen finds no obvious problems.
    """
    database = build_root / "compile_commands.json"
    with database.open(encoding="utf-8") as stream:
        entries = json.load(stream)
    if not isinstance(entries, list):
        raise ValueError("compile_commands.json must contain an array")

    recorded: dict[str, dict] = {}
    unresolved = False
    for role, name in SOURCES.items():
        path = (source_root / name).resolve()
        matches = [entry for entry in entries if input_path(entry) == path]
        if len(matches) != 1:
            recorded[role] = {
                "source": name,
                "status": "UNRESOLVED",
                "reason": f"expected one compile entry, found {len(matches)}",
            }
            unresolved = True
            continue

        entry = matches[0]
        argv = arguments(entry)
        missing, unsafe = screen_flags(argv)
        obj = output_path(entry, argv)
        object_digest = digest_file(obj) if obj is not None else None
        reasons = []
        if not argv:
            reasons.append("missing compile arguments")
        if missing:
            reasons.append("missing required strict-FP options")
        if unsafe:
            reasons.append("unsafe, ambiguous, or unexpanded options")
        if object_digest is None:
            reasons.append("compiled object not present or -o not identified")
        if reasons:
            unresolved = True

        recorded[role] = {
            "source": name,
            "source_sha256": digest_file(path),
            "object": str(obj) if obj is not None else None,
            "object_sha256": object_digest,
            "directory": str(Path(entry["directory"]).resolve()),
            "compiler_argv": argv,
            "toolchain_identity": toolchain_identity(argv),
            "missing_strict_options": missing,
            "suspicious_options": unsafe,
            "status": "UNRESOLVED" if reasons else "FLAGS_SCREENED_UNQUALIFIED",
            "reasons": reasons,
        }

    return {
        "phase": "VV0.1-B2b.3c.2",
        "verdict": "UNRESOLVED" if unresolved else "FLAGS_SCREENED_UNQUALIFIED",
        "qualification_granted": False,
        "warning": (
            "Compiler options are an audit input; inspect actual objects "
            "and fenv before qualification."
        ),
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "source_root": str(source_root),
        "source_git": source_git_identity(source_root),
        "build_root": str(build_root),
        "machine": platform.machine(),
        "platform": platform.platform(),
        "compile_commands_sha256": digest_file(database),
        "translation_units": recorded,
    }


def main() -> int:
    """Produce an auditable JSON record and fail closed on unresolved inputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=Path.cwd())
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    options = parser.parse_args()

    source = options.source_dir.resolve()
    build = options.build_dir.resolve()
    output = options.output or (build / "bbtc_vv_virial_fp_compile_evidence.json")
    output = output.resolve()
    if source in output.parents:
        parser.error("evidence output must be outside the source worktree")

    try:
        report = capture(source, build)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"UNRESOLVED: cannot capture compile evidence: {exc}", file=sys.stderr)
        return 2

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"VVUQ compile evidence: {report['verdict']} ({output})")
    for role, item in report["translation_units"].items():
        print(f"  {role}: {item['status']}")
    print("No scientific or compiler qualification was granted.")
    return 0 if report["verdict"] == "FLAGS_SCREENED_UNQUALIFIED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
