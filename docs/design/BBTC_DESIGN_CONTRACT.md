# BBTC Reconstruction Design Contract

**Contract version:** 0.1.22

**Project phase:** IB0.4f

**Applies to:** `rewrite/ib0_4f_thermochemical_source_rate_contract_v1`

**Status:** Accepted

**Date:** 2026-10-04

## 1. Purpose

This document defines the engineering boundary for the clean reconstruction of
BMAGS' Ballistic Trajectory Calculator (BBTC). It governs the architecture,
public behavior, numerical conventions, diagnostics, command-line presentation,
and scientific claims of the rewritten project.

The legacy implementation is preserved by the `legacy-pre-rewrite` tag. The
rewrite is not required to retain its source layout, APIs, output formats, or
behavior.

The first implementation target is a zero-dimensional internal-ballistics
solver. External ballistics will consume the muzzle state produced by internal
ballistics in a later project phase.

The words **MUST**, **MUST NOT**, **SHOULD**, **SHOULD NOT**, and **MAY** state
requirements. A requirement can change only through an explicit revision of
this contract.

## 2. Governing principles

BBTC is built around the following priorities, in order:

1. Clearly stated physical meaning and assumptions.
2. Correctness within the implemented model.
3. Detectable failures instead of plausible-looking nonsense.
4. Reproducibility and traceability.
5. Clean use as an independent C library.
6. Practical performance.
7. Pleasant command-line presentation.

Detail is added in layers. A new effect MUST have:

- a defined physical purpose;
- dimensions and units for every parameter;
- an independently testable implementation;
- documented applicability limits;
- a way to determine whether it was active;
- evidence that it improves the model or exposes useful information.

An elaborate but unvalidated effect is not automatically better than a simpler,
well-characterized one.

### 2.1 Engineering fidelity and model-inclusion policy

BBTC targets engineering-appropriate predictive fidelity rather than maximal
mechanistic resolution. Additional physical detail is valuable only when the
information and model required to use it are sufficiently trustworthy to improve
the intended engineering result.

Before a new physical effect, correction, or model term becomes part of the
default engineering model, its design *MUST* address all of the following:

1. Can the effect *materially change one or more outputs* that BBTC is intended
   to predict?

2. Can the parameters required by the model realistically be known, measured,
   calibrated, or otherwise supplied with *useful accuracy*?

3. Can the effect be modeled without introducing assumptions or parameter
   uncertainty comparable to or larger than the effect being modeled?

4. Is the expected improvement *materially distinguishable* relative to ordinary
   experimental, calibration, shot-to-shot, or existing-model uncertainty, or
   does the model materially expand BBTC's documented applicability or
   diagnostic value?

An effect that fails these tests SHOULD remain deferred, optional, or outside
the core engineering model until evidence justifies its inclusion. A physical
effect MUST NOT be added to the default model merely because it *can* be modeled.
Just because it can be modeled does not mean that it necessarily *should*.

Conversely, an effect MUST NOT be rejected solely because its implementation is
complex when validation or sensitivity analysis shows that it materially affects
the intended engineering outputs and its required parameters can be supported
with useful fidelity.

BBTC SHOULD prefer a simpler well-characterized model whose residual error lies
within the relevant experimental or application uncertainty over a more
elaborate model whose additional inputs are unknown or weakly constrained.

Sensitivity, uncertainty, and physical-validation studies SHOULD be used to
revisit these decisions as the coupled solver matures. Model complexity is
therefore evidence-driven rather than monotonically increasing.

### 2.2 Numerical fidelity and engineering error-budget policy

BBTC predicts physical behavior; it does not reproduce an unknowable exact
trajectory, exact propellant-combustion history, or exact chamber state.
Experimental scatter, parameter uncertainty, empirical-model error, unmodeled
physics, manufacturing variation, and numerical error are distinct contributors
to the total prediction error budget. A numerically precise answer MUST NOT be
presented as physically exact merely because the arithmetic converged tightly.

The numerical implementation SHOULD be accurate enough that avoidable numerical
error is materially smaller than the uncertainty of the physical models and
input data governing the quantity of interest. As an engineering guideline,
when a credible physical/model/input uncertainty estimate exists, BBTC SHOULD
normally target numerical error no larger than approximately one tenth of that
uncertainty when doing so is practical. This approximately 10:1 separation is a
design guideline, not a universal mathematical guarantee or a substitute for
convergence testing.

Numerical rigor remains important even when physical uncertainty is much larger.
Implementations MUST still:

- use the selected native scalar family without silently routing one precision
  through another;
- prefer algebraically equivalent forms that avoid unnecessary overflow,
  underflow, catastrophic cancellation, or loss of significance;
- preserve exact semantic boundaries when the contract defines them as exact;
- report an explicit numerical failure when a mathematically required result
  cannot be represented with the required sign and finiteness;
- avoid clipping, saturation, or fabricated fallback values unless a separate
  contract explicitly defines such behavior; and
- support convergence or sensitivity checks where an iterative or discretized
  algorithm can materially affect reported outputs.

Conversely, BBTC MUST NOT demand bit-for-bit identities between independently
computed interior derived quantities when doing so would require a numerically
inferior operation sequence or would provide no meaningful engineering benefit.
Small native-rounding differences are acceptable when each quantity is computed
from a stable form of the same physical contract and the discrepancy is
negligible relative to the applicable engineering error budget.

Exact software semantics remain exact even when the underlying physical model is
uncertain. Status precedence, deterministic failure-output clearing, exact
identity states, exact supported endpoints, and explicitly specified zero-state
behavior are contract requirements rather than statistical predictions.

Tolerance selection for the future coupled solver SHOULD therefore be justified
by convergence studies and by the physical/model uncertainty relevant to the
reported result. Tightening a tolerance beyond the point at which output changes
are immaterial relative to the engineering uncertainty is not automatically an
improvement. Likewise, a numerically cheap approximation is acceptable only
when its error remains controlled and immaterial at the intended fidelity.

The purpose of this policy is to keep BBTC's numerical machinery quieter than
the physics it is attempting to model. Model uncertainty MUST NOT be used as an
excuse for avoidable numerical error, and numerical precision MUST NOT be used
to imply physical certainty that the model or data do not support.

## 3. Project boundaries

### 3.1 Physics library

The physics library MUST:

- expose a C23 API under `include/bbtc/`;
- use the `bbtc_` prefix for public functions, types, and symbols;
- accept fully specified physical inputs from the caller;
- return structured results, diagnostics, and model-limit information;
- contain no command-line parsing or terminal-presentation logic;
- contain no *required* file I/O;
- contain no process termination calls;
- contain no hidden mutable global state;
- be reentrant and safe for concurrent simulations with distinct caller-owned
  inputs, outputs, workspaces, and history sinks;
- perform no mandatory dynamic allocation during a simulation;
- be usable without the BBTC command-line executable;
- be consumable by Cubes of Honor without adopting BBTC's user interface,
  atmosphere system, logging system, or data-file conventions.

The library MUST NOT read environment variables, inspect terminals, emit ANSI
escape sequences, print warnings, or call `exit()`.

Immutable compile-time tables are permitted. Any future shared cache MUST be
optional, explicitly owned, and independently synchronized.

### 3.2 Command-line application

The command-line application is a client of the library. It MAY:

- parse human-friendly units and convert them to SI;
- load and save user-facing files;
- select catalog records;
- format human-readable reports;
- emit documented machine-readable formats;
- detect terminal capabilities;
- apply ANSI styling;
- present concise safety and model notices.

The CLI MUST *NOT* contain an alternate physics implementation. All physical
results MUST come through the public or explicitly designated internal BBTC
library API.

### 3.3 Cubes of Honor

Cubes of Honor is a separate consumer. BBTC supplies physical predictions such
as pressure history, muzzle state, and model-limit flags. Cubes of Honor owns:

- weapon durability and condition;
- gameplay damage;
- cartridge-malfunction behavior;
- overload consequences;
- stochastic weapon-failure or explosion probabilities;
- audiovisual presentation;
- networking and replication.

BBTC MUST *NOT* encode a gameplay probability that a weapon explodes. A future
material-stress module may expose physical loads or estimated failure margins,
but it still MUST *NOT* label a real cartridge or firearm as necessarily safe.

## 4. Source and build architecture

The intended top-level layout is:

```
CMakeLists.txt
cmake/
include/bbtc/
src/
    internal/
apps/
    bbtc_cli/
tests/
docs/
    design/
```

The build MUST provide:

- a library target named `bbtc`;
- a namespaced alias named `bbtc::bbtc`;
- an optional CLI target named `bbtc_cli`;
- optional tests registered with CTest;
- target-local compile options and include paths;
- `C_EXTENSIONS OFF`;
- explicit source lists rather than recursive source globbing.

The build MUST NOT:

- impose warning flags, language options, or global definitions on a parent
  project;
- require the CLI in order to build the library;
- write generated artifacts into tracked source directories;
- track object files, executables, generated API documentation, CMake caches, or
  build directories.

When BBTC is the top-level project, developer-friendly defaults MAY enable the
CLI and tests. When included as a subproject, optional applications, tests, and
documentation SHOULD default to disabled.

The baseline library SHOULD depend only on the C standard library and the
platform math library. New mandatory dependencies require a contract revision
and a stated benefit.

All supported scalar-precision families MUST be available through the same
`bbtc` library target and `bbtc::bbtc` alias. A consumer selects precision
through a precision-qualified public type and function, not through a
translation-unit definition that changes the layout or meaning of an
unsuffixed public type.

## 5. C language and naming rules

The rewritten code targets portable ISO C23.

Library code MUST:

- compile without relying on GNU language extensions;
- use four-space indentation and Allman braces;
- use `snake_case`;
- suffix typedef names with `_t`;
- suffix enumeration typedef names with `_e`;
- prefix public identifiers with `bbtc_` or `BBTC_`;
- use fixed-width integer types when representation width matters;
- use fixed underlying types for public enumerations;
- avoid variable-length arrays;
- document all public declarations;
- place C++ linkage guards around public headers;
- expose no undocumented compiler-specific type in the public API.

Implementation-specific optimizations MAY be added behind capability checks and
MUST retain a portable reference path.

## 6. Units and dimensional naming

### 6.1 Canonical units

The library API and computational core use SI units exclusively:

| Quantity             | Unit                     | Naming suffix |
| -------------------- | ------------------------ | ------------- |
| Length               | meter                    | `_m`          |
| Area                 | square meter             | `_m2`         |
| Volume               | cubic meter              | `_m3`         |
| Mass                 | kilogram                 | `_kg`         |
| Time                 | second                   | `_s`          |
| Mass rate            | kilogram per second      | `_kg_per_s`   |
| Power                | watt                     | `_w`          |
| Velocity             | meter per second         | `_m_per_s`    |
| Acceleration         | metre per second squared | `_m_per_s2`   |
| Force                | newton                   | `_n`          |
| Absolute pressure    | pascal                   | `_pa`         |
| Energy               | joule                    | `_j`          |
| Specific energy      | joule per kilogram       | `_j_per_kg`   |
| Absolute temperature | kelvin                   | `_k`          |
| Angle                | radian                   | `_rad`        |
| Angular velocity     | radian per second        | `_rad_per_s`  |
| Density              | kilogram per cubic metre | `_kg_per_m3`  |

Pressure values in the library are absolute unless a field explicitly states
otherwise. Gauge pressure MUST NOT be accepted by an ambiguously named field.
Temperature values are absolute kelvin. Fractions are dimensionless and use the
closed interval `[0, 1]` unless otherwise documented.

Every dimensional public field MUST carry a unit suffix. Generic names such as
`pressure`, `temperature`, `volume`, `mass`, or `velocity` are forbidden in the
public API.

### 6.2 Conversions

Unit conversion belongs outside the solver. The CLI MAY accept grains, inches,
feet per second, pounds per square inch, degrees, and other documented units,
but it MUST convert them exactly once at the input boundary.

The library MUST NOT infer a unit from magnitude and MUST NOT silently convert
or reinterpret a value.

Conversion constants MUST be centralized and tested. Parsing and formatting
MUST remain locale-independent unless localization is introduced deliberately;
machine-readable numbers always use `.` as the decimal separator.

## 7. Numeric types and behavior

### 7.1 Scalar-precision families

BBTC provides three first-class continuous-scalar families based on the ISO C
types:

- `float`;
- `double`;
- `long double`.

Each family MUST implement the same physical equations, model terms, event
definitions, validation rules, and diagnostic semantics. The `float` and
`long double` families MUST NOT be wrappers that merely convert their inputs to
`double`, call the `double` solver, and convert the result back.

Within one simulation, continuous physical inputs, solver options, dynamic
state, intermediate calculations, workspace records, history samples, and
continuous result fields use the selected scalar family. Enumerations,
bitmasks, counters, indexes, version fields, and other discrete values retain
their explicitly declared integer types.

Mixing precision-qualified problem, options, workspace, history, or result
types in one call is forbidden and MUST require a compile-time diagnostic.
Conversions between precision families, where eventually provided, MUST be
explicitly requested and documented; they are not part of simulation dispatch.

The `double` family is the reference family for published regression values,
validation work, and default CLI operation. This does not make `double` more
physically valid than the supplied model or data. The `float` family is a
reduced-storage, lower-precision implementation intended for
performance-sensitive consumers, while `long double` is available for numerical
investigation and platforms on which it provides additional precision or range.

ISO C does not require `long double` to be wider than `double`. BBTC MUST
document the actual `FLT_RADIX`, significand precision, exponent range, and
storage size of each enabled family. It MUST NOT describe the `long double`
family as binary80, 80-bit, quadruple precision, or another specific format
unless the platform actually provides that format.

IB0.3a fixes `bbtc_precision_e` as a `uint8_t` public enumeration with the
following stable values:

| Value | Enumerator                   | ISO C family  |
| ----: | ---------------------------- | ------------- |
|     1 | `BBTC_PRECISION_FLOAT`       | `float`       |
|     2 | `BBTC_PRECISION_DOUBLE`      | `double`      |
|     3 | `BBTC_PRECISION_LONG_DOUBLE` | `long double` |

Zero and every unrecognized value are invalid and select no scalar family.
`bbtc_precision_string()` returns `"float"`, `"double"`, or
`"long double"` for the defined values and `"unknown BBTC precision"`
otherwise.

`bbtc_precision_info_t` and `bbtc_precision_info()` report the linked
library build's `FLT_RADIX`, matching `*_MANT_DIG`, `*_MIN_EXP`,
`*_MAX_EXP`, `*_DECIMAL_DIG`, and `sizeof` values. Precision identity is
metadata and MUST NOT replace precision-qualified simulation records or
functions with a runtime `void*` dispatch interface.

`_Float16` is not a first-class BBTC solver family. Its finite range cannot
represent the required public SI pressure domain in pascals, and its
precision is not suitable for the adaptive-integration and event-location
contract. A future API MAY use an explicitly documented half-precision
representation for bounded storage or interchange, but it MUST NOT advertise
that format through `bbtc_precision_e` unless a complete, scientifically
defensible solver family exists.

### 7.2 Concrete and generic APIs

Actual linkable functions and public record types MUST be
precision-qualified. The intended naming pattern is:

```c
bbtc_ib_problem_float_t
bbtc_ib_problem_double_t
bbtc_ib_problem_long_double_t

bbtc_ib_simulate_float(...)
bbtc_ib_simulate_double(...)
bbtc_ib_simulate_long_double(...)
```

The exact simulation declarations will be reviewed as IB0.3 proceeds, but an
unsuffixed public `bbtc_real_t` whose meaning changes with a build definition
is forbidden. Such a switch would make headers, object files, and ABI identity
depend on matching hidden configuration.

For C callers, BBTC will provide an optional header-only `_Generic` convenience
interface with an unsuffixed spelling such as:

```c
bbtc_ib_simulate(problem, options, result)
```

The generic interface selects a concrete function from the
precision-qualified problem type. It MUST:

- perform compile-time type dispatch with no runtime branch;
- evaluate each runtime argument exactly once;
- preserve the concrete function's validation and return semantics;
- reject unsupported or mixed precision-qualified records with a diagnostic;
- remain optional so callers may invoke or take the address of a concrete
  function directly.

The generic interface is convenience syntax, not the ABI. Public C++ headers
MUST leave the concrete `extern "C"` functions usable without requiring
`_Generic`.

### 7.3 Type-correct calculations

Continuous calculations MUST be expressed in their selected scalar family
unless a specific mixed-precision algorithm is later documented, justified, and
tested. Implementations MUST use type-correct constants, math functions,
finiteness checks, quiet NaNs, and machine-limit values. An accidental
language-level promotion to `double` or narrowing from `long double` is a
defect.

Compiler-permitted excess evaluation precision is a platform characteristic
that MUST be accounted for in reproducibility work; it is not permission to
route a precision family through another solver.

The public API remains SI-only. Internally, a solver MAY use documented
nondimensionalization or deterministic scaling to improve conditioning. Such
scaling MUST preserve the meaning of public tolerances, guards, histories, and
results and MUST NOT conceal overflow, underflow, or loss of applicability.

Default absolute and relative tolerances MUST be defined separately for each
scalar family. A requested tolerance below that implementation's documented
meaningful floor MUST be rejected as an invalid solver option; it MUST NOT be
silently raised. Solver defaults and validity domains MUST not assume that a
numerically suitable `double` configuration is automatically suitable for
`float`.

### 7.4 Floating-point behavior

Reference builds of every scalar family MUST NOT use options such as
`-ffast-math` that discard IEEE floating-point semantics or assume non-finite
values cannot occur.

Every floating-point input MUST be checked for finiteness before use. Required
positive quantities, nonnegative quantities, fractions, geometry relationships,
and model-specific domains MUST be validated explicitly.

The library MUST NOT:

- convert a malformed number to zero;
- silently clamp an invalid input into range;
- continue after a non-finite solver state;
- hide an integration failure behind a nominal-looking result;
- use a safety limit as an undocumented numerical constant.

If the caller requests a numerical guard such as maximum simulated pressure,
time, or step count, reaching it is a named termination condition. A numerical
pressure guard is not a firearm safety rating.

The same inputs, precision family, model selections, solver options, compiler,
platform, and floating-point environment SHOULD reproduce the same result.
Bit-for-bit identity across different compilers or architectures is not promised
unless a future deterministic mode defines and tests it.

Adaptive-solver tolerances, accepted and rejected step counts, termination
reason, and event location MUST be observable in the result or diagnostics.

No precision family is presumed sufficiently fast or accurate for a particular
consumer merely because of its C type. Performance and error are measured
separately. In particular, use of `float` by Cubes of Honor depends on benchmarks
and a declared accuracy envelope over the game's intended input domain.

## 8. API ownership and state

Inputs are caller-owned and treated as immutable for the duration of a call.
Outputs, optional workspace, and optional history storage are caller-owned.
All such records participating in one simulation MUST belong to the same
scalar-precision family.

The library MUST define and document:

- who owns every pointer;
- how long referenced memory must remain valid;
- what may alias;
- whether a callback is synchronous;
- what state is valid after success, warning, or failure.

A simulation MUST be possible without collecting a time history.

The ordinary simulation path MUST NOT require heap allocation. Optional history
collection will use caller-provided storage, a synchronous caller callback, or
both; the exact public history interface is deferred to the IB0.2 API review.

If history storage fills, the solver MUST continue unless the caller explicitly
requested history completeness as a hard requirement. Truncation MUST be
reported and MUST NOT alter the physical integration path.

No error information may be stored in a process-global "last error" object.
Human-readable status strings returned by the library must be immutable,
nonlocalized, and safe to read concurrently.

## 9. Status, termination, warning, and validity semantics

BBTC uses separate channels for separate meanings.

### 9.1 Function status

`bbtc_status_e` reports whether the API call was performed successfully. Status
categories MUST distinguish at least:

- success;
- null or invalid argument;
- NaN input;
- positive or negative infinity input;
- value outside the mathematical domain;
- inconsistent geometry or configuration;
- unsupported model or option;
- insufficient caller-provided storage;
- numerical failure;
- iteration or step limit;
- internal invariant failure.

Status value zero is success. A nonzero status MUST never mean "success with a
physical warning."

IB0.3a fixes the underlying representation of `bbtc_status_e` as `uint8_t`.
Values 0 through 9 retain their established meanings and numeric assignments.
IB0.4c appends `BBTC_STATUS_NAN_INPUT` as value 10 without renumbering,
aliasing, or reusing any earlier value. The current stable values and
nonlocalized strings are:

| Value | Enumerator                                |
| ----: | ----------------------------------------- |
|     0 | `BBTC_STATUS_SUCCESS`                     |
|     1 | `BBTC_STATUS_INVALID_ARGUMENT`            |
|     2 | `BBTC_STATUS_NONFINITE_INPUT`             |
|     3 | `BBTC_STATUS_OUTSIDE_DOMAIN`              |
|     4 | `BBTC_STATUS_INCONSISTENT_CONFIGURATION`  |
|     5 | `BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION` |
|     6 | `BBTC_STATUS_INSUFFICIENT_STORAGE`        |
|     7 | `BBTC_STATUS_NUMERICAL_FAILURE`           |
|     8 | `BBTC_STATUS_ITERATION_LIMIT`             |
|     9 | `BBTC_STATUS_INTERNAL_INVARIANT_FAILURE`  |
|    10 | `BBTC_STATUS_NAN_INPUT`                   |

| Value | Status string                              |
| ----: | ------------------------------------------ |
|     0 | `"success"`                                |
|     1 | `"invalid argument"`                       |
|     2 | `"non-finite input"`                       |
|     3 | `"value outside mathematical domain"`      |
|     4 | `"inconsistent geometry or configuration"` |
|     5 | `"unsupported model or option"`            |
|     6 | `"insufficient caller-provided storage"`   |
|     7 | `"numerical failure"`                      |
|     8 | `"iteration or step limit reached"`        |
|     9 | `"internal invariant failure"`             |
|    10 | `"not-a-number input"`                     |

For caller-supplied floating-point values, NaN is classified as
`BBTC_STATUS_NAN_INPUT`, while positive or negative infinity is classified as
`BBTC_STATUS_NONFINITE_INPUT`. Within one validation layer, NaN classification
takes precedence over infinity when both are present. This rule does not reorder
validation layers: required pointer or structural checks and any earlier
component or enclosing-layer validation retain their established precedence.

A nonfinite value produced by BBTC arithmetic from otherwise accepted finite
inputs is not a NaN-input classification. It returns
`BBTC_STATUS_NUMERICAL_FAILURE` or another already documented internal failure
status appropriate to that operation.

Existing values MUST NOT be renumbered, aliased, or reused for another meaning.
`bbtc_status_string()` MUST return `"unknown BBTC status"` for every
unrecognized numeric value. The function MUST NOT return a null pointer,
allocate memory, or modify shared state.

### 9.2 Simulation termination

Internal-ballistics termination is distinct from API status.
`bbtc_ib_termination_e` uses a fixed `uint8_t` underlying representation and
the following stable values:

| Value | Enumerator                                                  |
| ----: | ----------------------------------------------------------- |
|     0 | `BBTC_IB_TERMINATION_NOT_RUN`                               |
|     1 | `BBTC_IB_TERMINATION_MUZZLE_EXIT`                           |
|     2 | `BBTC_IB_TERMINATION_NO_IGNITION`                           |
|     3 | `BBTC_IB_TERMINATION_PROJECTILE_NOT_STARTED`                |
|     4 | `BBTC_IB_TERMINATION_PROJECTILE_STOPPED_BEFORE_MUZZLE_EXIT` |
|     5 | `BBTC_IB_TERMINATION_TIME_GUARD_REACHED`                    |
|     6 | `BBTC_IB_TERMINATION_PRESSURE_GUARD_REACHED`                |
|     7 | `BBTC_IB_TERMINATION_STEP_GUARD_REACHED`                    |
|     8 | `BBTC_IB_TERMINATION_NUMERICAL_FAILURE`                     |

`BBTC_IB_TERMINATION_NOT_RUN` is the zero-initialized sentinel and MUST NOT be
reported as the endpoint of an attempted simulation.

Muzzle exit, no ignition, a projectile that did not start, a projectile that
stopped in the bore, and a caller-requested guard are computed simulation
endpoints. If BBTC produces their required result metadata coherently, the API
status is `BBTC_STATUS_SUCCESS`. `BBTC_IB_TERMINATION_NUMERICAL_FAILURE`
accompanies `BBTC_STATUS_NUMERICAL_FAILURE` when integration began but could
not produce a coherent modeled endpoint.

`BBTC_STATUS_ITERATION_LIMIT` is reserved for an internal iterative operation
whose own limit prevented the requested API operation from completing. It is
not another spelling for a caller-configured time, pressure, or accepted-step
guard.

`bbtc_ib_termination_string()` returns the following immutable, nonlocalized
strings:

| Value | Termination string                                |
| ----: | ------------------------------------------------- |
|     0 | `"simulation not run"`                            |
|     1 | `"projectile reached muzzle exit"`                |
|     2 | `"ignition did not occur"`                        |
|     3 | `"projectile did not begin moving"`               |
|     4 | `"projectile stopped before muzzle exit"`         |
|     5 | `"caller time guard reached"`                     |
|     6 | `"caller pressure guard reached"`                 |
|     7 | `"caller step guard reached"`                     |
|     8 | `"numerical failure"`                             |

Every unrecognized numeric value maps to
`"unknown BBTC internal-ballistics termination"`. The function MUST NOT return
a null pointer, allocate memory, or modify shared state.

### 9.3 Warning flags

Nonfatal computational or reporting conditions use
`bbtc_warning_flags_t`, which is exactly `uint64_t`. Individual bit declarations
use `bbtc_warning_flag_e` with `uint64_t` representation:

| Bit | Value  | Enumerator                                     |
| --: | -----: | ---------------------------------------------- |
|   - | `0x00` | `BBTC_WARNING_NONE`                            |
|   0 | `0x01` | `BBTC_WARNING_HISTORY_TRUNCATED`               |
|   1 | `0x02` | `BBTC_WARNING_ENERGY_RESIDUAL_EXCEEDED`        |
|   2 | `0x04` | `BBTC_WARNING_INCOMPLETE_BURN_AT_MUZZLE_EXIT`  |
|   3 | `0x08` | `BBTC_WARNING_FALLBACK_APPROXIMATION_USED`     |
|   4 | `0x10` | `BBTC_WARNING_REDUCED_EVENT_LOCATION_ACCURACY` |
|   5 | `0x20` | `BBTC_WARNING_DATA_EXTRAPOLATED`               |

Zero means that no warning defined by this contract was reported. It does not
mean that the model is applicable, validated, or safe. Multiple nonzero bits
MAY be combined in one mask. Existing bits MUST NOT be renumbered, aliased, or
reused, and unassigned bits are reserved.

`bbtc_warning_flag_string()` describes `BBTC_WARNING_NONE` or one individual
defined warning bit:

| Value  | Warning string                                    |
| -----: | ------------------------------------------------- |
| `0x00` | `"no warning reported"`                           |
| `0x01` | `"requested history was truncated"`               |
| `0x02` | `"energy-accounting residual tolerance exceeded"` |
| `0x04` | `"propellant burn incomplete at muzzle exit"`     |
| `0x08` | `"fallback approximation used"`                   |
| `0x10` | `"event located with reduced accuracy"`           |
| `0x20` | `"data record extrapolated"`                      |

An unrecognized value or a combination of multiple bits maps to
`"unknown or combined BBTC warning flag"`. The string function is a diagnostic
convenience; the bitmask remains the programmatic contract.

### 9.4 Result-field validity

Results MUST include a validity mask or equivalent structured mechanism. A
field not marked valid MUST NOT be consumed.

Unavailable floating-point result fields SHOULD be initialized to a quiet NaN
of the corresponding scalar family to make accidental use visible. Integer
counters and bitmasks SHOULD be initialized to zero. Callers MUST still consult
status and validity metadata; NaN is a tripwire, not the API.

The public validity representation and bit assignments are deliberately
deferred until concrete result fields are reviewed. Validity bits MUST map
unambiguously to actual result fields or documented field groups; BBTC MUST NOT
publish speculative validity bits for fields that do not yet exist.

A valid field means only that BBTC computed and populated it according to the
selected model and termination path. Field validity does not erase warnings,
establish model applicability, or make a firearm-safety claim.

### 9.5 Model applicability

Computational success **DOES NOT imply scientific validity** for every input.
Model-limit conditions use `bbtc_applicability_flags_t`, which is exactly
`uint64_t`. Individual bit declarations use `bbtc_applicability_flag_e` with
`uint64_t` representation:

| Bit | Value  | Enumerator                                           |
| --: | -----: | ---------------------------------------------------- |
|   - | `0x00` | `BBTC_APPLICABILITY_NONE_REPORTED`                   |
|   0 | `0x01` | `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`      |
|   1 | `0x02` | `BBTC_APPLICABILITY_MODEL_COMBINATION_UNVALIDATED`   |
|   2 | `0x04` | `BBTC_APPLICABILITY_ASSUMPTIONS_MATERIALLY_STRESSED` |
|   3 | `0x08` | `BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN`         |
|   4 | `0x10` | `BBTC_APPLICABILITY_REQUESTED_EFFECT_APPROXIMATED`   |

Zero means that no limitation defined by this contract was reported. It does
not prove that a result is experimentally validated, approved, or safe.
Multiple nonzero bits MAY be combined in one mask. Existing bits MUST NOT be
renumbered, aliased, or reused, and unassigned bits are reserved.

`bbtc_applicability_flag_string()` describes the zero value or one individual
defined applicability bit:

| Value  | Applicability string                                |
| -----: | --------------------------------------------------- |
| `0x00` | `"no applicability limitation reported"`            |
| `0x01` | `"outside documented calibration domain"`           |
| `0x02` | `"model combination lacks experimental validation"` |
| `0x04` | `"model assumptions materially stressed"`           |
| `0x08` | `"data provenance unknown"`                         |
| `0x10` | `"requested physical effect approximated"`          |

An unrecognized value or a combination of multiple bits maps to
`"unknown or combined BBTC applicability flag"`.

Warning and applicability flags are not mutually exclusive. For example, an
extrapolated data record may set both `BBTC_WARNING_DATA_EXTRAPOLATED` and
`BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`. A fallback used for a
requested effect may set both corresponding warning and applicability bits.
Each channel retains its separate meaning.

No flag may be named or documented as `safe`, `unsafe`, `proofed`, or
`approved`.

## 10. Internal-ballistics problem boundary

The public internal-ballistics problem description will be composed from
explicit physical records rather than an unstructured bag of numbers. It must
represent, at minimum:

- cartridge and chamber geometry;
- initial geometric volume behind the projectile;
- propellant charge mass and condensed-phase density;
- projectile mass, initial position, and effective base area;
- bore geometry and travel to the muzzle event;
- propellant thermochemical parameters;
- burn-law parameters and their reference conditions;
- grain geometry and burn-surface evolution;
- ignition initial conditions;
- projectile start, engraving, and bore-resistance behavior;
- initial temperatures;
- enabled model terms.

Ambiguous convenience fields such as `case_volume` MUST be replaced by a name
and definition that states whether the quantity is geometric volume, initial
free-gas volume, water capacity, or another measurement.

Inputs that can be derived from other inputs MUST have one documented source of
truth. If both a primitive value and a derived override are accepted, the
precedence and consistency check MUST be explicit.

### 10.1 Initial geometry record

IB0.3b defines three precision-qualified geometry records:

```c
bbtc_ib_geometry_float_t
bbtc_ib_geometry_double_t
bbtc_ib_geometry_long_double_t
```

Each record contains the same four quantities in its native scalar family:

- `initial_behind_projectile_volume_m3` is the enclosed geometric volume behind
  the projectile at its initial modeled position before subtracting condensed
  propellant volume. It is not water capacity or initial free-gas volume;
- `bore_cross_sectional_area_m2` is the effective area used to increase volume
  as the projectile advances;
- `projectile_effective_base_area_m2` is the effective area used to convert
  modeled projectile-base pressure into axial projectile force; and
- `projectile_travel_to_muzzle_m` is the axial travel from the initial modeled
  projectile reference position to the muzzle-exit event.

Bore cross-sectional area and projectile effective base area are distinct
physical concepts. Validation MUST NOT require them to be numerically equal.
A later composed-problem validator may impose cross-record consistency rules
only after the required projectile and propellant records exist.

The three geometry records are distinct concrete types. The core solver API
MUST NOT replace them with one runtime-tagged union. A higher-level CLI,
configuration loader, or serialization adapter MAY later use a tagged union,
but it must select and call the matching concrete precision-qualified API.

A zero-initialized geometry record is deliberately invalid. BBTC provides no
physical default geometry. Each validator returns:

- `BBTC_STATUS_INVALID_ARGUMENT` for a null record pointer;
- `BBTC_STATUS_NAN_INPUT` when any field is NaN;
- `BBTC_STATUS_NONFINITE_INPUT` when no field is NaN and at least one field is
  positive or negative infinity;
- `BBTC_STATUS_OUTSIDE_DOMAIN` when any finite field is zero or negative; and
- `BBTC_STATUS_SUCCESS` when all four fields are finite and positive.

Validation treats the caller-owned record as immutable and performs no
allocation. `float` and `long double` validation MUST use their native types and
MUST NOT convert through `double`.

Before version 1.0, these records use deliberate source-level evolution rather
than public `struct_size`, version, reserved, or named-padding members. Such
compatibility machinery may be introduced later only with a concrete supported
ABI-evolution contract.

### 10.2 Projectile record

IB0.3c defines three precision-qualified projectile records:

```c
bbtc_ib_projectile_float_t
bbtc_ib_projectile_double_t
bbtc_ib_projectile_long_double_t
```

Each record contains one continuous quantity in its native scalar family:

- `mass_kg` is the total translational inertial mass of the modeled projectile
  assembly accelerated through the bore. It includes every component that
  remains mechanically coupled during the modeled bore travel and excludes
  propellant, cartridge-case, gas, and firearm recoiling mass.

Projectile effective base area, the initial projectile reference position, and
travel to muzzle exit remain geometry concepts defined by section 10.1 and MUST
NOT be duplicated in the projectile record. Initial projectile velocity is not
part of physical projectile identity; it remains deferred to a future initial
state or composed-problem record.

The three projectile records are distinct concrete types. The core solver API
MUST NOT replace them with one runtime-tagged union. A higher-level adapter MAY
store them in a tagged union only when it selects and calls the matching
precision-qualified API.

A zero-initialized projectile record is deliberately invalid. BBTC provides no
physical default projectile. Each validator returns:

- `BBTC_STATUS_INVALID_ARGUMENT` for a null record pointer;
- `BBTC_STATUS_NAN_INPUT` when `mass_kg` is NaN;
- `BBTC_STATUS_NONFINITE_INPUT` when `mass_kg` is positive or negative
  infinity;
- `BBTC_STATUS_OUTSIDE_DOMAIN` when finite `mass_kg` is zero or negative; and
- `BBTC_STATUS_SUCCESS` when `mass_kg` is finite and positive.

Validation treats the caller-owned record as immutable and performs no
allocation. `float` and `long double` validation MUST use their native types and
MUST NOT convert through `double`.

The projectile records follow the pre-1.0 source-level evolution policy from
section 10.1. They contain no public structure-size field, version member,
reserved array, or named padding.


### 10.3 Propellant charge record

IB0.3d defines three precision-qualified propellant-charge records:

```c
bbtc_ib_propellant_charge_float_t
bbtc_ib_propellant_charge_double_t
bbtc_ib_propellant_charge_long_double_t
```

Each record contains the same two continuous quantities in its native scalar
family:

- `charge_mass_kg` is the total initial mass of the modeled propellant charge;
  and
- `condensed_phase_density_kg_per_m3` is the material density of the condensed
  propellant phase, excluding intergranular void space.

Condensed-phase density is not bulk loading density, gravimetric bulk density,
gas density, or a burn-rate parameter. A commercial powder name or broad shape
label does not provide this value by itself.

The condensed material volume is a derived quantity with one source of truth:

```c
condensed_propellant_volume_m3 =
    charge_mass_kg / condensed_phase_density_kg_per_m3
```

A future composed-problem validator will compare that derived material volume
with `initial_behind_projectile_volume_m3` from section 10.1 to establish a
positive initial free-gas volume. IB0.3d does not perform that cross-record
calculation or claim that an individually valid charge fits within any
particular cartridge geometry.

The three propellant-charge records are distinct concrete types. The core
solver API MUST NOT replace them with one runtime-tagged union. A higher-level
adapter MAY store them in a tagged union only when it selects and calls the
matching precision-qualified API.

A zero-initialized propellant-charge record is deliberately invalid. BBTC
provides no physical default charge mass or condensed-phase density. Each
validator returns:

- `BBTC_STATUS_INVALID_ARGUMENT` for a null record pointer;
- `BBTC_STATUS_NAN_INPUT` when either field is NaN;
- `BBTC_STATUS_NONFINITE_INPUT` when neither field is NaN and at least one field
  is positive or negative infinity;
- `BBTC_STATUS_OUTSIDE_DOMAIN` when either finite field is zero or negative; and
- `BBTC_STATUS_SUCCESS` when both fields are finite and positive.

Validation treats the caller-owned record as immutable and performs no
allocation. `float` and `long double` validation MUST use their native types and
MUST NOT convert through `double`.

The propellant-charge records follow the pre-1.0 source-level evolution policy
from section 10.1. They contain no public structure-size field, version member,
reserved array, or named padding.

### 10.4 Composed loading-state record

IB0.3e defines three precision-qualified loading-state records:

```c
bbtc_ib_loading_state_float_t
bbtc_ib_loading_state_double_t
bbtc_ib_loading_state_long_double_t
```

Each record owns one matching-precision geometry, projectile, and
propellant-charge record by value. The composed record introduces no alternate
copy of a primitive field and contains no solver option, integration state, burn
law, thermochemical input, or result.

IB0.3e also defines matching derived-volume records with exactly two outputs:

- `condensed_propellant_volume_m3` is charge mass divided by condensed-phase
  density; and
- `initial_free_gas_volume_m3` is initial behind-projectile geometric volume
  minus condensed propellant volume.

The concrete `bbtc_ib_loading_state_evaluate_*()` functions validate geometry,
projectile, and propellant-charge components in that order and propagate the
first non-success status. A nonnull output record is cleared before any failure
return. Null input or output pointers return `BBTC_STATUS_INVALID_ARGUMENT`.

After component validation, condensed propellant volume MUST be representable
and strictly positive. It MUST also be strictly smaller than
`initial_behind_projectile_volume_m3`. Equality is invalid because it produces
zero initial free-gas volume. A larger condensed volume is invalid because the
primitive records cannot occupy the stated geometry. Derived zero, nonfinite,
or nonpositive volume returns `BBTC_STATUS_OUTSIDE_DOMAIN`.

These checks establish only mathematical and dimensional consistency among the
supplied model inputs. They do not establish safe pressure, safe charge mass,
compatible ammunition, firearm strength, or suitability for real loading.

The three loading-state and derived-volume families remain concrete native
scalar types. They use no runtime-tagged union, implicit conversion through
`double`, dynamic allocation, public size/version fields, reserved arrays, or
named padding.

### 10.5 Initial gas-state record

IB0.3f defines three precision-qualified initial gas-state records:

```c
bbtc_ib_initial_gas_state_float_t
bbtc_ib_initial_gas_state_double_t
bbtc_ib_initial_gas_state_long_double_t
```

Each record contains the same two intensive initial conditions in its native
scalar family:

- `absolute_pressure_pa` is the initial absolute pressure of the free gas behind
  the projectile, in pascals; and
- `temperature_k` is the initial absolute temperature of that gas, in kelvin.

The pressure field is absolute rather than gauge pressure. Neither field is
silently copied from the current ambient atmosphere, and BBTC provides no
physical default. A zero-initialized record is deliberately invalid.

Pressure and temperature are related by a future equation of state, but neither
can be derived from the other alone. Such a derivation additionally requires
quantities such as free-gas volume, gas amount or density, gas composition, and
a selected equation-of-state model. IB0.3f therefore treats pressure and
temperature as explicit caller-supplied initial boundary conditions.

Each concrete validator returns:

- `BBTC_STATUS_INVALID_ARGUMENT` for a null record pointer;
- `BBTC_STATUS_NAN_INPUT` when either field is NaN;
- `BBTC_STATUS_NONFINITE_INPUT` when neither field is NaN and at least one field
  is positive or negative infinity;
- `BBTC_STATUS_OUTSIDE_DOMAIN` when either finite field is zero or negative; and
- `BBTC_STATUS_SUCCESS` when both fields are finite and positive.

Validation treats the caller-owned record as immutable and performs no
allocation. `float` and `long double` validation use their native scalar
families and do not convert through `double`.

Passing validation establishes only the primitive mathematical domain. It does
not establish consistency with a loading-state volume, gas quantity,
composition, equation of state, energy balance, real ammunition, or firearm
safety. The records add no public structure-size field, version member,
reserved array, or named padding.


### 10.6 Calorically perfect Noble-Abel gas-model backend

IB0.3g defines three precision-qualified gas-model records:

```c
bbtc_ib_noble_abel_gas_model_float_t
bbtc_ib_noble_abel_gas_model_double_t
bbtc_ib_noble_abel_gas_model_long_double_t
```

These records define one explicit reduced equation-of-state backend. They do
not make Noble-Abel BBTC's universal or permanent gas model, and the future
solver architecture MUST remain capable of selecting other documented
constitutive backends.

Each record contains the same three constant model parameters in its native
scalar family:

- `specific_gas_constant_j_per_kg_k` is the mixture-specific gas constant `R`,
  in joules per kilogram-kelvin;
- `constant_volume_specific_heat_j_per_kg_k` is the constant-volume specific
  heat `c_v`, in joules per kilogram-kelvin; and
- `covolume_m3_per_kg` is the Noble-Abel specific covolume `b`, in cubic meters
  per kilogram.

The mechanical equation of state is:

```text
p * (V - m * b) = m * R * T
```

where `p` is absolute pressure, `V` is total free-gas geometric volume, `m` is
gas mass, and `T` is absolute temperature. Any mass/volume form of the model
MUST require the available translational volume `V - m * b` to be finite and
strictly positive. The IB0.3i density-form evaluator enforces the equivalent
state condition `1 - b * rho > 0`.

The caloric closure treats `c_v` as constant over the model's documented
applicability domain. Under this calorically perfect closure, constant-pressure
specific heat and heat-capacity ratio are derived quantities:

```text
c_p   = c_v + R
gamma = c_p / c_v
```

Neither `c_p` nor `gamma` is stored as a redundant public input.

A zero covolume is valid and explicitly selects the ideal-gas limit of the
mechanical equation. It is not interpreted as an omitted value. The specific
gas constant and constant-volume specific heat MUST be finite and strictly
positive. Covolume MUST be finite and nonnegative.

The model record does not identify gas composition, propellant chemistry,
combustion-product yield, flame temperature, energy release, parameter
provenance, uncertainty, or calibration limits. Those concerns belong to future
thermochemistry and data-provenance records.

Initial trapped fill gas and propellant combustion-product gas are distinct
conceptual populations. A caller MAY represent either population with this
backend only when it supplies parameters calibrated for that population and
documents the applicable density and temperature domain. BBTC MUST NOT
silently reuse combustion-product parameters for initial air or treat an
effective pseudo-gas as a species-resolved composition.

A future first-order virial backend and higher-fidelity thermochemical
reference models remain explicitly permitted. Reduced-model parameters require
provenance, calibration conditions, and uncertainty metadata before BBTC may
make validated predictive-accuracy claims.

Combining this model with an initial absolute pressure `p`, initial temperature
`T`, and initial free-gas volume `V` will later permit derivation of an initial
gas mass:

```text
m = p * V / (R * T + p * b)
```

IB0.3g records the mathematical source of truth but does not expose that
cross-record evaluator yet.

Each concrete validator returns:

- `BBTC_STATUS_INVALID_ARGUMENT` for a null record pointer;
- `BBTC_STATUS_NAN_INPUT` when any field is NaN;
- `BBTC_STATUS_NONFINITE_INPUT` when no field is NaN and at least one field is
  positive or negative infinity;
- `BBTC_STATUS_OUTSIDE_DOMAIN` when `R` or `c_v` is zero or negative, or when
  `b` is negative; and
- `BBTC_STATUS_SUCCESS` otherwise, including the explicit `b == 0` ideal-gas
  limit.

Validation treats the caller-owned record as immutable and performs no
allocation. Native `float` and `long double` validation does not convert through
`double`. The records contain no public structure-size field, version member,
reserved array, or named padding.


Passing validation establishes only the mathematical parameter domain. It does
not establish calibration validity or a physical prediction error bound.
Numerical convergence error and real-world model/input uncertainty MUST remain
separately reported concepts.



### 10.7 Temperature-dependent first-order density-virial backend

IB0.3h defines three native scalar families for a first-order density-virial
gas-model backend:

```c
bbtc_ib_first_order_virial_gas_model_float_t
bbtc_ib_first_order_virial_gas_model_double_t
bbtc_ib_first_order_virial_gas_model_long_double_t
```

The mechanical equation of state is:

```text
p = rho * R * T * (1 + B(T) * rho)
```

where:

- `p` is absolute pressure, in pascals;
- `rho` is gas density, in kilograms per cubic meter;
- `R` is the mass-specific gas constant, in joules per kilogram-kelvin;
- `T` is absolute temperature, in kelvins; and
- `B(T)` is the mass-specific second density virial coefficient, in cubic
  meters per kilogram.

The ideal-gas limit is the identically zero law `B(T) == 0`. The sign of an
individual coefficient or of `B(T)` is not restricted by record validation.
The IB0.3i state evaluator separately requires both a positive represented
compressibility factor and positive local isothermal mechanical stiffness:

```text
1 + B(T) * rho > 0
1 + 2 * B(T) * rho > 0
```

The second condition is equivalent to
`(partial p / partial rho)_T > 0` for positive `R` and `T`. Passing record
validation therefore does not prove that every possible state is admissible.

#### 10.7.1 Bounded Chebyshev temperature law

Each scalar family owns a temperature-law record and borrows a caller-owned
coefficient array:

```c
bbtc_ib_first_order_virial_temperature_law_float_t
bbtc_ib_first_order_virial_temperature_law_double_t
bbtc_ib_first_order_virial_temperature_law_long_double_t
```

For `N == coefficient_count`, the represented law is:

```text
B(T) = sum(c[k] * T_k(x), k = 0 .. N - 1)
```

No half-weight convention is applied to `c[0]`. `T_k` is the Chebyshev
polynomial of the first kind. Temperature is mapped from the closed represented
interval to the canonical Chebyshev interval by:

```text
x = 2 * (T - T_min) / (T_max - T_min) - 1
```

Thus `T_min` maps to `-1`, `T_max` maps to `+1`, and the midpoint maps to zero.
Every stored coefficient has units of cubic meters per kilogram. A degree-zero
law is valid. A one-element zero law is the explicit ideal-gas limit.

The coefficient pointer is borrowed, not owned. BBTC does not allocate, copy,
modify, resize, retain beyond the call, or free coefficient storage. The caller
MUST keep the array readable and immutable for the duration of validation or
evaluation. A null pointer is invalid even when `coefficient_count` is zero.

The temperature-law validator requires:

- a nonnull law pointer;
- a nonnull coefficient pointer;
- finite temperature bounds;
- `T_min > 0`;
- `T_max > T_min`;
- `coefficient_count > 0`; and
- every coefficient finite.

Coefficient signs are unrestricted.

Within temperature-law validation, a null law or coefficient pointer is an
invalid argument. Across the complete law record and its borrowed coefficient
array, any NaN bound or coefficient returns `BBTC_STATUS_NAN_INPUT`; otherwise
any positive or negative infinity returns `BBTC_STATUS_NONFINITE_INPUT`. Thus a
NaN encountered later in the coefficient array takes precedence over an
infinity elsewhere in that same law-validation layer.

#### 10.7.2 Analytic derivatives

The temperature-law evaluator returns:

```text
B(T)
dB/dT
d^2B/dT^2
```

in a precision-qualified output record. Derivatives are analytic derivatives of
the represented series, not finite-difference estimates.

The implementation uses a backward Clenshaw recurrence. For `N` coefficients:

```text
b_N = b_(N+1) = 0
b_k = 2*x*b_(k+1) - b_(k+2) + c_k, k = N-1 .. 1

B(x) = x*b_1 - b_2 + c_0
```

Differentiating the auxiliary recurrence gives:

```text
b'_k =
    2*b_(k+1) + 2*x*b'_(k+1) - b'_(k+2)

b''_k =
    4*b'_(k+1) + 2*x*b''_(k+1) - b''_(k+2)

dB/dx = b_1 + x*b'_1 - b'_2
d^2B/dx^2 = 2*b'_1 + x*b''_1 - b''_2
```

This evaluates the series and both derivatives in one backward pass with
constant storage. Because the temperature mapping is affine:

```text
dx/dT = 2 / (T_max - T_min)
d^2x/dT^2 = 0

dB/dT = dB/dx * dx/dT
d^2B/dT^2 = d^2B/dx^2 * (dx/dT)^2
```

Evaluation performs no allocation and no conversion through another scalar
family. The output is cleared before any failure that can be reported after a
nonnull output pointer is received. A NaN evaluation temperature returns
`BBTC_STATUS_NAN_INPUT`; positive or negative infinity returns
`BBTC_STATUS_NONFINITE_INPUT`; a finite temperature outside the closed interval
returns `BBTC_STATUS_OUTSIDE_DOMAIN`; and a nonfinite recurrence or derivative
result produced from accepted finite inputs returns
`BBTC_STATUS_NUMERICAL_FAILURE`.

#### 10.7.3 Caloric compatibility boundary

Each gas-model record stores a positive dilute-gas reference constant-volume
specific heat `c_v,0`. This is not silently treated as the complete
finite-density heat capacity when `B` depends on temperature.

For the mechanical EOS in this section, thermodynamic compatibility gives the
specific internal-energy form:

```text
e(rho, T) = e_0(T) - rho * R * T^2 * B'(T)
```

IB0.3i represents the dilute caloric branch by an explicit caller-selected
reference datum:

```text
e_0(T) = e_ref + c_v,0 * (T - T_ref)
```

where `T_ref` is finite and strictly positive and `e_ref` is any finite specific
internal-energy datum. BBTC supplies no hidden reference temperature or
zero-energy convention. This reduced-gas caloric datum does not by itself
define a chemical standard state, species composition, heat of formation, or
reaction energy.

The resulting finite-density constant-volume specific heat is:

```text
c_v(rho, T) =
    c_v,0
    - rho * R * (2*T*B'(T) + T^2*B''(T))
```

IB0.3i evaluates pressure, specific internal energy, finite-density
constant-volume specific heat, `(partial p / partial rho)_T`, and
`(partial p / partial T)_rho`. That constitutive evaluator still does not expose
enthalpy, entropy, sound speed, state inversion, combustion thermochemistry, or
a ballistic integration step. IB0.3j adds the separate initial free-gas
mass/density closure defined in Section 10.9 rather than folding that inversion
into the forward thermodynamic evaluator.


#### 10.7.4 Calibration-domain metadata

Each model record stores a closed calibrated density interval. The minimum
density is finite and nonnegative. The maximum density is finite and strictly
greater than the minimum. The temperature-law interval is the model's
represented temperature interval.

These intervals are applicability metadata, not clipping instructions. IB0.3i
treats the calibrated density interval as a soft scientific-applicability
boundary: a mathematically and thermodynamically admissible state outside that
density interval may still return `BBTC_STATUS_SUCCESS`, but the result MUST set
`BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`. This is not silent clipping and
does not claim validated extrapolative accuracy.

The represented Chebyshev temperature interval is different: it is a hard
evaluation domain for the current backend because BBTC does not extrapolate
`B(T)` beyond the supplied coefficient-law interval. Validation alone does not
establish provenance, experimental agreement, predictive uncertainty, or
safety.

The backend describes an effective pseudo-gas. It does not identify chemical
species, equilibrium composition, condensed products, combustion-product yield,
flame temperature, or parameter-fitting data. Initial trapped fill gas and
propellant combustion products remain distinct gas populations.

IB0.3h does not claim that this parameterization is universally more accurate
than Noble-Abel. It provides a more expressive reduced backend whose accuracy
must be established for a documented gas population and calibration domain.
Noble-Abel remains a supported baseline and comparison backend.


### 10.8 Reduced-gas caloric reference and thermodynamic evaluation

IB0.3i defines three explicit dilute-branch caloric-reference records:

```c
bbtc_ib_caloric_reference_float_t
bbtc_ib_caloric_reference_double_t
bbtc_ib_caloric_reference_long_double_t
```

Each record contains:

- `reference_temperature_k`, a finite strictly positive absolute temperature;
  and
- `reference_specific_internal_energy_j_per_kg`, a finite specific
  internal-energy datum with no sign restriction.

The reference is external to the concrete gas-model record so one constitutive
parameter set is not silently coupled to one arbitrary energy zero. A caller
MAY choose 298.15 K when its data convention requires that temperature, but
BBTC MUST NOT silently supply 298.15 K or describe it as a universal
standard-state temperature.

IB0.3i also defines one common thermodynamic-result record per scalar family:

```c
bbtc_ib_reduced_gas_thermodynamic_result_float_t
bbtc_ib_reduced_gas_thermodynamic_result_double_t
bbtc_ib_reduced_gas_thermodynamic_result_long_double_t
```

Each result contains:

- absolute pressure;
- specific internal energy relative to the supplied caloric datum;
- state constant-volume specific heat;
- `(partial p / partial rho)_T`;
- `(partial p / partial T)_rho`; and
- `bbtc_applicability_flags_t`.

The output record is cleared before any failure that occurs after a nonnull
output pointer is accepted. `rho == 0` is a valid mathematical boundary state;
negative density and nonpositive absolute temperature are outside the domain.
Within the direct density/temperature validation layer, a NaN in either scalar
returns `BBTC_STATUS_NAN_INPUT`; otherwise positive or negative infinity in
either scalar returns `BBTC_STATUS_NONFINITE_INPUT`. NaN therefore takes
precedence over infinity within that two-scalar layer. Earlier model and
caloric-reference validations retain their established precedence. Finite-input
arithmetic that cannot produce finite outputs returns
`BBTC_STATUS_NUMERICAL_FAILURE`.

The Noble-Abel evaluator uses:

```text
p = rho * R * T / (1 - b * rho)

e = e_ref + c_v * (T - T_ref)

c_v,state = c_v

(partial p / partial rho)_T =
    R * T / (1 - b * rho)^2

(partial p / partial T)_rho =
    rho * R / (1 - b * rho)
```

and requires `1 - b * rho > 0`.

The first-order virial evaluator uses the already-defined analytic `B(T)`,
`B'(T)`, and `B''(T)` temperature-law evaluation and does not duplicate the
Chebyshev recurrence:

```text
p = rho * R * T * (1 + B * rho)

e =
    e_ref
    + c_v,0 * (T - T_ref)
    - rho * R * T^2 * B'

c_v,state =
    c_v,0
    - rho * R * (2*T*B' + T^2*B'')

(partial p / partial rho)_T =
    R * T * (1 + 2*B*rho)

(partial p / partial T)_rho =
    rho * R * (1 + B*rho + rho*T*B')
```

It requires positive `1 + B*rho`, positive
`(partial p / partial rho)_T`, and positive finite `c_v,state`.
Density outside the documented calibration interval sets
`BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN` but does not, by itself, change
a successful API status into a hard failure. A temperature outside the
represented coefficient-law interval remains `BBTC_STATUS_OUTSIDE_DOMAIN`.

These constitutive evaluators do not derive gas mass, infer composition,
represent propellant combustion, supply formation energies, integrate the
projectile, establish predictive uncertainty, or make any ammunition/firearm
safety judgment.


### 10.9 Initial free-gas mass/density closure

IB0.3j defines one initial free-gas solution record per scalar family:

```c
bbtc_ib_initial_gas_solution_float_t
bbtc_ib_initial_gas_solution_double_t
bbtc_ib_initial_gas_solution_long_double_t
```

Each record contains:

- `bbtc_applicability_flags_t applicability_flags`;
- density in kilograms per cubic meter; and
- initial free-gas mass in kilograms.

The record describes only the initial free/trapped gas population occupying
the supplied initial free-gas volume. It MUST NOT be interpreted as including
condensed propellant, future propellant-combustion products, projectile mass,
case mass, or firearm mass.

IB0.3j exposes concrete precision-qualified closure functions:

```c
bbtc_ib_noble_abel_initial_gas_solve_float(...)
bbtc_ib_noble_abel_initial_gas_solve_double(...)
bbtc_ib_noble_abel_initial_gas_solve_long_double(...)

bbtc_ib_first_order_virial_initial_gas_solve_float(...)
bbtc_ib_first_order_virial_initial_gas_solve_double(...)
bbtc_ib_first_order_virial_initial_gas_solve_long_double(...)
```

Each closure call consumes:

- one concrete reduced gas-model record;
- one matching precision-qualified initial gas-state record carrying explicit
  absolute pressure `p` and temperature `T`;
- a finite strictly positive initial free-gas volume `V`; and
- a caller-owned output solution record.

The closure layer MUST accept the free-gas volume as the scalar derived
quantity rather than requiring the entire loading-state record. The closure
layer MUST NOT require a caloric-reference record because gas mass/density
closure is a mechanical equation-of-state inversion and does not depend on the
arbitrary specific-internal-energy datum.

For both reduced gas backends, define:

```text
q = p / (R * T)
```

where `R` is the model's mass-specific gas constant. Successful closure
requires `q`, the derived density, and the final gas mass to be representable as
finite strictly positive values in the selected native scalar family.

#### 10.9.1 A+ numerical policy

IB0.3j uses a targeted numerical-hardening policy rather than either naive
direct arithmetic or a general arbitrary-range arithmetic system.

The implementation MUST:

- use numerically stable algebraic forms where an equivalent expression avoids
  cancellation or an unnecessary intermediate range failure;
- evaluate `p / (R*T)` without requiring the direct product `R*T` to be
  representable when the final `q` is representable in the selected scalar
  family;
- retain native `float`, `double`, and `long double` arithmetic and matching
  native math functions for the three scalar families; and
- report `BBTC_STATUS_NUMERICAL_FAILURE` when a required finite result cannot
  be represented reliably in the selected scalar family.

The implementation is not required to recover every mathematically
representable final answer from every pathological combination of finite input
operands near the scalar type's extreme exponent limits. IB0.3j therefore does
not establish arbitrary-range arithmetic as part of the public contract.

#### 10.9.2 Noble-Abel closure

For Noble-Abel:

```text
p = rho * R * T / (1 - b * rho)
```

with specific covolume `b`. Solving for density gives:

```text
rho = p / (R*T + p*b)
    = q / (1 + b*q)
```

and:

```text
m = rho * V
```

`b == 0` is the exact ideal-gas limit `rho == q`.

The implementation MAY select algebraically equivalent reciprocal forms to
avoid unnecessary overflow in `b*q`. A successful result MUST preserve the
strict Noble-Abel excluded-volume interior:

```text
1 - b * rho > 0
```

If native-precision rounding places the derived state on or beyond that
singular boundary, the call MUST fail rather than report the boundary state as
valid.

#### 10.9.3 First-order virial closure

For the first-order density-virial backend:

```text
p = rho * R * T * (1 + B(T) * rho)
```

so closure requires solving:

```text
B(T) * rho^2 + rho - q = 0
```

BBTC MUST select the unique root that:

- is continuous with the ideal-gas limit `B(T) -> 0`; and
- has positive local isothermal mechanical stiffness.

The cancellation-prone expression

```text
(-1 + sqrt(1 + 4*B*q)) / (2*B)
```

MUST NOT be the primary closure formula near the ideal-gas limit.

For `B(T) == 0`, closure is exactly:

```text
rho = q
```

For `B(T) < 0`, the accepted stable branch requires:

```text
B(T) * q > -1/4
```

Equality corresponds to a zero isothermal pressure-density derivative and is a
hard `BBTC_STATUS_OUTSIDE_DOMAIN` boundary. The alternate algebraic root on
the negative-stiffness branch MUST NOT be exposed to callers.

For `B(T) > 0`, the implementation SHOULD avoid forming an otherwise
overflowing `B(T)*q` when the stable density remains representable. The current
reference implementation evaluates the required square-root product through
scaled native arithmetic and uses a stable positive-root form.

After closure, the derived virial density MUST remain on the positive local
isothermal-stiffness branch. Loss of that invariant from native arithmetic is a
numerical failure rather than a second caller-selectable root.

#### 10.9.4 Domain, applicability, and failure semantics

The first-order virial coefficient law MUST still be evaluated only over its
closed represented temperature interval. Temperature outside that interval is
`BBTC_STATUS_OUTSIDE_DOMAIN`; BBTC MUST NOT extrapolate the Chebyshev law
silently.

A successfully derived virial density outside the model's closed calibrated
density interval MUST return `BBTC_STATUS_SUCCESS` with
`BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`. Exact calibrated-density
boundaries are inside the calibration interval and MUST NOT set that flag.

A nonnull solution record MUST be cleared before any subsequent validation or
numerical operation that can fail. Null output pointers return
`BBTC_STATUS_INVALID_ARGUMENT`. Model and initial-gas-state validation retain
their established precedence. At the direct initial-free-gas-volume layer, NaN
returns `BBTC_STATUS_NAN_INPUT`, while positive or negative infinity returns
`BBTC_STATUS_NONFINITE_INPUT`. A zero or negative finite initial free-gas volume
is outside the closure domain.

The closure layer MUST NOT:

- infer the initial gas species or composition;
- substitute combustion-product pseudo-gas parameters for the initial gas
  population;
- derive a caloric energy reference;
- represent ignition or propellant burning;
- generate combustion-product mass;
- integrate projectile motion; or
- make an ammunition/firearm safety judgment.

Callers are responsible for supplying gas-model parameters that actually
describe the initial free-gas population. Initial trapped fill gas and future
propellant combustion products remain distinct modeled populations.


## 11. Propellant representation

"Ball," "flake," "extruded," "single-base," and similar labels are not complete
pressure-model inputs.

BBTC separates:

- chemical family;
- grain geometry;
- grain dimensions;
- condensed-phase density;
- thermochemical properties;
- pressure-dependent burn law;
- temperature sensitivity;
- charge mass;
- ignition conditions;
- provenance and calibration domain.

The solver consumes numerical physical parameters. A shape or chemical-family
enumeration MAY select a documented equation, but it MUST NOT conjure missing
thermochemical or burn-rate values.

The propellant architecture targets conventional gun and small-arms propellants,
including black powder, brown powder, Cordite-family materials, and single-,
double-, or triple-base smokeless propellants when appropriate independently
sourced model parameters exist. Grain geometry and chemical/formulation identity
remain orthogonal: a geometric backend MUST NOT imply a chemistry, and a named
propellant family MUST NOT imply a particular grain geometry. Rocket and missile
solid-propellant propulsion is outside BBTC's intended physical scope.

### 11.1 Reduced propellant thermochemical source

IB0.4a defines one reduced propellant-thermochemistry record per scalar family:

```c
bbtc_ib_propellant_thermochemistry_float_t
bbtc_ib_propellant_thermochemistry_double_t
bbtc_ib_propellant_thermochemistry_long_double_t
```

Each record contains:

- gaseous-product mass fraction `y_g`; and
- effective specific reaction internal-energy release `q_r`, in joules per
  kilogram of reacted propellant.

The model validator MUST require finite values satisfying:

```text
0 < y_g <= 1
q_r > 0
```

Within thermochemistry-record validation, any NaN field returns
`BBTC_STATUS_NAN_INPUT`; otherwise any positive or negative infinity returns
`BBTC_STATUS_NONFINITE_INPUT`. NaN takes precedence over infinity within that
record.

`q_r` is the positive effective decrease in modeled chemical internal energy
made available to a later internal-ballistics energy balance per kilogram of
reacted propellant. It MUST NOT be interpreted automatically as a standard
enthalpy of combustion, flame temperature, propellant force/impetus, gas
specific internal energy, or a caloric-reference datum. IB0.4a defines no hidden
reference temperature, reference pressure, standard-atmosphere state, species
composition, gas equation of state, or calibration data for this coefficient.

IB0.4a also defines one extensive thermochemical-source result record per
scalar family:

```c
bbtc_ib_propellant_thermochemical_source_float_t
bbtc_ib_propellant_thermochemical_source_double_t
bbtc_ib_propellant_thermochemical_source_long_double_t
```

Each result contains:

- generated gaseous-product mass `m_g`, in kilograms;
- generated condensed-product mass `m_c`, in kilograms; and
- released reaction internal energy `Q_r`, in joules.

The source evaluator consumes one matching thermochemistry record, one explicit
reacted propellant mass `m_r`, and one caller-owned output record. It MUST NOT
consume the complete propellant-charge record, grain geometry, pressure,
temperature, ignition state, burn-rate law, gas model, or loading-state record.

The reduced source equations are:

```text
m_g = y_g * m_r
m_c = (1 - y_g) * m_r
Q_r = q_r * m_r
```

These equations partition only the explicitly supplied reacted propellant mass.
They MUST NOT be interpreted as including unreacted propellant, initial trapped
gas, projectile mass, case mass, or any other material population.

Reacted-propellant-mass validation occurs after thermochemistry-record
validation. A NaN reacted mass returns `BBTC_STATUS_NAN_INPUT`; positive or
negative infinity returns `BBTC_STATUS_NONFINITE_INPUT`; and a negative finite
reacted mass is outside the source domain. Exactly zero reacted mass is a valid
identity state and MUST return `BBTC_STATUS_SUCCESS` with an all-zero source
record.

For positive reacted mass:

- generated gaseous-product mass MUST be finite and strictly positive;
- released reaction internal energy MUST be finite and strictly positive;
- when `y_g < 1`, generated condensed-product mass MUST be finite and strictly
  positive; and
- exact `y_g == 1` is the all-gas limit and MUST produce exactly zero
  condensed-product mass.

If a mathematically required positive source term overflows, underflows to zero,
or otherwise becomes nonfinite in the selected native scalar family, the
evaluator MUST return `BBTC_STATUS_NUMERICAL_FAILURE`. The implementation MUST
NOT clamp a failed quantity to a representable boundary.

The source evaluator MUST validate in this order once its output pointer is
known to be nonnull:

1. clear the output record;
2. validate the thermochemistry record;
3. classify reacted-mass NaN or infinity;
4. validate the finite reacted-mass domain;
5. handle the zero-reacted-mass identity;
6. evaluate the three extensive source terms;
7. validate representability and source invariants; and
8. commit the successful result.

This ordering makes failure output deterministic and preserves model-validation
precedence over reacted-mass validation. A null output pointer returns
`BBTC_STATUS_INVALID_ARGUMENT` and cannot be cleared.

Mass conservation is an algebraic model requirement:

```text
m_g + m_c = m_r
```

but the public contract MUST NOT require the two separately rounded native
floating-point result fields to sum bit-for-bit to the original input. Tests and
future conservation diagnostics SHOULD instead use an appropriate
precision-aware residual.

The thermochemical source layer MUST remain separate from the gas constitutive
model. In particular, thermochemistry records MUST NOT embed Noble-Abel,
first-order virial, or future mixture-EOS parameters merely because generated
gaseous product will later require a gas model.

IB0.4a does not assign public applicability flags to thermochemical-source
results because the evaluator receives no pressure, temperature, provenance, or
calibration-domain state against which such flags could be evaluated. A future
temperature-dependent or provenance-bearing thermochemistry model MAY add
explicit applicability metadata without changing the meaning of this
constant-coefficient source contract.

IB0.4a MUST NOT:

- infer chemical species or equilibrium composition;
- determine how much propellant has reacted;
- model grain regression or burn-surface evolution;
- evaluate a pressure-dependent burn law;
- model ignition progression;
- infer or silently reuse initial-fill-gas EOS parameters for combustion
  products;
- define reduced-gas mixture rules;
- calculate chamber pressure or temperature;
- integrate the coupled internal-ballistics state;
- move the projectile; or
- make an ammunition/firearm safety judgment.

Generated combustion-product gas and the initial trapped/free-gas population
therefore remain distinct modeled populations. A later mixture contract must
define how those populations share a chamber state; IB0.4a does not silently
collapse them into one pseudo-gas.

### 11.2 Canonical propellant grain regression geometry

IB0.4b defines chemically agnostic individual-grain geometry backends for four
canonical analytical families:

- spherical/ball grains;
- solid finite cylinders, including rod, cord, and strand approximations;
- rectangular prisms, including flake, strip, and simple prismatic
  approximations; and
- single-perforated finite cylinders, including simple tubular grains.

Each geometry family has native `float`, `double`, and `long double` record
families and concrete precision-qualified validators/evaluators. The public API
MUST NOT use a geometry enumeration, tagged union, runtime function-pointer
dispatch layer, or chemical-family identifier to hide the concrete geometry
being evaluated in IB0.4b.

IB0.4b evaluates one canonical grain at an explicitly supplied uniform normal
surface-regression distance `s`, in meters. Regression distance is a geometric
coordinate, not time, reacted mass, burn fraction, or burn rate. IB0.4b does not
define `ds/dt`.

The common native-precision grain-state result contains:

```text
remaining_volume_m3
burning_surface_area_m2
remaining_regression_to_burnout_m
consumed_volume_fraction
```

`remaining_regression_to_burnout_m` is the additional uniform normal regression
distance required to reach the geometry's first burnout condition. It is not a
generic physical web thickness; a physical dimension can be consumed by two
opposing burning surfaces while the remaining regression coordinate advances by
only half that dimension.

All geometrically exposed surfaces in the four IB0.4b analytical backends are
burning surfaces. Surface inhibition, deterrent-layer kinetics, spatially
nonuniform ignition, erosive burning, grain fracture, grain collision/migration,
and coating diffusion are outside this checkpoint.

For a spherical grain with initial radius `r0`:

```text
r(s) = r0 - s
s_max = r0
V(s) = (4/3) * pi * r(s)^3
A_b(s) = 4 * pi * r(s)^2
```

For a solid finite cylindrical grain with initial radius `r0` and length `L0`:

```text
r(s) = r0 - s
L(s) = L0 - 2*s
s_max = min(r0, L0/2)
V(s) = pi * r(s)^2 * L(s)
A_b(s) = 2*pi*r(s)*L(s) + 2*pi*r(s)^2
```

For a rectangular-prismatic grain with initial dimensions `L0`, `W0`, and
`H0`:

```text
L(s) = L0 - 2*s
W(s) = W0 - 2*s
H(s) = H0 - 2*s
s_max = 0.5 * min(L0, W0, H0)
V(s) = L(s) * W(s) * H(s)
A_b(s) = 2 * (L(s)*W(s) + L(s)*H(s) + W(s)*H(s))
```

For a single-perforated finite cylinder with initial outer radius `R0`, initial
inner radius `r0`, and initial length `L0`:

```text
R(s) = R0 - s
r(s) = r0 + s
L(s) = L0 - 2*s
s_max = min((R0 - r0)/2, L0/2)
V(s) = pi * (R(s)^2 - r(s)^2) * L(s)
A_b(s) = 2*pi*(R(s) + r(s))*L(s)
         + 2*pi*(R(s)^2 - r(s)^2)
```

The evaluator-domain policy is common to all four backends:

```text
s < 0       -> BBTC_STATUS_OUTSIDE_DOMAIN
s == 0      -> valid initial grain state
0 < s < max -> valid active grain state
s == s_max  -> valid exact-burnout state
s > s_max   -> BBTC_STATUS_OUTSIDE_DOMAIN
```

Exact burnout MUST return zero remaining volume, zero burning-surface area,
zero remaining regression to burnout, and consumed-volume fraction exactly one.
The implementation MUST NOT clamp geometric overshoot to burnout.

Once a nonnull output pointer has been established, the evaluator MUST clear the
result before model or regression validation so later failure leaves a
deterministic zero record. Geometry validation precedes regression-scalar
validation. Geometry NaN or infinity is classified by the geometry validator.
At the later regression-scalar layer, NaN returns `BBTC_STATUS_NAN_INPUT` and
positive or negative infinity returns `BBTC_STATUS_NONFINITE_INPUT`. Finite
nonpositive required initial dimensions and negative or overshooting regression
use the appropriate finite-domain status.

A mathematically required positive derived geometry quantity that cannot remain
positive and finite in the selected scalar family is a numerical failure. In
particular, underflow of a positive half-length, half-thickness, or half-web
used to define `s_max` MUST NOT be mistaken for an exact-burnout state.
Likewise, required positive interior volume or burning area that overflows,
underflows to zero, or becomes nonfinite is `BBTC_STATUS_NUMERICAL_FAILURE`.
The evaluator MUST NOT clamp a failed quantity to a representable boundary.

The consumed-volume fraction is a dimensionless geometric diagnostic. The
implementation SHOULD prefer algebraically stable forms that avoid unnecessary
subtraction of nearly equal initial and remaining volumes when regression is
small. Successful active states require a finite fraction in the closed interval
`[0, 1]`, with exact initial state zero and exact burnout one.

IB0.4b deliberately models one canonical grain and does not multiply by grain
count, infer a size distribution, consume charge mass or condensed-phase
density, or calculate reacted-mass rate. Multi-perforated grains with topology
changes, irregular/corned-grain empirical form functions, inhibited-surface
masks, and grain-population distributions are reserved for later explicit
extensions.

IB0.4b MUST NOT:

- identify chemical formulation or propellant product;
- compute pressure- or temperature-dependent regression rate;
- consume initial propellant temperature;
- determine reacted propellant mass or reacted-mass rate;
- call the IB0.4a thermochemical source evaluator;
- generate gas or reaction energy;
- model ignition progression;
- evolve chamber pressure or temperature;
- integrate projectile motion; or
- make an ammunition/firearm safety judgment.


### 11.3 Initial propellant condition

IB0.4c defines one explicit initial condensed-propellant condition record per
native scalar family:

```c
bbtc_ib_initial_propellant_condition_float_t
bbtc_ib_initial_propellant_condition_double_t
bbtc_ib_initial_propellant_condition_long_double_t
```

Each record contains exactly one continuous quantity: `temperature_k`, the
initial absolute temperature of the condensed propellant charge in kelvins.
It is not implicitly ambient-air, cartridge-case, chamber, initial free-gas,
or combustion-product temperature. A caller MAY supply equal numeric values
when its modeled setup establishes thermal equilibrium, but BBTC MUST NOT
silently alias or copy one population's temperature into another.

The word "condition" is deliberate. This record establishes an initial thermal
boundary supplied to later propellant models; it does not promise that
propellant temperature remains constant during firing and does not define a
thermal state-evolution equation. Future temperature-sensitive kinetics MAY
consume the initial value directly, or a future thermal model MAY evolve from
it. IB0.4c promises neither behavior.

A zero-initialized record is deliberately invalid. Each validator returns:

- `BBTC_STATUS_INVALID_ARGUMENT` for a null record pointer;
- `BBTC_STATUS_NAN_INPUT` when `temperature_k` is NaN;
- `BBTC_STATUS_NONFINITE_INPUT` for positive or negative infinity;
- `BBTC_STATUS_OUTSIDE_DOMAIN` when finite `temperature_k <= 0`; and
- `BBTC_STATUS_SUCCESS` for every finite `temperature_k > 0`.

The validator imposes no arbitrary 200--400 K band and no universal finite
upper-temperature cap. Positive finite values near the scalar family's lower
representable range are structurally valid. Validation establishes only a
well-formed absolute Kelvin boundary; it does not establish material survival,
chemical stability, cook-off margin, ignition behavior, burn-rate
applicability, calibration validity, ammunition compatibility, firearm
strength, or firing safety.

The record remains separate from propellant charge. Charge records own initial
mass and condensed-phase density; this condition record owns the initial
condensed-propellant thermal boundary. It adds no applicability flags,
provenance fields, hidden corrections, age, lot, storage history, stabilizer
condition, moisture/volatile content, burn coefficients, or state evolution.

Additional environmental or provenance descriptors MAY be represented when a
model can consume them physically. Initial free-gas absolute pressure and
temperature remain owned by the initial gas-state contract. Relative humidity
belongs to a future gas-composition/moisture model; age, lot, storage history,
stabilizer condition, moisture/volatile content, and similar provenance data
MUST NOT act as undocumented correction factors.

### 11.4 Empirical propellant burn kinetics

IB0.4d establishes the engineering boundary for empirical propellant
surface-regression kinetics. A burn-rate backend returns the linear normal
regression rate of an already burning propellant surface:

```text
burn_rate_m_per_s = ds/dt
```

The regression coordinate `s` is the same geometric coordinate consumed by the
IB0.4b grain-regression evaluators. Burn rate is not reacted-propellant mass
rate, burn fraction, burning-surface area, ignition progression, gas-generation
rate, or projectile velocity.

Grain geometry and burn kinetics remain orthogonal. Geometry determines
`A_b(s)`; kinetics determines `ds/dt`. A later charge-coupling layer may combine
them with condensed-phase density and effective whole-charge burning area to
derive reacted-propellant mass rate. A burn-law record MUST NOT silently embed
grain shape merely because experimental calibration may have depended on a
particular test specimen or grain configuration.

IB0.4d defines two initial concrete pressure-only empirical backend families:

1. a normalized pressure-power backend in the Saint-Robert/Vieille family; and
2. a tabulated pressure-to-linear-regression-rate backend.

Neither backend is universally preferred. The pressure-power representation is
compact and appropriate when one pressure exponent adequately represents the
calibrated data. The tabulated representation preserves measured curvature or
pressure-regime changes when a single exponent is not adequate.

Backend selection MUST be explicit. The primitive physics API MUST NOT silently
select a tabulated backend because a table happens to exist, silently fall back
to a pressure-power backend because data are missing, or otherwise change the
physical model based on hidden availability state. A higher-level catalog or
configuration layer MAY implement an explicit documented selection policy and
MUST make the selected backend discoverable.

All concrete burn-kinetics evaluators share one semantic native-precision result
contract containing:

```text
applicability_flags
burn_rate_m_per_s
```

`burn_rate_m_per_s` is the evaluated linear normal surface-regression rate.
`applicability_flags` reports nonfatal limitations on scientific interpretation
and is independent of operation status.

A null result pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. Once a nonnull
result pointer has been accepted, the evaluator MUST clear the complete result
before model validation or direct-state validation that can subsequently fail.
Every later failure therefore leaves a deterministic zero result.

Successful evaluation at a physical state for which the mathematical model
requires a strictly positive burn rate MUST return a finite strictly positive
native value. A zero burn rate is successful only when the concrete backend
defines an exact mathematical zero-rate boundary. Caller-owned model and input
records MUST remain unchanged.

The exact public result-type names and declaration layout are frozen when the
first concrete kinetics API is introduced; this section freezes their physical,
diagnostic, and failure-output semantics.

IB0.4d-B1 freezes the first concrete public kinetics declarations in
`<bbtc/internal_ballistics/propellant_burn_kinetics.h>`. The normalized
pressure-power model types are:

```c
bbtc_ib_pressure_power_burn_kinetics_float_t
bbtc_ib_pressure_power_burn_kinetics_double_t
bbtc_ib_pressure_power_burn_kinetics_long_double_t
```

The common result types, intentionally reusable by later empirical kinetics
backends, are:

```c
bbtc_ib_propellant_burn_kinetics_result_float_t
bbtc_ib_propellant_burn_kinetics_result_double_t
bbtc_ib_propellant_burn_kinetics_result_long_double_t
```

The first concrete validator and evaluator families are:

```c
bbtc_ib_pressure_power_burn_kinetics_validate_float(...)
bbtc_ib_pressure_power_burn_kinetics_validate_double(...)
bbtc_ib_pressure_power_burn_kinetics_validate_long_double(...)

bbtc_ib_pressure_power_burn_kinetics_evaluate_float(...)
bbtc_ib_pressure_power_burn_kinetics_evaluate_double(...)
bbtc_ib_pressure_power_burn_kinetics_evaluate_long_double(...)
```

Each pressure-power model record contains exactly the five continuous quantities
already defined by section 11.4.1 in its native scalar family. Each common result
contains exactly `bbtc_applicability_flags_t applicability_flags` followed by
one native `burn_rate_m_per_s` scalar. No exact structure-size equality is
promised; ordinary implementation padding remains permitted.

#### 11.4.1 Normalized pressure-power backend

The normalized pressure-power relation is:

```text
r(P) = r_ref * (P / P_ref)^n
```

where:

```text
r(P)   = linear normal surface-regression rate, in m/s
r_ref  = reference burn rate, in m/s
P      = supplied absolute pressure, in Pa
P_ref  = explicit reference absolute pressure, in Pa
n      = positive dimensionless pressure exponent
```

The normalized representation is algebraically equivalent to a conventional
Saint-Robert/Vieille relation `r = a*P^n`, but avoids exposing a coefficient
whose dimensions change with `n`. At `P == P_ref`, the evaluator MUST return
`r_ref`.

The initial native scalar-family pressure-power model contract contains these
continuous quantities:

```text
reference_burn_rate_m_per_s
reference_pressure_pa
pressure_exponent
minimum_calibrated_pressure_pa
maximum_calibrated_pressure_pa
```

A concrete pressure-power record MUST expose these quantities without changing
their physical meaning or units.

The reference burn rate, reference pressure, and calibration pressures MUST be
finite and strictly positive. The pressure exponent MUST be finite and strictly
positive for this concrete monotonic pressure-power backend. The maximum
calibrated pressure MUST be strictly greater than the minimum, and
`reference_pressure_pa` MUST lie inside the inclusive calibration interval.
Any finite violation of these positivity, ordering, or containment requirements
returns `BBTC_STATUS_OUTSIDE_DOMAIN`.

These requirements define this concrete backend; they do not assert that every
possible propellant burn model has a positive constant exponent. Plateau, mesa,
multi-regime, pressure-independent, transient, erosive, or other kinetics
require an explicitly different representation when supported.

Within one pressure-power record, NaN in any scalar field returns
`BBTC_STATUS_NAN_INPUT`; otherwise infinity in any field returns
`BBTC_STATUS_NONFINITE_INPUT` before finite-domain relationships are tested.
Zero initialization is deliberately invalid.

The evaluator consumes absolute pressure. Gauge pressure MUST NOT be accepted by
an ambiguously named scalar. A null model pointer returns
`BBTC_STATUS_INVALID_ARGUMENT`. Model validation precedes direct pressure
validation.

For the direct pressure argument:

```text
NaN          -> BBTC_STATUS_NAN_INPUT
+Inf / -Inf  -> BBTC_STATUS_NONFINITE_INPUT
P < 0        -> BBTC_STATUS_OUTSIDE_DOMAIN
P == 0       -> valid mathematical boundary with burn rate exactly zero
P > 0        -> evaluate the pressure-power relation
```

Once a nonnull result pointer has been accepted and cleared, evaluator ordering
is:

1. validate the pressure-power model;
2. classify direct-pressure NaN or infinity;
3. validate the finite direct-pressure mathematical domain;
4. handle the exact zero-pressure boundary;
5. evaluate the positive-pressure relation;
6. classify calibration-domain applicability; and
7. commit the successful result.

This ordering preserves model-validation precedence over the later direct
pressure argument.

`P == 0` does not represent an ignition model or assert that a real propellant
can sustain combustion at vacuum. It is only the exact zero-pressure boundary
of this mathematical backend.

Any mathematically valid pressure below
`minimum_calibrated_pressure_pa` or above
`maximum_calibrated_pressure_pa` remains evaluable. This includes `P == 0`,
because the required minimum calibrated pressure is strictly positive. A
successful result outside the inclusive calibration interval MUST set
`BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`; the evaluator MUST NOT clip the
pressure to the calibration interval. The two calibration endpoints themselves
are inside the interval.

For positive-pressure evaluation, the implementation SHOULD use algebraically
equivalent or exponent-safe forms when they avoid unnecessary intermediate
overflow, underflow, or loss of significance while preserving the normalized
pressure-power relation and exact boundary behavior. This requirement does not
promise arbitrary-range arithmetic beyond the selected native scalar family.

Because the exact mathematical result is strictly positive for accepted
`P > 0`, accepted finite inputs that cannot produce the required finite positive
native result MUST return `BBTC_STATUS_NUMERICAL_FAILURE`. A numerical failure
MUST NOT be misreported as physical zero burning.

#### 11.4.2 Tabulated pressure backend

The tabulated backend represents measured or otherwise externally calibrated
pairs:

```text
(P_i, r_i)
```

where each `P_i` is absolute pressure in pascals and each `r_i` is positive
linear surface-regression rate in meters per second.

IB0.4d-B2 freezes the concrete public names for the tabulated backend in
`<bbtc/internal_ballistics/propellant_burn_kinetics.h>`. The point types are:

```c
bbtc_ib_pressure_burn_point_float_t
bbtc_ib_pressure_burn_point_double_t
bbtc_ib_pressure_burn_point_long_double_t
```

The borrowed table-model types are:

```c
bbtc_ib_pressure_burn_table_float_t
bbtc_ib_pressure_burn_table_double_t
bbtc_ib_pressure_burn_table_long_double_t
```

The corresponding validator and evaluator families are:

```c
bbtc_ib_pressure_burn_table_validate_float(...)
bbtc_ib_pressure_burn_table_validate_double(...)
bbtc_ib_pressure_burn_table_validate_long_double(...)

bbtc_ib_pressure_burn_table_evaluate_float(...)
bbtc_ib_pressure_burn_table_evaluate_double(...)
bbtc_ib_pressure_burn_table_evaluate_long_double(...)
```

Each point record contains one native `pressure_pa` and one native
`burn_rate_m_per_s`. Each table record contains a borrowed immutable pointer to
the matching point type plus a `size_t point_count`. These names deliberately use
`pressure_burn_table` rather than the longer
`tabulated_pressure_burn_kinetics` phrase while preserving the independent
variable, modeled behavior, and representation in the public symbol.

The native scalar-family table model contains a borrowed immutable point-storage
reference and an explicit point count. Point storage remains caller-owned for the
lifetime of every validation or evaluation that consumes the model. The library
MUST NOT require allocation or make an undocumented copy of the point set.

A null table-model pointer or null point-storage reference returns
`BBTC_STATUS_INVALID_ARGUMENT`. A table with fewer than two points returns
`BBTC_STATUS_OUTSIDE_DOMAIN`.

Every pressure and burn-rate value MUST be finite and strictly positive, and
pressures MUST be strictly increasing. After NaN and infinity classification,
a finite nonpositive pressure, finite nonpositive burn rate, or failure of the
strictly increasing pressure ordering returns `BBTC_STATUS_OUTSIDE_DOMAIN`.
Burn-rate values are not required to be monotonic; the table may therefore
represent curvature, plateau-like behavior, mesa-like behavior, or changing
local pressure exponents when supported by the supplied data.

Structural validation of the model pointer, point-storage reference, and count
precedes scalar-data validation. Within the tabulated scalar-data layer, any NaN
in the complete point set takes precedence over any infinity; infinity takes
precedence over finite-domain and ordering failures.

The represented pressure domain is the closed interval from the first pressure
knot through the last pressure knot. IB0.4d deliberately performs no tabulated
pressure extrapolation. A finite pressure outside that represented interval
returns `BBTC_STATUS_OUTSIDE_DOMAIN` rather than extending the first or last
segment beyond measured support.

Once a nonnull result pointer has been accepted and cleared, tabulated
evaluation validates the complete table model before classifying the later
direct pressure argument. Direct-pressure NaN returns `BBTC_STATUS_NAN_INPUT`;
positive or negative infinity returns `BBTC_STATUS_NONFINITE_INPUT`; a finite
pressure outside the represented interval returns
`BBTC_STATUS_OUTSIDE_DOMAIN`.

Evaluation at an exact knot MUST return the corresponding stored burn rate.
Between adjacent knots, the initial interpolation contract is mathematically
piecewise linear in log-pressure/log-burn-rate space:

```text
x = ln(P / P0) / ln(P1 / P0)
r = r0 * exp(x * ln(r1 / r0))
```

Every logarithm therefore acts on a dimensionless positive ratio. These
equations define the interpolation relation, not a mandatory floating-point
operation sequence. The implementation SHOULD use algebraically equivalent
forms, including forms based on `log1p`-style evaluation where useful, when they
reduce cancellation, avoid unnecessary intermediate range loss, or otherwise
improve native-precision robustness while preserving the same mathematical
interpolation and exact-knot behavior.

The interpolation is equivalent to a local pressure-power relation within each
interval while allowing the effective pressure exponent to vary from interval
to interval. It MUST preserve positive burn rate and MUST NOT replace the
specified piecewise log/log relation with a higher-order unconstrained spline
that can overshoot between knots.

Accepted finite inputs that cannot produce the required finite positive native
result return `BBTC_STATUS_NUMERICAL_FAILURE`. The evaluator MUST NOT clamp a
failed result or silently substitute a neighboring knot.

The tabulated backend preserves the supplied empirical curve more directly than
a single pressure-power fit, but it does not remove measurement uncertainty,
test-fixture dependence, temperature dependence, lot variation, or other model
limitations. More tabulated points do not imply more physical certainty than the
measurements support.

#### 11.4.3 Temperature dependence

Initial condensed-propellant temperature is a physically distinct input owned by
the IB0.4c initial-propellant-condition record. Pressure-only kinetics MUST NOT
silently copy ambient temperature, initial free-gas temperature, chamber-wall
temperature, or another population's temperature into that record.

A pressure-only `r(P)` backend MUST NOT accept a temperature argument and then
quietly ignore it. Conversely, a temperature-aware backend MUST NOT alter burn
rate from temperature without an explicit calibrated temperature-response model.

A pressure-only model or catalog record MAY preserve the conditioning or
reference temperature at which its coefficients or table were measured as
provenance. That temperature metadata does not itself modify the evaluated burn
rate and MUST NOT be presented as a temperature correction. A higher-level
validation layer MAY use it to diagnose a mismatch between simulation conditions
and source-data conditions.

A later temperature-aware kinetics backend may represent, for example:

- a documented compact pressure/temperature response law;
- temperature-indexed pressure-power fits; or
- a tabulated pressure/temperature-to-regression-rate surface.

The concrete representation MUST state its interpolation or response equation,
temperature calibration domain, source data, and behavior outside that domain.
When a selected kinetics backend requires initial propellant temperature, it
MUST consume the matching native-precision IB0.4c initial-propellant-condition
record.

The existence of an initial propellant temperature therefore does not itself
define a temperature correction. Temperature sensitivity is empirical
propellant data and MUST NOT be invented from a generic constant.

The solver or result metadata MUST make it possible to determine whether the
selected burn-kinetics backend was temperature-aware. A pressure-only model MUST
NOT be presented as having accounted for initial-temperature sensitivity merely
because the surrounding simulation contains an initial propellant temperature.

#### 11.4.4 Calibration, provenance, and interpretation

Relative-quickness charts, marketing powder names, published charge tables,
muzzle velocity, and peak chamber pressure are not direct substitutes for
calibrated linear surface-regression-rate data.

A catalog layer MAY map a named propellant product or formulation to a versioned
kinetics record, but that mapping SHOULD preserve, when known:

- source and test method;
- propellant lot or formulation identity;
- conditioning temperature;
- pressure domain;
- measurement uncertainty or repeatability;
- raw tabulated data or derivation method;
- fitted-model residuals or other fit-quality information; and
- record revision.

A numerically precise evaluation does not imply equally precise physical
knowledge. Model-fit error, measurement uncertainty, test-fixture differences,
and firing-environment differences remain part of the engineering error budget.

IB0.4d burn-kinetics backends MUST NOT:

- ignite propellant or determine ignition progression;
- calculate burning surface from grain geometry;
- infer whole-charge grain count or grain-size distribution;
- calculate reacted-propellant mass rate by themselves;
- apply undocumented temperature, deterrent, erosive, or aging corrections;
- generate combustion-product mass or reaction energy;
- evolve chamber pressure or gas temperature;
- move the projectile;
- select a named commercial powder from relative quickness; or
- make an ammunition/firearm safety judgment.

### 11.5 Propellant regression-to-mass coupling

IB0.4e introduces the engineering layer that maps one canonical grain's
regression state and an already-evaluated linear surface-regression rate into
whole-charge condensed-propellant volume, burning area, reacted mass, and
reacted-mass rate. This layer connects the IB0.4b grain-geometry contract to the
IB0.4d burn-kinetics contract without making either subsystem responsible for
the other's physics.

Regression distance remains the authoritative burn-progress state. IB0.4e does
not introduce independently integrated reacted mass as a second competing
progress coordinate. A later coupled solver advances the regression coordinate
through `ds/dt`; grain geometry determines the corresponding geometric state;
and the mass-coupling layer derives whole-charge mass quantities from that
state. This ownership prevents numerical drift between an independently
integrated mass fraction and the geometry that is supposed to represent the
same propellant.

#### 11.5.1 Required inputs and physical interpretation

The initial coupling primitive consumes matching native-precision forms of:

- one validated propellant-charge record containing initial charge mass `m0` and
  constant condensed-phase material density `rho_p`;
- one explicit positive initial canonical-grain volume `V_g0`, in cubic meters;
- one current canonical-grain state produced by the IB0.4b geometry semantics;
  and
- one common IB0.4d burn-kinetics result containing linear normal
  surface-regression rate `r = ds/dt` and its applicability flags.

The initial single-grain volume is supplied explicitly rather than reconstructed
from current remaining volume and consumed-volume fraction. A caller may obtain
`V_g0` once by evaluating the selected canonical geometry at exact regression
distance `s == 0` and may cache it as an invariant for later coupling calls.
IB0.4e MUST NOT reconstruct `V_g0` from a quotient such as
`V_g / (1 - f)` because that becomes poorly conditioned near burnout and is
undefined at exact burnout.

The coupling primitive is geometry-backend-neutral. It receives the common
grain-state quantities and does not contain a geometry enumeration, tagged
union, runtime function-pointer dispatch table, or geometry-specific formula.
Likewise, it receives the common burn-kinetics result and does not know whether
`r` came from the normalized pressure-power backend, the pressure-burn table,
or a future compatible kinetics backend.

For the initial IB0.4e model, condensed propellant material density is treated
as constant during regression. Thermal expansion, porosity evolution,
decomposition-driven density change, swelling, fracture, and other
density-changing effects are outside this increment.

#### 11.5.2 Equivalent population scale

IB0.4e represents the complete charge as a real-valued equivalent population of
identical canonical grains sharing one regression state. Define the initial
condensed charge volume

```text
V_charge0 = m0 / rho_p
```

and the dimensionless equivalent population scale

```text
N_eq = V_charge0 / V_g0
     = m0 / (rho_p * V_g0)
```

`N_eq` is a scaling factor, not a literal integer grain count. It MUST be
permitted to take any finite strictly positive representable value, including a
noninteger value or a value below one. No rounding to an integer grain count is
permitted. A value such as `12543.8` means that the canonical grain geometry is
being used as an equivalent monodisperse basis for the supplied charge; it does
not assert that the physical charge contains a fractional grain.

The represented population assumes that every equivalent grain has the same
canonical geometry parameters, the same current regression coordinate, and the
same instantaneous normal regression rate. This is a synchronous,
monodisperse-equivalent population approximation. It does not assert that real
manufactured grains are identical or that real ignition occurs simultaneously
throughout the charge.

Grain-size distributions, mixed geometry populations, explicit integer grain
counts, partial population ignition, grain fracture, grain migration, and
multiple simultaneously active regression coordinates require later explicit
population models rather than hidden corrections to `N_eq`.

#### 11.5.3 Whole-charge geometric and mass quantities

Let the current single-grain state provide:

```text
V_g = remaining_volume_m3
A_g = burning_surface_area_m2
f   = consumed_volume_fraction
```

The whole-charge remaining condensed-propellant volume is

```text
V_remaining = N_eq * V_g
```

and total burning surface area is

```text
A_total = N_eq * A_g
```

The mass state is

```text
m_remaining = m0 * (V_g / V_g0)
m_reacted   = m0 * f
```

IB0.4e-B1 freezes the concrete native result families as:

```text
bbtc_ib_propellant_mass_result_float_t
bbtc_ib_propellant_mass_result_double_t
bbtc_ib_propellant_mass_result_long_double_t
```

Each record exposes these fields:

```text
applicability_flags
equivalent_population_scale
remaining_volume_m3
burning_surface_area_m2
remaining_mass_kg
reacted_mass_kg
reacted_mass_rate_kg_per_s
```

The shorter field names are intentional. The enclosing `bbtc_ib_propellant_mass_*`
type already establishes that these are whole-charge propellant quantities, so
repeating `propellant` or `total` in every member would add length without adding
physical meaning. The units and semantics remain those frozen by IB0.4e-A.

`remaining_volume_m3` describes only the modeled condensed
propellant population represented by this coupling primitive. It is not chamber
free volume, gas volume, cartridge-case volume, or projectile-displacement
volume. A later chamber-state layer may use this quantity when constructing the
evolving free volume.

`burning_surface_area_m2` is an explicit whole-charge diagnostic and coupling
output.
It is useful both for physical interpretation and for later verification of the
mass-rate calculation. It does not include inhibited, unignited, fractured, or
otherwise unrepresented surfaces unless a later model explicitly introduces
those effects.

#### 11.5.4 Reacted-propellant mass rate

For a current linear normal surface-regression rate

```text
r = ds/dt
```

the whole-charge reacted-propellant mass rate is mathematically

```text
dm_reacted/dt = rho_p * A_total * r
```

or, after substitution of the equivalent population scale,

```text
dm_reacted/dt = m0 * (A_g / V_g0) * r
```

These equations define the physical relation, not a mandatory floating-point
operation sequence. An implementation SHOULD choose an algebraically equivalent
native-precision form that avoids unnecessary intermediate overflow, underflow,
or cancellation for the supplied values.

The coupling evaluator does not itself evaluate pressure, temperature, or a burn
law. It consumes an already evaluated `r`. A finite exact zero burn rate is a
valid state and produces reacted-propellant mass rate exactly zero. A finite
negative burn rate returns `BBTC_STATUS_OUTSIDE_DOMAIN`.

At exact geometric burnout, total burning area and reacted-propellant mass rate
are exactly zero regardless of a positive supplied kinetics rate. The geometry
has no represented burning surface left on which that rate can act. IB0.4e MUST
NOT reinterpret a positive post-burnout kinetics value as continued mass
consumption.

#### 11.5.5 Endpoint semantics

The IB0.4b grain-state contract defines exact initial and exact burnout states.
IB0.4e preserves those semantic boundaries rather than reconstructing them
through subtraction of nearly equal floating-point values.

At exact initial grain state:

```text
m_reacted   = 0
m_remaining = m0
V_remaining = V_charge0
```

The initial total burning area remains the equivalent population scale times
the initial single-grain burning area. The initial reacted-mass rate is therefore
zero only when the supplied burn rate is zero; an already-burning initial
surface with positive `r` may have a positive instantaneous mass rate at
`s == 0`.

At exact burnout:

```text
V_remaining       = 0
A_total           = 0
m_remaining       = 0
m_reacted         = m0
dm_reacted/dt     = 0
```

These endpoint identities are exact software semantics. They do not imply that a
real propellant charge burns with perfect simultaneity or reaches a physically
sharp global burnout event.

For interior states, BBTC does not require the independently stable evaluations

```text
m_remaining + m_reacted
```

to equal `m0` bit-for-bit. The quantities MUST remain mathematically consistent
with the coupling contract to native rounding, but the implementation SHOULD
prefer stable direct forms over forcing an exact floating-point complement.
In particular, small reacted mass SHOULD be derived from the consumed-volume
fraction rather than from subtracting two nearly equal masses, while small
remaining mass SHOULD be derived from remaining volume rather than from
`1 - f` when that avoids cancellation.

#### 11.5.6 Validation and failure precedence

A null output-result pointer returns `BBTC_STATUS_INVALID_ARGUMENT`. Once a
nonnull result pointer has been accepted, the complete result MUST be cleared
before validating later inputs so every subsequent failure leaves a deterministic
zero record.

The initial evaluator ordering is:

1. validate the propellant-charge record through its existing native validator;
2. classify the explicit initial-grain-volume scalar;
3. validate the complete current grain-state scalar layer;
4. validate cross-record grain-state consistency against `V_g0`;
5. classify the burn-kinetics result's burn-rate scalar;
6. handle exact initial, exact burnout, and exact zero-rate semantics;
7. evaluate required positive interior coupling quantities; and
8. commit applicability metadata and the successful result.

For `V_g0`:

```text
NaN          -> BBTC_STATUS_NAN_INPUT
+Inf / -Inf  -> BBTC_STATUS_NONFINITE_INPUT
V_g0 <= 0    -> BBTC_STATUS_OUTSIDE_DOMAIN
```

Within the current grain-state scalar layer, NaN in any floating scalar takes
precedence over infinity anywhere in that record. Otherwise infinity in any
floating scalar returns `BBTC_STATUS_NONFINITE_INPUT`. After nonfinite
classification:

- `remaining_volume_m3` MUST be nonnegative;
- `burning_surface_area_m2` MUST be nonnegative;
- `remaining_regression_to_burnout_m` MUST be nonnegative; and
- `consumed_volume_fraction` MUST lie in the closed interval `[0, 1]`.

A finite violation returns `BBTC_STATUS_OUTSIDE_DOMAIN`.

The following cross-record relationships return
`BBTC_STATUS_INCONSISTENT_CONFIGURATION`:

- current remaining single-grain volume is greater than `V_g0`;
- exact `consumed_volume_fraction == 0` does not coincide with
  `remaining_volume_m3 == V_g0`, strictly positive burning area, and strictly
  positive remaining regression to burnout;
- exact `consumed_volume_fraction == 1` does not coincide with zero remaining
  volume, zero burning area, and zero remaining regression to burnout; or
- an interior consumed-volume fraction in `(0, 1)` does not coincide with
  positive remaining volume below `V_g0`, positive burning area, and positive
  remaining regression to burnout.

IB0.4e deliberately does not impose an arbitrary floating tolerance requiring

```text
f == 1 - V_g / V_g0
```

for every interior state. The IB0.4b geometry evaluator owns the detailed
relation between its own volume and consumed-fraction outputs. Rechecking that
identity here with an arbitrary tolerance would duplicate geometry logic and
could reject a numerically sound upstream result merely because two stable forms
round differently. The coupling layer validates the endpoint and broad
cross-record invariants it can defend without pretending to reproduce the
geometry backend.

For the burn-kinetics result's rate scalar:

```text
NaN          -> BBTC_STATUS_NAN_INPUT
+Inf / -Inf  -> BBTC_STATUS_NONFINITE_INPUT
r < 0        -> BBTC_STATUS_OUTSIDE_DOMAIN
r == 0       -> valid exact zero mass-rate state
r > 0        -> evaluate the positive rate relation when burning area remains
```

The applicability mask itself is metadata rather than a scalar mathematical
domain. IB0.4e MUST NOT reject, sanitize, or filter unknown applicability bits.

#### 11.5.7 Applicability propagation

On successful evaluation, the mass-coupling result MUST preserve the complete
incoming `bbtc_applicability_flags_t` bit pattern from the burn-kinetics result
verbatim, including bits introduced by future library versions. The coupling
layer MUST NOT silently discard an upstream scientific limitation merely because
the regression-to-mass arithmetic succeeded.

The initial IB0.4e coupling primitive introduces no new applicability flag of its
own. A later model may add one only when this layer has a scientifically distinct
nonfatal applicability limitation that cannot be represented by existing
metadata.

On failure, deterministic result clearing leaves the complete output record,
including applicability metadata, zeroed.

#### 11.5.8 Numerical representability and error budget

Every successful nonzero physical output that is mathematically required to be
positive MUST remain finite and strictly positive in the selected native scalar
family. If valid finite inputs require a positive equivalent population scale,
remaining volume, total burning area, remaining mass, reacted mass, or positive
reacted-mass rate that becomes zero through underflow or becomes nonfinite
through overflow, the evaluator returns `BBTC_STATUS_NUMERICAL_FAILURE`.

This rule applies to quantities that are physically required to be positive in
the evaluated state; it does not turn contract-defined exact zeros into errors.
Initial reacted mass, burnout remaining quantities, and a zero-rate mass flux
remain valid exact zeros.

The implementation SHOULD use exponent-safe rearrangement, ratio ordering, or
other algebraically equivalent native-precision forms when they avoid unnecessary
intermediate range loss. It is not required to provide arbitrary-range
arithmetic beyond the selected scalar family.

IB0.4e follows the section 2.2 engineering error-budget policy. Interior
floating-point identities need not be made artificially exact when doing so
would worsen conditioning. The target is for numerical error from this primitive
to be negligible relative to the uncertainty of the grain, kinetics, material,
and eventual experimental model inputs, while preserving the exact software
semantics explicitly required by this contract.

#### 11.5.9 Model boundary and deferred population physics

IB0.4e represents a uniformly regressing equivalent monodisperse population.
It does not determine whether or when that population ignites. A later ignition
model may determine when regression becomes active or what fraction of a more
advanced population participates, but such behavior MUST enter through an
explicit model rather than an undocumented multiplier in this primitive.

IB0.4e MUST NOT:

- evaluate pressure-dependent or temperature-dependent burn kinetics itself;
- infer a literal integer grain count from the equivalent population scale;
- round the equivalent population scale to an integer;
- infer grain-size distributions or mixed grain geometries;
- model partial ignition, primer-flame propagation, or ignition delay;
- model grain fracture, migration, collision, orientation, or compaction;
- alter condensed material density during regression;
- generate combustion-product mass or thermochemical reaction energy;
- call the IB0.4a thermochemical-source evaluator;
- mix combustion products with the initial trapped/free-gas population;
- evaluate a gas equation of state or chamber temperature;
- evolve chamber pressure or free volume as a dynamical state;
- move the projectile or evaluate projectile resistance;
- integrate time, regression distance, mass, pressure, or projectile motion;
- select a propellant product, grain geometry, or kinetics backend; or
- make an ammunition/firearm safety judgment.

The output `reacted_mass_rate_kg_per_s` is therefore a coupling source
term for a later solver, not a complete combustion model or firing prediction.

#### 11.5.10 IB0.4e-B1 public API

The concrete coupling API lives in:

```text
<bbtc/internal_ballistics/propellant_mass.h>
```

The module name deliberately uses the shorter `propellant_mass` term. The design
contract continues to describe the physical responsibility as regression-to-mass
coupling, but public identifiers do not repeat that phrase when the surrounding
internal-ballistics namespace already supplies the context.

The native evaluators are:

```c
bbtc_ib_propellant_mass_evaluate_float(...)
bbtc_ib_propellant_mass_evaluate_double(...)
bbtc_ib_propellant_mass_evaluate_long_double(...)
```

Each evaluator consumes, in this order:

1. a pointer to the matching `bbtc_ib_propellant_charge_*_t` record;
2. the explicit matching native scalar `initial_grain_volume_m3`;
3. a pointer to the matching `bbtc_ib_propellant_grain_state_*_t`;
4. a pointer to the matching
   `bbtc_ib_propellant_burn_kinetics_result_*_t`; and
5. a pointer to the matching `bbtc_ib_propellant_mass_result_*_t` output.

No public validator is introduced for `bbtc_ib_propellant_mass_result_*_t` because
the record is produced by BBTC rather than supplied as model configuration. No
new public generic grain-state validator is introduced solely for this coupling
layer; defensive grain-state validation remains a private implementation detail
whose behavior is governed by section 11.5.6.

A null output pointer returns `BBTC_STATUS_INVALID_ARGUMENT` without a writable
record. Once a nonnull output is accepted, it is zeroed before charge, scalar,
grain-state, or kinetics validation. A null grain-state or kinetics pointer is
therefore an invalid argument after output clearing. Charge validation continues
to use the existing charge validator and its established precedence.

The B1 implementation MUST keep `float`, `double`, and `long double` arithmetic
native. It MAY use private exponent-separated product/ratio helpers, `frexp` /
`ldexp` families, or other algebraically equivalent native forms to avoid
avoidable intermediate range failure. Such helpers MUST NOT convert one scalar
family through another or weaken the representability requirements of section
11.5.8.

The public result preserves the complete incoming applicability mask verbatim on
success. The evaluator does not inspect the provenance of unknown applicability
bits and does not add a B1-specific applicability bit.

IB0.4e-B1 remains a primitive algebraic coupling layer. It does not call a burn
kinetics backend, grain-geometry evaluator, or thermochemical-source evaluator
internally, and it does not advance regression distance or time.

### 11.6 Thermochemical source-rate coupling

IB0.4f introduces the explicit algebraic bridge between the whole-charge
reacted-propellant mass rate produced by IB0.4e and the reduced thermochemical
source terms required by the future coupled chamber-state solver.

The layer answers one deliberately narrow question:

```text
Given a valid reduced propellant thermochemistry record and an instantaneous
whole-charge reacted-propellant mass rate, at what rates are gaseous product,
condensed product, and reaction internal energy being generated?
```

IB0.4f does not determine how quickly the grain surface regresses, how much
surface area is burning, or how the reacted-mass rate was obtained. Those
responsibilities remain upstream in IB0.4b, IB0.4d, and IB0.4e.

Likewise, IB0.4f does not decide how generated gaseous product mixes with the
initial trapped gas, how released reaction energy changes gas temperature, how
condensed product occupies chamber volume, or how any source term changes
pressure. Those responsibilities belong to later chamber-population, energy,
equation-of-state, and integration layers.

#### 11.6.1 Inputs and dependency direction

The first source-rate coupling consumes:

1. one matching validated
   `bbtc_ib_propellant_thermochemistry_*_t` record; and
2. one matching successful `bbtc_ib_propellant_mass_result_*_t` record from
   the IB0.4e coupling layer.

The complete propellant-mass result is consumed instead of only a naked scalar
mass rate so that the downstream source-rate result can preserve the complete
upstream applicability mask without adding a separate metadata argument.

Only these two fields of the IB0.4e result participate in the IB0.4f source-rate
contract:

```text
applicability_flags
reacted_mass_rate_kg_per_s
```

The other IB0.4e result fields remain useful diagnostics and state quantities,
but they are not inputs to the source-rate equations. IB0.4f MUST NOT attempt to
reconstruct or repeat the IB0.4e charge/geometry consistency checks from those
diagnostic fields because the original charge, initial grain volume, and grain
state are not inputs to this layer.

A caller supplying a hand-constructed propellant-mass result rather than a
successful upstream BBTC result is responsible for the unrelated fields in that
record. IB0.4f defensively validates the mass-rate scalar it actually consumes
and treats the applicability mask as opaque metadata.

The dependency direction is therefore:

```text
burn kinetics + grain geometry + charge
                  |
                  v
       IB0.4e propellant mass
                  |
                  | reacted_mass_rate_kg_per_s
                  | applicability_flags
                  v
     IB0.4f thermochemical source rates
```

IB0.4f MUST NOT call back into a burn-kinetics backend, grain-geometry evaluator,
or propellant-mass evaluator.

#### 11.6.2 Rate equations and dimensional semantics

Let

```text
mdot_r = whole-charge reacted-propellant mass rate, kg/s
y_g    = gaseous-product mass fraction, dimensionless
q_r    = positive specific reaction internal-energy release, J/kg
```

with `y_g` and `q_r` retaining exactly the meanings already frozen by
section 11.1.

The reduced thermochemical source rates are:

```text
mdot_g = y_g * mdot_r
mdot_c = (1 - y_g) * mdot_r
Qdot_r = q_r * mdot_r
```

where:

```text
mdot_g = generated gaseous-product mass rate, kg/s
mdot_c = generated condensed-product mass rate, kg/s
Qdot_r = positive reaction internal-energy release rate, J/s = W
```

The energy-rate quantity is intentionally a positive release rate. It is the
rate at which the reduced thermochemical model makes reaction internal energy
available to a later energy balance.

`Qdot_r` MUST NOT be interpreted directly as:

- the time derivative of chamber-gas internal energy;
- net heat-transfer power into the gas;
- projectile-work power;
- a flame temperature;
- propellant force or impetus;
- an enthalpy-flow term; or
- a caloric-reference correction.

A later energy balance is responsible for combining reaction-energy release
with mechanical work, heat transfer, gas and condensed-phase caloric models,
reference-state conventions, and any other explicitly selected terms.

#### 11.6.3 Relationship to the cumulative IB0.4a source

IB0.4a and IB0.4f describe complementary views of the same reduced
thermochemistry.

IB0.4a maps an accumulated reacted propellant mass to accumulated extensive
source quantities:

```text
m_r  ->  m_g, m_c, Q_r
```

IB0.4f maps the instantaneous reacted-propellant mass rate to instantaneous
source rates:

```text
mdot_r  ->  mdot_g, mdot_c, Qdot_r
```

For constant `y_g` and `q_r`, and for a differentiable reacted-mass history,
the IB0.4f equations are the time derivatives of the IB0.4a extensive-source
equations.

This mathematical relationship does not permit an implementation to pass a
value expressed in kilograms per second to the IB0.4a evaluator as though it
were a mass in kilograms. The two APIs have different dimensional contracts
even though their reduced coefficients lead to algebraically similar
multiplications.

IB0.4f MUST therefore expose source-rate semantics explicitly. A future
implementation MAY share private dimension-neutral arithmetic helpers with
IB0.4a when useful, but it MUST NOT implement the public rate evaluator by
dimensionally misusing the public cumulative-source evaluator.

When a later time integrator uses constant thermochemistry over an interval,
the time integral of the IB0.4f source rates SHOULD agree with the corresponding
change in the IB0.4a cumulative source quantities to within the selected
integration and native-arithmetic error. IB0.4f itself performs no integration
and accepts no time step.

#### 11.6.4 Applicability propagation

The first reduced thermochemistry record carries no applicability mask of its
own. The IB0.4e propellant-mass result, however, preserves applicability
metadata originating in the selected burn-kinetics backend.

On every successful IB0.4f evaluation, the output applicability mask MUST equal
the complete incoming IB0.4e applicability mask bit-for-bit.

This includes:

- `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN`; and
- any future applicability bits unknown to the IB0.4f implementation.

IB0.4f MUST NOT filter, reinterpret, clear, or manufacture applicability bits.
It introduces no new applicability flag in this checkpoint.

Propagation is required even for an exact zero reacted-mass-rate identity
state. A downstream zero source rate does not erase scientific information
about the upstream state that produced it.

If a later thermochemistry backend gains its own calibration-domain or
provenance applicability metadata, a future contract revision MUST define how
the two masks are combined. IB0.4f-A does not invent such a policy in advance.

#### 11.6.5 Zero, positive, and negative reacted-mass-rate semantics

An exact finite zero reacted-propellant mass rate is a valid identity state.
Successful evaluation at:

```text
mdot_r == 0
```

MUST produce exactly:

```text
mdot_g = 0
mdot_c = 0
Qdot_r = 0
```

while preserving the incoming applicability mask verbatim.

This identity is suitable for pre-reaction states, zero-rate burn-kinetics
boundaries, and exact geometric burnout. It does not itself distinguish why the
upstream rate is zero.

A finite negative reacted-propellant mass rate is outside the domain of the
first irreversible reduced-combustion source model and MUST return
`BBTC_STATUS_OUTSIDE_DOMAIN`.

IB0.4f does not model reverse chemical reaction, product recombination into
unreacted propellant, negative burning-area conventions, or a signed source/sink
formalism. Such models require an explicit later contract if they become
scientifically justified.

For a positive accepted reacted-mass rate:

- generated gaseous-product mass rate MUST be finite and strictly positive;
- reaction internal-energy release rate MUST be finite and strictly positive;
- when `y_g < 1`, generated condensed-product mass rate MUST be finite and
  strictly positive; and
- exact `y_g == 1` is the all-gas limit and MUST produce exactly zero
  condensed-product mass rate.

#### 11.6.6 Validation and failure precedence

IB0.4f preserves the established BBTC pattern of deterministic output clearing
and model-validation precedence.

The first rate evaluator MUST validate in this order:

1. if the output-result pointer is null, return
   `BBTC_STATUS_INVALID_ARGUMENT`;
2. clear the complete output record;
3. validate the matching reduced thermochemistry record with the existing
   `bbtc_ib_propellant_thermochemistry_validate_*()` semantics;
4. if the propellant-mass-result pointer is null, return
   `BBTC_STATUS_INVALID_ARGUMENT`;
5. classify `reacted_mass_rate_kg_per_s`: NaN returns
   `BBTC_STATUS_NAN_INPUT`, otherwise positive or negative infinity returns
   `BBTC_STATUS_NONFINITE_INPUT`;
6. reject a finite negative reacted-mass rate with
   `BBTC_STATUS_OUTSIDE_DOMAIN`;
7. handle exact zero as the identity state, copy the incoming applicability
   mask, and return success;
8. evaluate the three positive-rate source terms in the selected native scalar
   family;
9. validate representability and the exact all-gas condensed-rate invariant;
   and
10. commit the successful source rates and the unmodified applicability mask to
    the caller-owned output.

Once a nonnull output is accepted, every subsequent failure MUST leave the
complete output in its cleared state. Applicability metadata is committed only
on success.

Thermochemistry validation intentionally precedes classification of the
upstream mass rate, matching the model-before-direct-source precedence already
used by the IB0.4a cumulative-source evaluator.

IB0.4f MUST NOT reject an otherwise valid evaluation merely because an
unrelated diagnostic field in the supplied IB0.4e result is not independently
revalidated by this layer.

#### 11.6.7 Numerical and conservation requirements

Each scalar family MUST evaluate the source-rate equations natively as
`float`, `double`, or `long double`. One family MUST NOT route its calculation
through another.

The implementation MAY use algebraically equivalent native-scalar arithmetic
when useful, but the public equations and units remain those in section 11.6.2.
No result may be clipped or saturated to manufacture success.

For positive `mdot_r`, any mathematically required positive source rate that
overflows, underflows to zero, becomes nonfinite, or otherwise cannot be
represented as a finite strictly positive value in the selected native scalar
family MUST produce `BBTC_STATUS_NUMERICAL_FAILURE`.

A positive subnormal output remains valid when it is representable and survives
the selected native operation as a finite positive value.

The reduced mass-partition equation is:

```text
mdot_g + mdot_c = mdot_r
```

This is an algebraic model requirement, not a bit-for-bit floating-point
identity requirement. Independently rounded native result fields MAY differ
from an exact floating-point complement by ordinary rounding. Tests and future
conservation diagnostics SHOULD use a precision-aware residual rather than
requiring exact equality of the rounded sum.

The rate layer introduces no time-discretization error because it performs no
integration. Numerical integration error enters only when a later solver
integrates these rates over time.

#### 11.6.8 Ownership, mutation, and allocation

All IB0.4f input records are borrowed, immutable caller-owned objects for the
duration of one call.

The evaluator MUST NOT:

- mutate the thermochemistry record;
- mutate the propellant-mass result;
- retain pointers to caller-owned inputs after return;
- allocate dynamic memory;
- access hidden mutable global state; or
- perform file, terminal, logging, or environment-variable I/O.

The caller owns the output record. Successful evaluation writes only the
documented output fields after all validation and numerical checks have passed.

#### 11.6.9 Explicit exclusions and deferred responsibilities

IB0.4f MUST NOT:

- evaluate pressure- or temperature-dependent burn kinetics;
- determine grain regression distance or burning surface area;
- derive reacted-propellant mass rate from charge or grain geometry;
- model ignition progression or primer/flame propagation;
- infer species composition or chemical equilibrium;
- calculate flame or adiabatic-combustion temperature;
- alter the reduced `y_g` or `q_r` coefficients from pressure or temperature;
- mix generated gaseous product with the initial trapped/free-gas population;
- assign an equation of state to generated combustion products;
- determine combustion-product temperature;
- convert the reaction-energy release rate directly into chamber-gas internal
  energy without a later explicit energy balance;
- determine heat-transfer, wall-loss, projectile-work, or flow-work rates;
- determine the volume occupied by condensed reaction products;
- evolve chamber free volume, pressure, or temperature;
- move the projectile or evaluate projectile resistance;
- integrate any state in time;
- select a propellant formulation or calibration record; or
- make an ammunition/firearm safety judgment.

The immediate downstream consumer is a future chamber-state/energy coupling
layer that will decide how generated products and reaction-energy release enter
the evolving lumped internal-ballistics state.

#### 11.6.10 IB0.4f-A checkpoint boundary

IB0.4f-A is a design-contract checkpoint only.

It freezes:

- consumption of the complete successful IB0.4e propellant-mass result rather
  than a naked reacted-mass-rate scalar;
- reuse of the existing reduced thermochemistry record;
- verbatim propagation of the complete upstream applicability mask;
- separate cumulative-source and instantaneous source-rate semantics;
- exact zero-rate identity behavior;
- rejection of finite negative reacted-mass rate;
- positive reaction-energy-release sign convention;
- model-before-mass-rate validation precedence;
- native-precision representability requirements; and
- the responsibility boundary between source-rate generation and the future
  chamber-state/energy solver.

IB0.4f-A intentionally does not freeze exact public source-rate type names,
function names, or declaration layout. Those identifiers are reviewed in the
first concrete IB0.4f implementation checkpoint so that the public API can
remain concise without weakening dimensional clarity.

IB0.4f-A introduces no source implementation, public symbol, CMake target,
solver state, time integrator, pressure evolution, projectile motion, firing
prediction, or safety judgment.

## 12. Initial physical model

The first complete internal-ballistics model is a zero-dimensional,
lumped-parameter engineering model. Its target scope is:

- explicit initial free-gas absolute pressure and temperature;
- explicit initial condensed-propellant temperature;
- initial free-gas mass and density closure from the selected gas model;
- a documented ignition model or explicitly identified simplified ignition
  assumption;
- empirical propellant surface-regression kinetics;
- temperature-aware burn kinetics when a calibrated selected backend supports
  that dependence;
- burn-surface evolution from grain geometry;
- explicit coupling from charge mass, condensed density, grain geometry, and
  regression rate to whole-charge reacted-mass rate;
- reduced propellant thermochemical gas-mass, condensed-product-mass, and
  reaction-energy source rates;
- an explicit closure for the evolving initial-gas and combustion-product gas
  populations;
- a selected reduced gas equation of state and compatible caloric model;
- an explicit single-shot energy balance;
- changing gas volume as propellant is consumed and the projectile moves;
- a documented lumped pressure-gradient/gas-inertia correction that can
  distinguish mean, breech, and projectile-base pressure without claiming to
  resolve a spatial pressure field;
- an explicit pressure boundary ahead of the projectile, with equality to
  ambient pressure only through a documented input or helper policy;
- a defined projectile-start/engraving and bore-resistance model;
- adaptive integration of the coupled state;
- accurate location of ignition-related events represented by the model,
  projectile start, propellant burnout when applicable, and muzzle exit;
- energy-accounting diagnostics;
- model/applicability metadata;
- optional time-history sampling; and
- reproducible identification of every active physical backend.

Mean chamber pressure, estimated breech pressure, estimated projectile-base
pressure, and pressure ahead of the projectile are distinct physical concepts.
A lumped correction may derive several of them from one mean state, but the
result MUST NOT imply that a zero-dimensional model resolved multidimensional
pressure waves or a full spatial field.

A primitive burn-kinetics evaluator receives one explicit absolute-pressure
scalar and does not infer whether that scalar represents mean, breech,
projectile-base, or another modeled pressure quantity. The coupled-solver
contract MUST explicitly define and expose which modeled pressure quantity drives
the selected burn-kinetics backend. The solver MUST NOT silently substitute a
different pressure convention when a pressure-gradient correction is enabled or
disabled.

The first complete model MAY use an adiabatic single-shot energy balance while
detailed wall, case, barrel, projectile, and unburned-propellant heat transfer
remains deferred. That approximation MUST be identifiable in the model
configuration or result metadata.

An initial helper MAY explicitly construct initial free-gas conditions from a
supplied ambient state, but the low-level physical records MUST NOT silently
equate chamber gas, propellant, or forward-bore conditions with ambient
conditions.

Individual physical effects SHOULD be independently selectable where doing so
supports verification, validation, sensitivity analysis, or scientific
comparison. A disabled effect MUST be recorded in the model configuration or
result metadata.

## 13. Deferred and evidence-driven physical effects

The first complete engineering model deliberately does not claim to resolve:

- multidimensional gas flow, shocks, or pressure waves;
- full spatial primer-flame propagation;
- detailed granular-bed gas permeability;
- individual-grain fracture, migration, collision, or orientation;
- explicit grain-to-grain manufacturing distributions;
- erosive burning unless a later calibrated model earns inclusion;
- depth-dependent deterrent/coating diffusion or multi-zone chemistry;
- evolving condensed-propellant temperature beyond the selected initial or
  reduced temperature-response model;
- detailed heat transfer to case, chamber, projectile, propellant, and barrel;
- gas leakage, blow-by, or obturation failure;
- detailed compression and expulsion of bore gas ahead of the projectile beyond
  the selected forward-pressure boundary model;
- elastic or plastic cartridge-case and chamber deformation;
- detailed finite-element projectile engraving or rifling deformation;
- detailed gas-species reaction chemistry or full chemical equilibrium;
- barrel thermoelastic response, wear, erosion, or changing roughness;
- structural firearm failure;
- stochastic cartridge or firearm explosion probability;
- full rotating-system and recoil-system dynamics;
- one-, two-, or three-dimensional combustion-flow CFD; or
- external ballistics.

These effects are deferred, not forbidden. Each MUST enter, if justified, as a
versioned and independently testable model term rather than an invisible
correction factor.

Promotion of a deferred effect into the default engineering model SHOULD be
supported by the section 2.1 engineering-fidelity criteria, validation evidence,
or sensitivity analysis showing that the effect materially improves intended
outputs or expands a useful applicability domain.

An effect whose expected contribution is smaller than the uncertainty of the
parameters required to model it SHOULD normally remain optional or deferred.
This rule does not prohibit research-oriented implementations; it prevents
research complexity from silently becoming a mandatory engineering dependency.

## 14. Internal-ballistics result boundary

The result model will distinguish and expose, when valid:

- mean chamber peak pressure, time, and projectile position;
- estimated projectile-base peak pressure, time, and position;
- muzzle velocity;
- muzzle pressure;
- muzzle gas temperature;
- barrel time;
- propellant burn fraction at muzzle exit;
- projectile kinetic energy;
- work against engraving and bore resistance;
- gas and thermal energy terms represented by the model;
- energy-accounting residual in joules and as a dimensionless fraction;
- accepted and rejected integration steps;
- termination reason;
- warning flags;
- model-limit and applicability flags;
- field-validity metadata;
- active model identifier and model revision.

There will be no generic result member named merely `pressure`.

The muzzle-state contract used later by external ballistics MUST state the
reference position, coordinate frame, time, velocity convention, orientation,
and every quantity's validity.

## 15. Data provenance and reproducibility

User-entered values are valid inputs, but they are not automatically validated
physical data.

Catalog and calibration records SHOULD carry:

- stable identifier;
- schema version;
- source citation or source description;
- publication or retrieval date when applicable;
- lot, formulation, conditioning temperature, and test-fixture assumptions when
  known;
- measurement or parameter uncertainty when known;
- repeatability statistics when available;
- represented and calibrated domains;
- raw measurement-set identity or location when applicable;
- derivation or fitting method when a compact model was fitted from data;
- fit residuals or other fit-quality metrics when available; and
- revision history.

A simulation result SHOULD be reproducible from:

- all normalized SI inputs;
- the selected scalar-precision family and its relevant numeric characteristics;
- model and solver versions;
- enabled model terms;
- solver tolerances and guards;
- catalog-record identifiers and revisions;
- BBTC library version.

The core solver MUST NOT fetch data from a network.

### 15.1 Sensitivity and uncertainty analysis

After the first coupled internal-ballistics solver exists, BBTC SHOULD support
repeatable sensitivity and uncertainty studies over selected physical inputs and
model parameters.

Such studies are intended to answer engineering questions such as whether
uncertainty in charge mass, grain dimensions, burn kinetics, initial propellant
temperature, thermochemical parameters, gas-model parameters, projectile
resistance, or another input materially controls uncertainty in an output.

Relevant outputs include, when modeled:

- muzzle velocity;
- peak mean, breech, and projectile-base pressure;
- muzzle pressure;
- barrel time;
- propellant burnout time and position;
- burn fraction at muzzle exit; and
- energy-accounting residuals.

Sensitivity results MUST NOT be interpreted as proof that omitted physics is
irrelevant outside the studied input and calibration domain.

When an optional physical effect changes an intended engineering output by less
than the uncertainty already induced by supported inputs or experimental
variation, that evidence MAY justify leaving the effect optional or deferred.
When an effect materially changes the result and its parameters can be supplied
with useful fidelity, that evidence SHOULD weigh in favor of implementing or
promoting the model.

Uncertainty analysis MUST distinguish numerical precision from parameter,
calibration, and physical-model uncertainty. Additional floating-point digits
MUST NOT be presented as additional experimental certainty.

## 16. Verification and validation

BBTC distinguishes:

- **software verification**: the equations are implemented and solved as
  intended;
- **numerical verification**: convergence, invariants, event location, and
  error behavior are characterized;
- **physical validation**: predictions are compared with suitable experimental
  measurements over a stated domain.

Tests will include, as the relevant code appears:

- input-validation tests;
- unit tests for geometry and burn laws;
- pressure-power fit-residual studies against source burn-rate data;
- exact-knot and between-knot tests for tabulated burn-rate interpolation;
- dimensional and limiting-case checks;
- closed-bomb tests;
- energy and monotonicity invariants;
- adaptive-step convergence studies;
- separate convergence studies for `float`, `double`, and `long double`;
- cross-precision comparisons over declared input domains;
- event-location tests;
- regression tests with explained tolerances;
- sensitivity and uncertainty studies for influential physical inputs;
- comparisons showing whether optional model terms materially improve suitable
  reference cases;
- sanitizer runs;
- fuzz or property tests for parsers and validation boundaries;
- comparison with cited experimental pressure and velocity data.

A build that passes unit tests is not thereby physically validated.

Reference cases MUST record their provenance, conditions, uncertainty where
known, and why the comparison is appropriate. Golden numbers without an
explanation are insufficient.

Cross-precision comparison MUST NOT treat `long double` output as experimental
truth. It is a numerical diagnostic. The `float` family must have stated error
envelopes for any domain in which it is recommended, and performance claims
must be supported by benchmarks on named platforms.

## 17. CLI output and ANSI color

### 17.1 Output classes

The CLI separates:

- human-readable output;
- diagnostics;
- machine-readable output.

Human results normally go to standard output. Diagnostics normally go to
standard error. Machine-readable schemas MUST be documented and MUST never
contain ANSI escape sequences.

### 17.2 Color modes

The CLI supports:

```
--color=auto
--color=always
--color=never
--no-color
```

`--no-color` is an alias for `--color=never`. The default is
`--color=auto`. An unknown color value is a usage error. If color options are
repeated, the last occurrence wins.

Color-mode precedence is:

1. An explicit command-line color option.
2. Presence of the `NO_COLOR` environment variable, which selects `never`.
3. Automatic terminal and capability detection.

In `auto` mode, color is enabled independently for each destination stream only
when that stream is an interactive terminal, `TERM` is not `dumb`, and the
platform supports or can enable the required terminal behavior.

`always` may emit color to a redirected human-readable stream because the user
requested it explicitly. CSV, JSON, and every other machine-readable format
remain uncolored even under `--color=always`.

On Windows, terminal-mode changes belong to the CLI and MUST be scoped and
handled safely. Failure to enable virtual-terminal processing causes `auto` to
fall back to plain text; it is not a simulation failure.

### 17.3 Semantic accessibility

Color is decoration, never the sole carrier of meaning. Every diagnostic uses a
text label such as `INFO`, `WARNING`, or `ERROR`, and tabular output remains
understandable without color.

Escape sequences MUST be centralized in the CLI presentation layer. Physical
code, public result strings, logs intended as data, CSV, JSON, and redirected
output in `auto` mode remain plain.

Help and version requests MUST print the requested information and terminate
without running a simulation or creating an output file.

The CLI MUST allow deliberate selection of an available scalar-precision family
and MUST identify the selected family in human-readable and machine-readable
results. Its default precision family is `double`.

## 18. Safety and claims

BBTC is educational and experimental simulation software. Its output consists
of model predictions, not pressure measurements, published load data, proof
testing, or a determination that ammunition or a firearm is safe.

The software **MUST NOT**:

- recommend a charge as safe;
- generate a "maximum safe load";
- certify a cartridge, component combination, or firearm;
- imply that absence of a warning is evidence of safety;
- disguise unknown propellant data behind a confident product-name preset.

The canonical notice will live in `DISCLAIMER.md`. Short, consistent notices
will appear in:

- the README;
- CLI help;
- human-readable simulation reports;
- public API documentation;
- documentation for machine-readable result fields and applicability flags.

The concise notice should communicate:

> BBTC produces model predictions, not pressure measurements or load data. It
> does not determine whether ammunition or a firearm is safe.

Machine-readable results MUST include model identity, status, termination,
warnings, and applicability information. They need not inject prose into every
numeric record.

Disclaimers do not repair bad engineering. Validation, explicit uncertainty,
provenance, and honest model limits remain mandatory.

## 19. Versioning and compatibility

The rewrite begins below version 1.0. Until 1.0:

- source and binary compatibility are not guaranteed;
- public API changes MUST still be deliberate and documented;
- model revisions that can change results MUST be identifiable;
- regression-reference updates MUST explain the physical or numerical reason;
- silent semantic changes to an existing field are forbidden.

The library version, model version, and data-record revision are separate
concepts.

Compatibility with the legacy pre-rewrite CLI and source API is not a goal.

## 20. Change discipline

Each implementation increment SHOULD be small enough to review as one coherent
idea. Physics, API, build-system, and presentation changes SHOULD be separated
when practical.

A change is incomplete until:

- its public meaning is documented;
- invalid-input behavior is defined;
- relevant tests pass;
- compiler diagnostics are reviewed;
- the worktree contains no generated garbage;
- any changed model claim or applicability boundary is recorded.

If implementation and this contract disagree, the discrepancy MUST be resolved
explicitly. Neither code nor prose silently wins.

## 21. Decisions resolved in IB0.2a

The following decisions were explicitly approved after IB0.1:

1. **Project license.** BBTC is publicly available under the PolyForm
   Noncommercial License 1.0.0. Commercial use requires a separate written
   license or written permission from the copyright holder. Project-specific
   terms and contact information live in `LICENSING.md`.
2. **Minimum CMake version.** BBTC requires CMake 3.22 or newer. This matches
   the Cubes of Honor baseline and provides C23 standard-selection support plus
   top-level/subproject detection without requiring newer CMake features.

The initial verification compilers are GCC and Clang. A complete supported
compiler and platform matrix remains deferred until those combinations have
repeatable CI or equivalent recorded verification.

## 22. Decisions resolved in IB0.2b

The following decisions define the first public BBTC API:

1. **Status representation.** IB0.2b originally used a fixed `uint32_t`
   underlying type. IB0.3a supersedes only that width with `uint8_t`; the
   stable numeric assignments in section 9.1 are unchanged.
2. **Public status headers.** `<bbtc/status.h>` owns the status declaration,
   while `<bbtc/bbtc.h>` is the public umbrella header. Both are valid from
   ISO C23 and C++11 or newer.
3. **Status text.** `bbtc_status_string()` returns immutable, nonlocalized
   static text, has no mutable global state, and maps unknown values to one
   stable fallback string.
4. **Scope boundary.** This checkpoint adds no solver, physical termination
   reason, warning flag, validity mask, applicability flag, or safety judgment.

## 23. Decisions resolved in IB0.2c

The following decisions define BBTC's first public diagnostic metadata:

1. **Header ownership.** `<bbtc/diagnostics.h>` owns termination, warning, and
   applicability declarations and is included by `<bbtc/bbtc.h>`.
2. **Termination representation.** IB0.2c originally used `uint32_t`
   representation. IB0.3a supersedes only that width with `uint8_t`; the
   stable values in section 9.2 and the zero-value meaning are unchanged.
3. **Flag representation.** `bbtc_warning_flags_t` and
   `bbtc_applicability_flags_t` are exactly `uint64_t`. Their individual
   declarations use fixed `uint64_t` enumeration types, and every assigned
   nonzero value is one independent bit.
4. **Separate meanings.** API status reports whether an operation completed;
   termination reports why a simulation stopped; warnings report nonfatal
   computational or reporting conditions; applicability flags report limits on
   scientific interpretation.
5. **Diagnostic text.** Termination and individual-flag string functions return
   immutable, nonlocalized static text. Combined or unknown flag values receive
   an explicit fallback instead of an invented aggregate sentence.
6. **Validity timing.** Result-field validity remains mandatory, but its public
   representation is deferred until concrete result fields exist.
7. **Scope boundary.** This checkpoint adds no solver, result structure,
   validity bit, physical prediction, or safety judgment.

## 24. Decisions resolved in IB0.3a

The following decisions define scalar-family identity and platform metadata:

1. **Precision identity.** `bbtc_precision_e` uses `uint8_t` representation and
   identifies the `float`, `double`, and `long double` families.
2. **Metadata only.** Runtime precision identity reports host properties and
   does not replace precision-qualified records or concrete functions.
3. **Half precision.** `_Float16` is not a first-class solver family because it
   cannot represent BBTC's required public SI pressure domain and does not meet
   the numerical contract.
4. **Enum refinement.** `bbtc_status_e` and `bbtc_ib_termination_e` use
   `uint8_t` representation without changing their established values.
5. **Scope boundary.** This checkpoint adds no ballistic input record, solver,
   physical prediction, or result.

## 25. Decisions resolved in IB0.3b

The following decisions define the first public physical input component:

1. **Header ownership.** `<bbtc/internal_ballistics.h>` is the public
   internal-ballistics umbrella header, and
   `<bbtc/internal_ballistics/geometry.h>` owns geometry declarations.
2. **Concrete scalar records.** Float, double, and long-double geometry use
   separate precision-qualified structures whose continuous fields remain in
   their native scalar family.
3. **Geometry boundary.** The four fields and their exact physical meanings are
   fixed by section 10.1. The two effective areas remain distinct and need not
   be equal.
4. **Validation.** Null, NaN, infinity, and finite out-of-domain inputs return the
   statuses specified in section 10.1. Validation does not modify its input.
5. **No runtime union foundation.** A tagged union may exist later as an
   adapter, but it is not the concrete solver ABI and does not replace static
   precision identity.
6. **No physical defaults.** Zero initialization is an invalid sentinel state;
   BBTC supplies no imaginary default chamber, bore, or barrel geometry.
7. **Pre-1.0 structure evolution.** Public structure evolution remains
   deliberate at source level. This checkpoint adds no structure-size field,
   version member, reserved array, or named padding.
8. **Scope boundary.** This checkpoint validates geometry but adds no composed
   problem, propellant model, integration state, solver, or physical result.

## 26. Decisions resolved in IB0.3c

The following decisions define the first public projectile component:

1. **Header ownership.**
   `<bbtc/internal_ballistics/projectile.h>` owns projectile declarations and is
   included by `<bbtc/internal_ballistics.h>`.
2. **Concrete scalar records.** Float, double, and long-double projectile data
   use separate precision-qualified structures with native scalar `mass_kg`
   members.
3. **Mass boundary.** `mass_kg` has the exact physical meaning stated in section
   10.2. Geometry-owned area and travel fields are not duplicated.
4. **Initial state separation.** Initial projectile velocity is deferred to a
   future initial-state or composed-problem record.
5. **Validation.** Null, NaN, infinity, and finite out-of-domain inputs return the
   statuses specified in section 10.2. Validation does not modify its input.
6. **No runtime union foundation.** A tagged union may exist later as an
   adapter, but it does not replace static precision identity or concrete
   functions.
7. **No physical defaults.** Zero initialization is an invalid sentinel state;
   BBTC supplies no imaginary default projectile mass.
8. **Pre-1.0 structure evolution.** Projectile records add no structure-size
   field, version member, reserved array, or named padding.
9. **Scope boundary.** This checkpoint validates projectile mass but adds no
   composed problem, propellant model, integration state, solver, or result.

## 27. Decisions resolved in IB0.3d

The following decisions define the first public propellant-charge component:

1. **Header ownership.**
   `<bbtc/internal_ballistics/propellant_charge.h>` owns propellant-charge
   declarations and is included by `<bbtc/internal_ballistics.h>`.
2. **Concrete scalar records.** Float, double, and long-double
   propellant-charge data use separate precision-qualified structures with
   native `charge_mass_kg` and `condensed_phase_density_kg_per_m3` members.
3. **Density boundary.** Condensed-phase density is material density excluding
   intergranular void space. Bulk loading density, powder shape, and product
   identity are not substitutes.
4. **Derived-volume source of truth.** Condensed propellant material volume is
   derived from charge mass divided by condensed-phase density and is not stored
   as a redundant public input.
5. **Composition boundary.** Cross-record free-gas-volume validation remains
   deferred until a composed problem record exists.
6. **Validation.** Null, NaN, infinity, and finite out-of-domain inputs return the
   statuses specified in section 10.3. Validation does not modify its input.
7. **No runtime union foundation.** A tagged union may exist later as an
   adapter, but it does not replace static precision identity or concrete
   functions.
8. **No physical defaults.** Zero initialization is an invalid sentinel state;
   BBTC supplies no imaginary default charge or density.
9. **Pre-1.0 structure evolution.** Propellant-charge records add no
   structure-size field, version member, reserved array, or named padding.
10. **Scope boundary.** This checkpoint validates primitive propellant-charge
    data but adds no burn law, thermochemistry, grain model, composed problem,
    integration state, solver, or result.

## 28. Decisions resolved in IB0.3e

The following decisions define the first composed physical-input boundary:

1. **Header ownership.**
   `<bbtc/internal_ballistics/loading_state.h>` owns loading-state and derived
   initial-volume declarations and is included by
   `<bbtc/internal_ballistics.h>`.
2. **By-value composition.** Each loading state owns matching-precision
   geometry, projectile, and propellant-charge components without duplicating
   their primitive fields.
3. **Derived source of truth.** Condensed propellant volume and initial free-gas
   volume are outputs derived from existing primitives, not redundant caller
   inputs.
4. **Cross-record fit rule.** Condensed propellant volume must be representable,
   positive, and strictly smaller than initial behind-projectile geometric
   volume.
5. **Output discipline.** A nonnull derived-volume output is zeroed before every
   failure return. Null input or output pointers are invalid arguments.
6. **Status propagation.** Component validation runs geometry, projectile, then
   propellant charge and returns the first non-success status.
7. **No runtime union foundation.** Concrete native scalar families remain the
   public computational boundary.
8. **No safety inference.** Passing composition validation says nothing about
   real pressure, firearm strength, ammunition compatibility, or load safety.
9. **Pre-1.0 structure evolution.** Loading-state records add no structure-size
   field, version member, reserved array, or named padding.
10. **Scope boundary.** This checkpoint adds no burn law, thermochemistry, grain
    model, ignition state, resistance model, integration state, solver, result,
    pressure, or velocity.

## 29. Decisions resolved in IB0.3f

The following decisions define the primitive initial gas-state boundary:

1. **Header ownership.**
   `<bbtc/internal_ballistics/initial_gas_state.h>` owns initial gas-state
   declarations and is included by `<bbtc/internal_ballistics.h>`.
2. **Concrete scalar records.** Float, double, and long-double records use
   separate native scalar `absolute_pressure_pa` and `temperature_k` fields.
3. **Absolute quantities.** Pressure is absolute rather than gauge pressure, and
   temperature is expressed in kelvin.
4. **Boundary-condition ownership.** The caller supplies both values explicitly.
   BBTC does not silently substitute ambient conditions or physical defaults.
5. **No unsupported derivation.** Pressure and temperature are not derived from
   each other until gas quantity or density, composition, volume, and an
   equation-of-state model establish a sufficient relationship.
6. **Validation.** Null, NaN, infinity, and finite out-of-domain inputs return the
   statuses specified in section 10.5. Validation does not modify its input.
7. **No runtime union foundation.** Concrete native scalar families remain the
   public computational boundary.
8. **No safety inference.** Passing primitive validation says nothing about real
   pressure evolution, ammunition compatibility, firearm strength, or load
   safety.
9. **Pre-1.0 structure evolution.** Initial gas-state records add no
   structure-size field, version member, reserved array, or named padding.
10. **Scope boundary.** This checkpoint adds no gas composition, gas amount,
    equation of state, thermochemistry, energy balance, burn law, solver,
    pressure evolution, velocity prediction, or result record.

## 30. Decisions resolved in IB0.3g

The following decisions define the first explicit gas constitutive model:

1. **Header ownership.**
   `<bbtc/internal_ballistics/noble_abel_gas_model.h>` owns Noble-Abel gas-model
   declarations and is included by `<bbtc/internal_ballistics.h>`.
2. **Concrete scalar records.** Float, double, and long-double records use
   separate native scalar `R`, `c_v`, and specific-covolume fields with the
   names and units in section 10.6.
3. **Mechanical closure.** The selected pressure-volume-temperature relation is
   the Noble-Abel equation in section 10.6.
4. **Caloric closure.** The model is calorically perfect with constant `c_v`.
   `c_p` and `gamma` are derived rather than stored as alternate sources of
   truth.
5. **Ideal-gas limit.** Zero covolume is valid and explicitly represents the
   ideal-gas limit. Negative covolume is outside the model domain.
6. **Initial-mass boundary.** The closed-form initial gas-mass relation is
   documented, but its cross-record evaluator remains deferred.
7. **Validation.** Null, NaN, infinity, and finite out-of-domain inputs return the
   statuses specified in section 10.6. Validation does not modify its input.
8. **Model identity boundary.** The record supplies constant constitutive
   parameters for one effective pseudo-gas but does not identify chemistry,
   composition, calibration, provenance, or uncertainty. Initial fill gas and
   combustion-product gas remain distinct populations.
9. **No runtime union foundation.** Concrete native scalar families remain the
   public computational boundary.
10. **Scope boundary.** This checkpoint selects no universal EOS and adds no
    runtime backend dispatch, virial implementation, gas-mass evaluator,
    thermochemistry, combustion-product generation, grain model, burn law,
    energy integration, pressure evolution, solver, result, or safety judgment.


### 30.1 Decisions resolved in IB0.3h

The following decisions define the temperature-dependent first-order virial
parameter and coefficient-law boundary:

1. **Density form.** The model uses the mass-density form
   `p = rho*R*T*(1 + B(T)*rho)`. `B(T)` therefore has units of cubic meters per
   kilogram.
2. **Temperature dependence.** `B` is represented as a function of absolute
   temperature rather than as one permanently constant scalar.
3. **Series representation.** A bounded first-kind Chebyshev series over an
   explicit closed temperature interval is the foundational public
   representation. The series uses `sum(c[k]*T_k(x))` with no half-weighted
   zeroth term.
4. **Borrowed storage.** Coefficient arrays remain caller-owned and immutable.
   BBTC performs no allocation or hidden copy.
5. **Derivatives.** The public evaluator returns `B(T)`, `B'(T)`, and `B''(T)`
   analytically in the selected native scalar family.
6. **Ideal-gas limit.** An identically zero coefficient law is valid and
   explicitly represents the ideal-gas limit.
7. **Coefficient signs.** Coefficient and evaluated-law signs are unrestricted
   by record validation. State admissibility is deferred to constitutive
   evaluation.
8. **Caloric source of truth.** The stored heat capacity is the dilute-gas
   reference `c_v,0`; future finite-density energy and heat capacity must
   include the documented `B'(T)` and `B''(T)` compatibility terms.
9. **Calibration metadata.** Every model supplies represented temperature and
   calibrated density intervals. These are applicability metadata, not
   guarantees or clipping rules.
10. **Backend coexistence.** The virial backend supplements rather than replaces
    Noble-Abel. Neither reduced backend is declared universally correct.
11. **Scope boundary.** This checkpoint adds no pressure evaluator, state
    inversion, gas-mass closure, chemistry, combustion, burn law, ODE solver,
    firing prediction, trajectory, or safety judgment.


## 31. Decisions deferred to later checkpoints

The following choices remain deliberately deferred:

1. **History delivery API.** Caller buffer, synchronous callback, or both will
   be chosen during the public simulation-API review.
2. **Supported compiler and platform matrix.** This will be expanded only from
   recorded builds and tests rather than inferred from language claims.
3. **Exact CLI exit-code table and machine-readable schemas.** Their categories
   are constrained here, but their concrete representation belongs to the CLI
   contract.

## 32. Acceptance criteria for IB0.1

IB0.1 is complete when:

- this contract has been reviewed and explicitly accepted;
- unresolved wording has been revised or recorded as a deferred decision;
- the document is committed alone as a documentation-only change;
- no legacy implementation file has been deleted or rewritten in the same
  commit;
- no physics result is claimed merely because the future architecture is
  described here.

IB0.2a turned the accepted rules into the smallest possible CMake library
skeleton. IB0.2b replaces its private link anchor with the public status API
without pretending to simulate internal ballistics. IB0.2c defines how a future
simulation reports termination, warnings, and model-applicability limitations
while still producing no physical result.
IB0.3a adds explicit scalar-family identity and host precision metadata while
still introducing no ballistic problem, solver, or physical result. IB0.3b adds
the first precision-qualified physical input component and validates its
mathematical domain without pretending to solve internal ballistics. IB0.3c
adds native projectile-mass records and validation while preserving geometry,
initial-state, solver, and result boundaries. IB0.3d adds primitive
propellant-charge mass and condensed-phase-density records without
pretending that charge fit, burn behavior, pressure, or velocity has been
computed. IB0.3e composes the first three physical records, derives initial
volumes from one source of truth, and rejects mathematically impossible volume
relationships without claiming a safe or validated firing solution. IB0.3f adds
explicit initial free-gas absolute-pressure and temperature boundary conditions
without inventing a missing gas model or deriving one quantity from the other.
IB0.3g adds the constant calorically perfect Noble-Abel gas-model parameters and
their domains while leaving gas-mass closure and pressure evaluation for later
composition checkpoints.

## 33. Acceptance criteria for IB0.2b

IB0.2b is complete when:

- the public headers compile as ISO C23 and as C++11 or newer;
- `bbtc_status_e` preserves the original values 0 through 9 documented in
  section 9.1 with the current `uint8_t` representation; later append-only
  pre-1.0 status extensions do not renumber those values;
- `bbtc_status_string()` returns the exact documented string for every known
  status and the documented fallback for representative unknown values;
- status lookup requires no allocation and uses no mutable global state;
- the temporary private build-anchor symbol and its smoke test are removed;
- GCC and Clang builds pass the registered status and C++ compatibility tests;
- the public meaning and unknown-value behavior are documented; and
- no solver, physical result, warning, termination, applicability, or safety
  API is introduced.

## 34. Acceptance criteria for IB0.2c

IB0.2c is complete when:

- the public diagnostic header compiles as ISO C23 and as C++11 or newer;
- `bbtc_ib_termination_e` has `uint8_t` representation and the exact values in
  section 9.2;
- warning and applicability aggregate masks are exactly `uint64_t`;
- individual warning and applicability enumeration types have `uint64_t`
  representation and the exact one-bit assignments in sections 9.3 and 9.5;
- every defined termination, warning, and applicability value returns its exact
  documented string;
- representative unknown termination values, combined flag values, and
  unassigned high bits return their documented fallback strings;
- string lookup requires no allocation and uses no mutable global state;
- GCC and Clang builds pass the status, diagnostic, and C++ compatibility
  tests;
- result-field validity remains explicitly deferred until result fields are
  reviewed; and
- no solver, result structure, physical prediction, or safety judgment is
  introduced.


## 35. Acceptance criteria for IB0.3a

IB0.3a is complete when:

- `bbtc_precision_e` has `uint8_t` representation and the exact three scalar
  family values documented in section 7.1;
- runtime precision strings and host metadata match the linked build;
- C and C++ consumers can use the precision declarations;
- strict GCC, strict Clang, AddressSanitizer, and UndefinedBehaviorSanitizer
  verification pass; and
- no ballistic problem, solver, or physical result is introduced.

## 36. Acceptance criteria for IB0.3b

IB0.3b is complete when:

- all three geometry records expose the exact fields and meanings in section
  10.1 using native `float`, `double`, and `long double` members;
- the public umbrella-header chain exposes geometry to C23 and C++11-or-newer
  consumers;
- each concrete validator reports null, NaN, infinity, and finite out-of-domain
  inputs with the required status and accepts finite positive values;
- unequal effective bore and projectile-base areas remain valid;
- validation leaves the caller-owned record unchanged and allocates no memory;
- tests cover each field and scalar family, including NaN, both infinities,
  positive subnormal values, and finite maxima;
- strict GCC, strict Clang, independent-consumer, AddressSanitizer, and
  UndefinedBehaviorSanitizer verification pass; and
- no composed problem, solver, physical prediction, or safety judgment is
  introduced.


## 37. Acceptance criteria for IB0.3c

IB0.3c is complete when:

- all three projectile records expose exactly one native scalar `mass_kg` field
  with the physical meaning defined in section 10.2;
- the public umbrella-header chain exposes projectile declarations to C23 and
  C++11-or-newer consumers;
- each concrete validator reports null, NaN, infinity, and finite out-of-domain
  inputs with the required status and accepts finite positive mass;
- validation leaves the caller-owned record unchanged and allocates no memory;
- tests cover every scalar family, including NaN, both infinities, zero,
  negative values, positive subnormal values, and finite maxima;
- strict GCC, strict Clang, independent-consumer, AddressSanitizer, and
  UndefinedBehaviorSanitizer verification pass; and
- no composed problem, initial-state record, solver, physical prediction, or
  safety judgment is introduced.

## 38. Acceptance criteria for IB0.3d

IB0.3d is complete when:

- all three propellant-charge records expose native scalar `charge_mass_kg` and
  `condensed_phase_density_kg_per_m3` fields with the meanings defined in
  section 10.3;
- the public umbrella-header chain exposes propellant-charge declarations to
  C23 and C++11-or-newer consumers;
- each concrete validator reports null, NaN, infinity, and finite out-of-domain
  inputs with the required status and accepts finite positive values;
- validation leaves the caller-owned record unchanged and allocates no memory;
- tests cover both fields in every scalar family, including NaN, both
  infinities, zero, negative values, positive subnormal values, and finite
  maxima;
- the contract distinguishes condensed material density from bulk loading
  density and records the derived material-volume source of truth;
- strict GCC, strict Clang, independent-consumer, AddressSanitizer, and
  UndefinedBehaviorSanitizer verification pass; and
- no composed problem, free-gas-volume result, burn law, thermochemistry, grain
  model, solver, physical prediction, or safety judgment is introduced.

## 39. Acceptance criteria for IB0.3e

IB0.3e is complete when:

- all three loading-state records own matching native geometry, projectile, and
  propellant-charge components by value;
- all three derived-volume records expose native scalar condensed-propellant and
  initial-free-gas volume fields;
- the public umbrella-header chain exposes loading-state declarations to C23 and
  C++11-or-newer consumers;
- each concrete evaluator validates components in the required order and
  propagates the first non-success status;
- null-pointer behavior and output clearing match section 10.4;
- valid inputs derive condensed volume from mass divided by density and derive
  free-gas volume from geometric volume minus condensed volume;
- exact fill, overfill, arithmetic underflow to zero, and other nonpositive or
  nonrepresentable derived-volume cases are rejected;
- validation leaves every caller-owned input component unchanged and allocates
  no memory;
- strict GCC, strict Clang, independent-consumer, AddressSanitizer, and
  UndefinedBehaviorSanitizer verification pass; and
- no burn law, thermochemistry, grain model, ignition model, resistance model,
  integration state, solver, pressure prediction, velocity prediction, or
  safety judgment is introduced.

## 40. Acceptance criteria for IB0.3f

IB0.3f is complete when:

- all three initial gas-state records expose native scalar
  `absolute_pressure_pa` and `temperature_k` fields with the meanings defined in
  section 10.5;
- the public umbrella-header chain exposes initial gas-state declarations to C23
  and C++11-or-newer consumers;
- each concrete validator reports null, NaN, infinity, and finite out-of-domain
  inputs with the required status and accepts finite positive values;
- validation leaves the caller-owned record unchanged and allocates no memory;
- tests cover both fields in every scalar family, including NaN, both
  infinities, zero, negative values, positive subnormal values, and finite
  maxima;
- the contract records that pressure and temperature are explicit initial
  boundary conditions and are not derivable from each other without additional
  state information and an equation-of-state model;
- strict GCC, strict Clang, independent-consumer, AddressSanitizer, and
  UndefinedBehaviorSanitizer verification pass; and
- no gas composition, gas quantity, equation of state, thermochemistry, energy
  balance, burn law, solver, pressure evolution, velocity prediction, result
  record, or safety judgment is introduced.

## 41. Acceptance criteria for IB0.3g

IB0.3g is complete when:

- all three Noble-Abel gas-model records expose native scalar
  `specific_gas_constant_j_per_kg_k`,
  `constant_volume_specific_heat_j_per_kg_k`, and `covolume_m3_per_kg` fields
  with the meanings in section 10.6;
- the public umbrella-header chain exposes the explicitly named Noble-Abel
  backend declarations to C23 and C++11-or-newer consumers;
- each concrete validator reports null, NaN, infinity, and finite out-of-domain
  inputs with the required status;
- finite positive `R` and `c_v` are required, zero covolume is accepted as the
  explicit ideal-gas limit, and negative covolume is rejected;
- validation leaves the caller-owned record unchanged and allocates no memory;
- tests cover every field and scalar family, including NaN, both infinities,
  zero, negative values where invalid, positive subnormal values, finite maxima,
  and the zero-covolume ideal-gas limit;
- the contract defines the Noble-Abel mechanical equation, constant-`c_v`
  caloric closure, derived `c_p` and `gamma`, and future initial-gas-mass
  relation without storing redundant inputs;
- the contract identifies Noble-Abel as one reduced backend, distinguishes
  initial fill gas from combustion-product gas, leaves room for virial and
  thermochemical backends, and separates numerical error from prediction
  uncertainty;
- strict GCC, strict Clang, independent-consumer, AddressSanitizer, and
  UndefinedBehaviorSanitizer verification pass; and
- no gas-mass evaluator, thermochemistry, combustion-product generation, grain
  model, burn law, energy integration, pressure evolution, solver, result
  record, or safety judgment is introduced.

## 42. Acceptance criteria for IB0.3h

IB0.3h is complete when:

- native `float`, `double`, and `long double` temperature-law, evaluated-term,
  and gas-model records are public through the umbrella-header chain;
- every public field has documented SI units, physical meaning, ownership, and
  applicability semantics;
- the Chebyshev coefficient convention and temperature normalization are
  unambiguous and include no hidden half-weight rule;
- validators reject null pointers, NaN and infinite data, empty coefficient
  arrays,
  nonpositive temperature minima, unordered temperature intervals, nonpositive
  gas constants or dilute-gas heat capacities, negative minimum densities, and
  unordered density intervals;
- positive, zero, and negative finite virial coefficients are accepted;
- degree-zero constant and identically zero ideal-gas laws are accepted;
- evaluators return analytic `B(T)`, `B'(T)`, and `B''(T)` in native precision,
  accept both closed interval endpoints, reject out-of-range temperature, clear
  outputs on failure, allocate no storage, and report nonfinite recurrence
  output as numerical failure;
- tests cover all scalar families, constant and quadratic reference laws,
  ideal-gas behavior, interval endpoints, failure statuses, output clearing,
  C++ header interoperability, and independent CMake consumption;
- the contract records the future thermodynamically compatible internal-energy
  and heat-capacity relations without exposing those evaluators prematurely;
- strict GCC, strict Clang, AddressSanitizer, UndefinedBehaviorSanitizer, C++,
  and independent-consumer gates pass; and
- no pressure evolution, state inversion, chemistry, combustion, solver,
  trajectory, physical firing result, or safety judgment is introduced.

## 43. Acceptance criteria for IB0.4c

IB0.4c is complete when:

- `BBTC_STATUS_NAN_INPUT` is appended as value 10 without renumbering values
  0 through 9, and caller NaN/infinity classification follows section 9.1;
- derived nonfinite arithmetic from accepted finite inputs remains a numerical
  or internal failure rather than caller-input NaN;
- native `float`, `double`, and `long double`
  `bbtc_ib_initial_propellant_condition_*_t` records are public through the
  umbrella-header chain and contain exactly one native `temperature_k` field;
- validators report null, NaN, infinity, and finite nonpositive temperature
  with the required statuses while accepting every finite positive Kelvin
  value;
- tests cover 293.15 K, another finite positive value, the positive
  representable floor, signed zeros, a negative finite value, NaN, both
  infinities, null pointers, immutability, type/layout probes, C++ use, and an
  independent CMake consumer;
- no exact `sizeof(record) == sizeof(scalar)` ABI promise is introduced;
- propellant condition remains separate from charge, initial free gas,
  provenance, burn kinetics, thermal evolution, chemistry, and safety; and
- strict GCC, strict Clang, AddressSanitizer, UndefinedBehaviorSanitizer, C++,
  independent-consumer, and full CTest gates pass.

## 44. Acceptance criteria for IB0.4d-A

IB0.4d-A is complete when:

- section 2 records the engineering-fidelity and model-inclusion policy,
  including the four explicit questions governing whether additional physical
  complexity belongs in the default engineering model;
- section 11.4 defines the physical meaning and coexistence of normalized
  pressure-power and tabulated pressure burn-kinetics backends without hidden
  backend selection;
- the common burn-rate result contract defines applicability metadata,
  deterministic clearing after a nonnull output is accepted, and the distinction
  between successful mathematical zero rate and failed positive-rate arithmetic;
- the pressure-power contract defines absolute-pressure semantics, normalization,
  finite-domain status, validation ordering, calibration-domain behavior
  including zero-pressure applicability, exact zero-pressure and
  reference-pressure boundaries, range-robust implementation freedom, and
  numerical-failure behavior;
- the tabulated contract defines borrowed immutable point storage, structural
  validation precedence and finite-domain statuses, finite positive and strictly
  increasing pressure knots, positive burn rates, dimensionless-ratio
  log-pressure/log-rate interpolation, numerically equivalent implementation
  freedom, exact-knot behavior, and deliberate rejection of extrapolation;
- the contract preserves the separation between grain geometry, linear
  regression rate, whole-charge reacted-mass rate, ignition, thermochemistry,
  and projectile dynamics;
- temperature dependence is explicitly tied to the IB0.4c initial propellant
  condition and may affect kinetics only through a documented calibrated
  temperature-response backend;
- the first complete solver boundary includes explicit initial gas and
  propellant thermal conditions, burn kinetics, grain/charge coupling,
  thermochemical sources, gas/energy closure, a lumped pressure-gradient
  correction, explicit identification of the pressure quantity driving burn
  kinetics, forward pressure, projectile resistance, and coupled integration;
- deferred physical effects are governed by the section 2.1 evidence/ROI policy
  rather than by an assumption that greater model complexity is automatically
  better;
- provenance, fit-quality, sensitivity, uncertainty, and validation expectations
  are recorded;
- the contract has been reviewed and its status changed from `Draft` to
  `Accepted` before the checkpoint is committed; and
- IB0.4d-A changes documentation only and introduces no public symbol, source
  implementation, solver, firing prediction, or safety judgment.

## 45. Acceptance criteria for IB0.4d-B1

IB0.4d-B1 is complete when:

- `<bbtc/internal_ballistics/propellant_burn_kinetics.h>` is public through the
  internal-ballistics umbrella-header chain and is usable from C23 and C++11;
- native `float`, `double`, and `long double` normalized pressure-power model
  records expose exactly the five continuous quantities frozen by section
  11.4.1 without routing one scalar family through another;
- the common native result records expose applicability metadata and linear
  surface-regression rate with the physical meaning frozen by section 11.4;
- model validators implement null, complete-record NaN-before-infinity, and
  finite-domain precedence, including positive exponent, ordered calibration
  bounds, and reference-pressure containment;
- pressure-power evaluators clear a nonnull result before later failure, preserve
  model-validation precedence over the direct pressure argument, accept exact
  zero pressure as the documented zero-rate mathematical boundary, and preserve
  the reference-pressure burn rate exactly;
- successful evaluation outside the inclusive calibrated-pressure interval sets
  `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN` without clipping pressure;
- positive-pressure evaluation uses native-scalar, algebraically equivalent
  arithmetic intended to avoid unnecessary intermediate range loss, and a
  required positive result that becomes zero or nonfinite is reported as
  `BBTC_STATUS_NUMERICAL_FAILURE`;
- dedicated tests cover every scalar family, model validation, mixed NaN/infinity
  precedence, finite-domain failures, exact zero/reference boundaries,
  calibration endpoints, out-of-calibration applicability, deterministic output
  clearing, positive representable floors, and overflow/underflow failure;
- the independent CMake consumer exercises the new public declarations and
  linked evaluator, while the C++ public-header test verifies native field types;
- the tabulated-pressure backend, temperature-aware kinetics, ignition,
  regression-to-mass coupling, thermochemical coupling, chamber evolution,
  projectile motion, and firing prediction remain outside this increment;
- strict GCC, strict Clang, AddressSanitizer, UndefinedBehaviorSanitizer, C++,
  independent-consumer, and full CTest gates pass; and
- contract version 0.1.18 is reviewed and changed from `Draft` to `Accepted`
  before IB0.4d-B1 is committed.

## 46. Acceptance criteria for IB0.4d-B2

IB0.4d-B2 is complete when:

- native `float`, `double`, and `long double` pressure/burn point records expose
  absolute pressure in pascals and linear normal surface-regression rate in
  meters per second;
- native pressure-burn-table records borrow immutable caller-owned point storage
  with an explicit `size_t` point count and perform no allocation or
  undocumented copy;
- table validation performs structural checks first, rejects fewer than two
  points, applies complete-point-set NaN-before-infinity precedence, then
  rejects finite nonpositive pressure/rate data or pressure ordering that is not
  strictly increasing;
- burn-rate values remain free to be nonmonotonic when the supplied empirical
  data support that shape;
- table evaluation clears a nonnull result before later failure, validates the
  complete table before the direct pressure argument, accepts the closed first-
  through-last-knot pressure interval, and rejects pressure extrapolation with
  `BBTC_STATUS_OUTSIDE_DOMAIN`;
- exact first, interior, and final pressure knots return their stored burn rates
  exactly without transcendental reconstruction;
- interpolation between adjacent knots implements the section 11.4.2 piecewise
  log-pressure/log-burn-rate relation in the selected native scalar family and
  uses algebraically equivalent forms where useful to avoid unnecessary ratio
  overflow, underflow, or cancellation;
- successful table evaluation reports no fabricated applicability limitation,
  while an accepted finite calculation that cannot produce the required finite
  positive native result reports `BBTC_STATUS_NUMERICAL_FAILURE`;
- dedicated tests cover every scalar family, structural failures, whole-table
  nonfinite precedence, finite positivity and ordering, explicitly
  nonmonotonic-rate tables, exact knots, interpolation, represented-domain
  rejection, deterministic output clearing, caller-storage immutability, and
  extreme-scale native arithmetic;
- the independent CMake consumer exercises the linked public table API and the
  C++ public-header test verifies the native point, borrowed-pointer, and count
  types;
- temperature-aware kinetics, ignition, regression-to-mass coupling,
  thermochemical coupling, chamber evolution, projectile motion, catalog
  provenance infrastructure, and firing prediction remain outside this
  increment;
- strict GCC, strict Clang, AddressSanitizer, UndefinedBehaviorSanitizer, C++,
  independent-consumer, and full CTest gates pass; and
- contract version 0.1.19 is reviewed and changed from `Draft` to `Accepted`
  before IB0.4d-B2 is committed.

**Acceptance note (2026-10-02):** The IB0.4d-B2 implementation commit
`0d391d67f75b01c7dc333173e054b2b56c6d3b68` was committed and pushed while
contract version 0.1.19 still carried `Draft` status. The required verification
gates, including the repository CI workflow, passed on that implementation
commit. This follow-up acceptance records formal contract acceptance without
rewriting already-published history.

## 47. Acceptance criteria for IB0.4e-A

IB0.4e-A is a design-contract checkpoint. It is complete when:

- section 2.2 explicitly separates numerical error from model, input,
  experimental, manufacturing, and shot-to-shot uncertainty and establishes the
  engineering goal that numerical error remain materially smaller than the
  dominant physical uncertainty;
- the approximately 10:1 numerical-to-physical uncertainty separation is
  documented as a practical guideline rather than a universal guarantee;
- regression distance remains the authoritative propellant burn-progress state,
  with whole-charge reacted and remaining mass derived algebraically from the
  canonical grain state rather than independently integrated as competing state;
- the first regression-to-mass coupling consumes one validated charge record, an
  explicit positive initial canonical-grain volume, one common current grain
  state, and one common burn-kinetics result;
- the equivalent population scale is defined as a real-valued positive
  dimensionless scaling factor derived from charge mass, condensed density, and
  initial canonical-grain volume, with no integer interpretation or rounding;
- the represented population is explicitly a synchronous, monodisperse-equivalent
  approximation and does not silently claim literal grain count, size
  distribution, mixed geometry, partial ignition, or grain-fracture physics;
- the required result semantics include complete applicability metadata,
  equivalent population scale, remaining condensed-propellant volume, total
  burning surface area, remaining propellant mass, reacted propellant mass, and
  reacted-propellant mass rate;
- exact initial-state, exact burnout, and exact zero-rate semantics are frozen,
  while interior mass quantities are permitted native-rounding differences when
  independently stable forms are preferable to a bit-exact complement;
- validation precedence distinguishes structural argument failure, complete
  record NaN/infinity classification, finite scalar-domain failure, cross-record
  inconsistency, and numerical representability failure;
- negative supplied burn rate is explicitly outside the mathematical domain,
  while exact zero burn rate remains a valid zero-mass-rate state;
- successful coupling preserves every incoming burn-kinetics applicability bit
  verbatim, including future unknown bits, and introduces no new applicability
  flag in this design checkpoint;
- required positive results that cannot remain finite and strictly positive in
  the selected native scalar family are numerical failures rather than clamped
  or silently zeroed outputs;
- ignition, temperature correction, thermochemical source generation, gas/EOS
  evolution, chamber-state integration, projectile dynamics, and firing
  prediction remain outside this coupling layer;
- exact public type names, function names, and declaration layout remain
  intentionally deferred to the first concrete IB0.4e implementation increment;
- IB0.4e-A changes documentation only and introduces no public symbol, source
  implementation, CMake integration, solver, firing prediction, or safety
  judgment; and
- contract version 0.1.20 is reviewed and changed from `Draft` to `Accepted`
  before the IB0.4e-A checkpoint is committed.

## 48. Acceptance criteria for IB0.4e-B1

IB0.4e-B1 is complete when:

- `<bbtc/internal_ballistics/propellant_mass.h>` is public through the
  internal-ballistics umbrella-header chain and is usable from C23 and C++11;
- native `bbtc_ib_propellant_mass_result_float_t`, `_double_t`, and
  `_long_double_t` records expose applicability flags, equivalent population
  scale, whole-charge remaining volume, whole-charge burning area, remaining
  mass, reacted mass, and reacted-mass rate using the concise field names frozen
  by section 11.5.10;
- `bbtc_ib_propellant_mass_evaluate_float()`, `_double()`, and `_long_double()`
  consume matching charge, initial-grain-volume, grain-state, kinetics-result,
  and output families without routing one scalar precision through another;
- a nonnull output is deterministically cleared before later failure, charge
  validation precedes the explicit initial volume, complete grain-state
  NaN-before-infinity precedence is preserved, cross-record invariants are
  enforced, and burn-rate classification occurs after grain-state validation;
- exact initial state, exact burnout, and exact zero-rate behavior match section
  11.5.5, including exact zero burnout area/rate and exact initial/burnout mass
  identities;
- equivalent population remains a positive real-valued scale with no integer
  rounding and supports valid values below one;
- successful evaluation preserves every incoming applicability bit verbatim,
  including unknown future bits;
- native arithmetic uses algebraically equivalent scaling where useful to avoid
  unnecessary intermediate overflow or underflow, while any mathematically
  required positive output that is actually unrepresentable returns
  `BBTC_STATUS_NUMERICAL_FAILURE`;
- dedicated tests cover all scalar families, argument/output-clearing semantics,
  initial-volume validation, whole-grain-state nonfinite precedence, finite
  grain-state domain failures, endpoint/interior inconsistencies, burn-rate
  classification, fractional equivalent populations, applicability propagation,
  exact endpoints, zero rate, avoidable intermediate range loss, and genuine
  representability failure;
- the independent CMake consumer exercises the linked public evaluator and the
  C++ public-header test verifies native result field types;
- ignition, thermochemical source generation, gas/EOS evolution, chamber-state
  integration, projectile dynamics, and firing prediction remain outside B1;
- strict GCC, strict Clang, AddressSanitizer, UndefinedBehaviorSanitizer, C++,
  independent-consumer, and full CTest gates pass; and
- contract version 0.1.21 is reviewed and changed from `Draft` to `Accepted`
  before IB0.4e-B1 is committed.

## 49. Acceptance criteria for IB0.4f-A

IB0.4f-A is a design-contract checkpoint. It is complete when:

- section 6.1 defines canonical public naming suffixes for mass rate in
  kilograms per second and power in watts;
- section 11.6 defines thermochemical source-rate coupling as the explicit
  bridge from IB0.4e whole-charge reacted-propellant mass rate to instantaneous
  gaseous-product, condensed-product, and reaction-energy source rates;
- the first source-rate coupling consumes one matching validated reduced
  thermochemistry record and one complete successful matching IB0.4e
  propellant-mass result;
- only the upstream reacted-mass-rate scalar and applicability mask participate
  in the IB0.4f equations, while unrelated IB0.4e diagnostics are not
  incompletely revalidated without their original inputs;
- the source-rate equations, SI units, positive reaction-energy-release sign
  convention, all-gas limit, and mass-partition relation are unambiguous;
- the contract distinguishes accumulated IB0.4a extensive-source quantities
  from instantaneous IB0.4f source rates and forbids dimensionally reusing the
  cumulative public evaluator as a rate evaluator;
- exact zero reacted-mass rate succeeds with exact zero source rates while
  preserving the incoming applicability mask, and finite negative
  reacted-mass rate returns `BBTC_STATUS_OUTSIDE_DOMAIN`;
- every successful result preserves all incoming IB0.4e applicability bits
  verbatim, including future unknown bits, and IB0.4f-A introduces no new
  applicability flag;
- validation precedence freezes deterministic output clearing, existing
  thermochemistry-record validation before upstream mass-rate classification,
  NaN-before-infinity classification, zero identity handling, native
  source-rate evaluation, representability checks, and commit-on-success;
- required positive source rates that cannot be represented as finite strictly
  positive values in the selected native scalar family produce
  `BBTC_STATUS_NUMERICAL_FAILURE` rather than clipping, saturation, or silent
  zero;
- the algebraic mass-rate partition is required physically without imposing a
  bit-for-bit sum identity on independently rounded native result fields;
- thermochemical source-rate generation remains separate from ignition,
  kinetics, grain geometry, product-gas mixing, gas EOS/caloric closure,
  chamber energy balance, heat transfer, free-volume evolution, pressure
  evolution, projectile dynamics, time integration, and firing prediction;
- the first complete internal-ballistics target explicitly includes
  thermochemical gas-mass, condensed-product-mass, and reaction-energy source
  rates before evolving gas-population and chamber-energy closure;
- exact public source-rate type/function names remain intentionally deferred to
  the first concrete IB0.4f implementation checkpoint;
- IB0.4f-A changes documentation only and introduces no public symbol, source
  implementation, CMake integration, solver, firing prediction, or safety
  judgment; and
- contract version 0.1.22 is reviewed and changed from `Draft` to `Accepted`
  before IB0.4f-A is committed.
