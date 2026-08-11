# BBTC Reconstruction Design Contract

**Contract version:** 0.1.13

**Project phase:** IB0.3j

**Applies to:** `rewrite/ib0_3j_initial_gas_closure_v1`

**Status:** Accepted

**Date:** 2026-08-10

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

The CLI MUST NOT contain an alternate physics implementation. All physical
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

BBTC MUST NOT encode a gameplay probability that a weapon explodes. A future
material-stress module may expose physical loads or estimated failure margins,
but it still MUST NOT label a real cartridge or firearm as safe.

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
- non-finite input;
- value outside the mathematical domain;
- inconsistent geometry or configuration;
- unsupported model or option;
- insufficient caller-provided storage;
- numerical failure;
- iteration or step limit;
- internal invariant failure.

Status value zero is success. A nonzero status MUST never mean "success with a
physical warning."

IB0.3a fixes the underlying representation of `bbtc_status_e` as `uint8_t`
and retains the following stable values and nonlocalized strings. This
pre-release revision replaces the earlier IB0.2b `uint32_t` choice without
renumbering any value:

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
- `BBTC_STATUS_NONFINITE_INPUT` when any field is NaN or infinite;
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
- `BBTC_STATUS_NONFINITE_INPUT` when `mass_kg` is NaN or infinite;
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
- `BBTC_STATUS_NONFINITE_INPUT` when either field is NaN or infinite;
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
- `BBTC_STATUS_NONFINITE_INPUT` when either field is NaN or infinite;
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
- `BBTC_STATUS_NONFINITE_INPUT` when any field is NaN or infinite;
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
nonnull output pointer is received. A nonfinite temperature returns
`BBTC_STATUS_NONFINITE_INPUT`; a finite temperature outside the closed interval
returns `BBTC_STATUS_OUTSIDE_DOMAIN`; and nonfinite recurrence or derivative
output returns `BBTC_STATUS_NUMERICAL_FAILURE`.

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
Finite-input arithmetic that cannot produce finite outputs returns
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
`BBTC_STATUS_INVALID_ARGUMENT`. Nonfinite scalar inputs return the existing
nonfinite-input status through the appropriate validation layer. A zero or
negative initial free-gas volume is outside the closure domain.

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

The initial pressure-dependent linear burn law is expressed in normalized form:

```c
burn_rate_m_per_s = coefficient_m_per_s * pow(pressure_pa / reference_pressure_pa, pressure_exponent)
```

The reference pressure MUST be positive and explicit. The coefficient's meaning
must not change when a user chooses a different display unit.

Relative quickness charts and marketing powder names **MUST NOT** be treated as
direct substitutes for calibrated burn-law and thermochemical data. A future
catalog layer may map a named product to a versioned parameter set with source,
lot assumptions, uncertainty, and applicability metadata.

## 12. Initial physical model

The first complete internal-ballistics model is a zero-dimensional,
lumped-parameter model. Its target scope is:

- spatially uniform mean gas state;
- pressure-dependent propellant surface regression;
- burn-surface evolution from grain geometry;
- a nonideal Noble-Abel-style gas equation of state;
- an explicit energy balance;
- changing volume as the projectile moves;
- a defined projectile-start and bore-resistance model;
- adaptive integration of the coupled state;
- accurate location of projectile-start and muzzle-exit events;
- energy-accounting diagnostics;
- optional time-history sampling.

Mean chamber pressure and estimated projectile-base pressure are distinct result
concepts even when an early model makes them numerically equal. A result field
MUST NOT be marked valid until its corresponding model exists.

Individual physical effects SHOULD be independently selectable where doing so
supports testing and scientific comparison. A disabled effect must be recorded
in the model configuration or result metadata.

## 13. Deferred physical effects

The initial model deliberately does not claim to resolve:

- multidimensional gas flow or pressure waves;
- spatial primer-flame propagation;
- granular-bed gas permeability;
- grain fracture, migration, or collision;
- erosive burning;
- deterrent-coating diffusion or multi-zone chemistry;
- detailed heat transfer to case, chamber, projectile, and barrel;
- gas leakage, blow-by, or obturation failure;
- elastic or plastic case and chamber deformation;
- detailed projectile engraving deformation;
- barrel wear, erosion, or changing roughness;
- structural firearm failure;
- stochastic cartridge or firearm explosion probability;
- full rotating-system and recoil-system dynamics;
- external ballistics.

These are deferred, not forbidden. Each enters as a versioned, testable model
term rather than an invisible correction factor.

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
- lot, temperature, and test-fixture assumptions when known;
- uncertainty when known;
- calibration domain;
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
- dimensional and limiting-case checks;
- closed-bomb tests;
- energy and monotonicity invariants;
- adaptive-step convergence studies;
- separate convergence studies for `float`, `double`, and `long double`;
- cross-precision comparisons over declared input domains;
- event-location tests;
- regression tests with explained tolerances;
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
4. **Validation.** Null, nonfinite, and finite out-of-domain inputs return the
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
5. **Validation.** Null, nonfinite, and finite out-of-domain inputs return the
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
6. **Validation.** Null, nonfinite, and finite out-of-domain inputs return the
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
6. **Validation.** Null, nonfinite, and finite out-of-domain inputs return the
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
7. **Validation.** Null, nonfinite, and finite out-of-domain inputs return the
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
- `bbtc_status_e` has `uint8_t` representation and the exact values in
  section 9.1;
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
- each concrete validator reports null, nonfinite, and finite out-of-domain
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
- each concrete validator reports null, nonfinite, and finite out-of-domain
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
- each concrete validator reports null, nonfinite, and finite out-of-domain
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
- each concrete validator reports null, nonfinite, and finite out-of-domain
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
- each concrete validator reports null, nonfinite, and finite out-of-domain
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
- validators reject null pointers, nonfinite data, empty coefficient arrays,
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
