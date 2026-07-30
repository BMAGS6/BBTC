# BBTC Reconstruction Design Contract

**Contract version:** 0.1.1

**Project phase:** IB0.2a

**Applies to:** `rewrite/library_first_v1`

**Status:** Accepted

**Date:** 2026-07-30

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

The exact complete declarations will be reviewed in IB0.2, but an unsuffixed
public `bbtc_real_t` whose meaning changes with a build definition is
forbidden. Such a switch would make headers, object files, and ABI identity
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

### 9.2 Physical termination

Internal-ballistics termination is distinct from API status. Named termination
reasons will include, as applicable:

- muzzle exit;
- no ignition;
- projectile did not begin moving;
- projectile stopped before muzzle exit;
- caller time guard reached;
- caller pressure guard reached;
- solver step guard reached;
- numerical failure.

A successfully computed stuck projectile or incomplete burn is a physical model
outcome, not automatically an API error.

### 9.3 Warning flags

Nonfatal computational or reporting conditions use a bitmask. Candidate
warnings include:

- requested history was truncated;
- energy-accounting residual exceeded its requested tolerance;
- burn remained incomplete at muzzle exit;
- a fallback approximation was used;
- an event was located with reduced accuracy;
- a supplied data record was extrapolated.

Every warning bit MUST have one stable meaning. Warning text is presentation;
the bit is the programmatic contract.

### 9.4 Result-field validity

Results MUST include a validity mask or equivalent structured mechanism. A
field not marked valid MUST NOT be consumed.

Unavailable floating-point result fields SHOULD be initialized to a quiet NaN
of the corresponding scalar family to make accidental use visible. Integer
counters and bitmasks SHOULD be initialized to zero. Callers MUST still consult
status and validity metadata; NaN is a tripwire, not the API.

### 9.5 Model applicability

Computational success **DOES NOT imply scientific validity** for every input.
Model-limit or applicability flags MUST identify conditions such as:

- use outside a parameter set's documented calibration domain;
- missing experimental validation for the selected model combination;
- assumptions materially stressed by the supplied geometry or state;
- a user-supplied parameter set with unknown provenance;
- use of an approximation in place of a requested physical effect.

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

## 22. Decisions deferred to later checkpoints

The following choices are deliberately not smuggled into IB0.1:

1. **History delivery API.** Caller buffer, synchronous callback, or both will
   be chosen during the IB0.2 public-API review.
2. **Public-struct evolution mechanism.** Struct-size/version fields versus
   pre-1.0 source-level evolution will be decided before the first public
   simulation structure is frozen.
3. **Supported compiler and platform matrix.** This will be expanded only from
   recorded builds and tests rather than inferred from language claims.
4. **Exact CLI exit-code table and machine-readable schemas.** Their categories
   are constrained here, but their concrete representation belongs to the CLI
   contract.

## 23. Acceptance criteria for IB0.1

IB0.1 is complete when:

- this contract has been reviewed and explicitly accepted;
- unresolved wording has been revised or recorded as a deferred decision;
- the document is committed alone as a documentation-only change;
- no legacy implementation file has been deleted or rewritten in the same
  commit;
- no physics result is claimed merely because the future architecture is
  described here.

The next phase, IB0.2, will turn the accepted rules into the smallest possible
CMake library skeleton and public status/diagnostic API. It will not yet pretend
to simulate internal ballistics.
