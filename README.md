# BMAGS' Ballistic Trajectory Calculator

BBTC is a C23 ballistics library under clean reconstruction. The new design is
library-first, SI-based, allocation-conscious, explicit about model limits, and
intended to support both scientific experimentation and integration with
Cubes of Honor.

## Current status

The rewrite is in **IB0.3d**. This checkpoint contains:

- a strict C23 CMake/Ninja library target;
- the namespaced CMake alias `bbtc::bbtc`;
- top-level and subproject-aware test defaults;
- public `<bbtc/bbtc.h>`, `<bbtc/status.h>`, `<bbtc/diagnostics.h>`,
  `<bbtc/precision.h>`, and internal-ballistics geometry, projectile, and
  propellant-charge headers;
- one-byte `bbtc_status_e` and `bbtc_ib_termination_e` contracts;
- one-byte `bbtc_precision_e` identity for `float`, `double`, and `long double`;
- queried host metadata for radix, significand precision, exponent range,
  decimal round-trip digits, and scalar storage size;
- native `float`, `double`, and `long double` internal-ballistics geometry
  records with explicit SI field meanings and validation;
- native precision-qualified projectile-mass records and validation;
- native precision-qualified propellant-charge mass and condensed-phase-density
  records with validation;
- fixed `uint64_t` warning and model-applicability flag contracts;
- immutable, nonlocalized strings for statuses, termination reasons, and
  individual diagnostic flags;
- CTest coverage for every defined value, representative unknown and combined
  values, C++ header use, and independent CMake consumption; and
- GitHub Actions coverage for strict GCC, strict Clang, AddressSanitizer, and
  UndefinedBehaviorSanitizer builds.

There is still no ballistic solver, composed problem record, result record,
result-field validity mask, command-line application, or validated firing
solution in this checkpoint. Geometry, projectile, and propellant-charge
validation establish public physical-input components; they do not compute
pressure, velocity, burn rate, or another prediction.

The accepted reconstruction rules live in
[`docs/design/BBTC_DESIGN_CONTRACT.md`](docs/design/BBTC_DESIGN_CONTRACT.md).
The original implementation remains recoverable from the
`legacy-pre-rewrite` Git tag.

## Build and test

Requirements:

- CMake 3.22 or newer;
- Ninja;
- a C compiler with ISO C23 support; and
- optionally, a C++11 compiler for the public-header compatibility test.

Configure, build, and test:

```bash
cmake \
    -S . \
    -B build/debug \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DBBTC_BUILD_TESTS=ON

cmake --build build/debug

ctest \
    --test-dir build/debug \
    --output-on-failure
```

`BBTC_BUILD_TESTS` defaults to `ON` when BBTC is the top-level project and
`OFF` when BBTC is included by another CMake project.

## Embedding

The foundation already supports the intended CMake integration shape:

```cmake
add_subdirectory(path/to/BBTC)
target_link_libraries(your_target PRIVATE bbtc::bbtc)
```

A consumer can then include the umbrella header:

```c
#include <bbtc/bbtc.h>
```

`bbtc_status_e` reports whether an API call completed successfully.
`bbtc_status_string()` returns immutable text for known and unknown status
values.

`bbtc_precision_e` identifies the three first-class scalar families.
`bbtc_precision_info()` reports the actual `<float.h>` and storage properties
of the linked library build. This runtime identity is metadata; future
simulations remain precision-qualified at compile time.

`bbtc_ib_geometry_float_t`, `bbtc_ib_geometry_double_t`, and
`bbtc_ib_geometry_long_double_t` describe the same four geometric quantities in
native scalar families. Their concrete validators reject null, nonfinite, zero,
and negative inputs without requiring the bore and projectile-base areas to be
equal.

`bbtc_ib_projectile_float_t`, `bbtc_ib_projectile_double_t`, and
`bbtc_ib_projectile_long_double_t` carry the total modeled projectile-assembly
mass in native scalar families. Their validators reject null, nonfinite, zero,
and negative mass without inventing a default projectile.

`bbtc_ib_propellant_charge_float_t`,
`bbtc_ib_propellant_charge_double_t`, and
`bbtc_ib_propellant_charge_long_double_t` carry initial charge mass and
condensed propellant material density in native scalar families. Their
validators reject null, nonfinite, zero, and negative inputs without treating
bulk loading density or a commercial powder name as sufficient physical data.

`bbtc_ib_termination_e` independently describes why a future
internal-ballistics simulation stopped. `bbtc_warning_flags_t` and
`bbtc_applicability_flags_t` carry nonfatal computational conditions and
scientific model limitations. A successful status, a zero warning mask, or a
zero applicability mask does not establish that ammunition or a firearm is
safe.

## Safety

BBTC produces experimental model output, not measured pressure data or approved
ammunition-loading instructions. It **MUST NEVER** be used to declare a real
cartridge, charge, firearm, or procedure safe. Read
[`DISCLAIMER.md`](DISCLAIMER.md) before using future simulation output.

## License

BBTC is publicly available for noncommercial purposes under the
[PolyForm Noncommercial License 1.0.0](LICENSE.md). Commercial use requires a
separate written license or explicit written permission from L. Brandon Magoni.
See [`LICENSING.md`](LICENSING.md) for details and contact information.
