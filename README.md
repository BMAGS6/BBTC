# BMAGS' Ballistic Trajectory Calculator

BBTC is a C23 ballistics library under clean reconstruction. The new design is
library-first, SI-based, allocation-conscious, explicit about model limits, and
intended to support both scientific experimentation and integration with
Cubes of Honor.

## Current status

The rewrite is in **IB0.3i**. This checkpoint contains:

- a strict C23 CMake/Ninja library target;
- the namespaced CMake alias `bbtc::bbtc`;
- top-level and subproject-aware test defaults;
- public `<bbtc/bbtc.h>`, `<bbtc/status.h>`, `<bbtc/diagnostics.h>`,
  `<bbtc/precision.h>`, and internal-ballistics geometry, projectile,
  propellant-charge, loading-state, and initial-gas-state headers;
- one-byte `bbtc_status_e` and `bbtc_ib_termination_e` contracts;
- one-byte `bbtc_precision_e` identity for `float`, `double`, and `long double`;
- queried host metadata for radix, significand precision, exponent range,
  decimal round-trip digits, and scalar storage size;
- native `float`, `double`, and `long double` internal-ballistics geometry
  records with explicit SI field meanings and validation;
- native precision-qualified projectile-mass records and validation;
- native precision-qualified propellant-charge mass and condensed-phase-density
  records with validation;
- composed precision-qualified loading states with cross-record volume
  validation and derived initial-volume outputs;
- native precision-qualified initial gas absolute-pressure and temperature
  records with validation;
- native precision-qualified calorically perfect Noble-Abel gas-model backend
  records with explicit ideal-gas-limit semantics and validation;
- native precision-qualified temperature-dependent first-order density-virial
  gas-model records, bounded Chebyshev coefficient laws backed by borrowed
  caller-owned arrays, analytic first and second temperature derivatives, and
  validation;
- explicit native precision-qualified dilute-branch caloric-reference records
  with no hidden reference-temperature or zero-energy default;
- common native precision-qualified reduced-gas thermodynamic result records;
- concrete Noble-Abel and first-order-virial thermodynamic evaluators for
  pressure, specific internal energy, state constant-volume specific heat,
  `(partial p / partial rho)_T`, and `(partial p / partial T)_rho`;
- virial state-domain checks for positive compressibility factor, positive
  isothermal pressure-density derivative, and positive finite-density
  constant-volume specific heat;
- successful virial evaluation outside the documented calibrated density
  interval with `BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN` rather than a
  fabricated hard computational failure;
- fixed `uint64_t` warning and model-applicability flag contracts;
- immutable, nonlocalized strings for statuses, termination reasons, and
  individual diagnostic flags;
- CTest coverage for every defined value, representative unknown and combined
  values, C++ header use, and independent CMake consumption; and
- GitHub Actions coverage for strict GCC, strict Clang, AddressSanitizer, and
  UndefinedBehaviorSanitizer builds.

There is still no ballistic solver, complete problem record, simulation
result record, result-field validity mask, command-line application, or
validated firing solution in this checkpoint. The loading-state and
initial-gas-state records establish initial geometry and boundary conditions.
Noble-Abel and first-order virial now both have concrete reduced-gas
thermodynamic state evaluators, but this checkpoint still does not derive gas
mass from initial pressure/volume data, model combustion or propellant burning,
integrate projectile motion, or produce a firing prediction.

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

`bbtc_ib_loading_state_float_t`, `bbtc_ib_loading_state_double_t`, and
`bbtc_ib_loading_state_long_double_t` compose matching geometry, projectile,
and propellant-charge records. Their evaluators derive condensed propellant and
initial free-gas volumes, reject zero or negative free-gas volume, clear outputs
on failure, and make no ammunition- or firearm-safety judgment.

`bbtc_ib_initial_gas_state_float_t`,
`bbtc_ib_initial_gas_state_double_t`, and
`bbtc_ib_initial_gas_state_long_double_t` carry explicit initial free-gas
absolute pressure and temperature boundary conditions in native scalar
families. Their validators reject null, nonfinite, zero, and negative inputs.
Pressure and temperature are not derived from each other without an additional
gas model and state information.

`bbtc_ib_noble_abel_gas_model_float_t`,
`bbtc_ib_noble_abel_gas_model_double_t`, and
`bbtc_ib_noble_abel_gas_model_long_double_t` carry the specific gas constant,
constant-volume specific heat, and specific covolume for a calorically perfect
Noble-Abel gas. Their validators require positive gas constant and specific
heat, permit zero covolume as the explicit ideal-gas limit, and reject negative
or nonfinite parameters.

Noble-Abel is one supported reduced equation-of-state backend, not BBTC's
permanent universal gas model. Its fields describe an effective pseudo-gas and
do not identify chemical composition, calibration range, provenance, or
predictive uncertainty. Initial trapped fill gas and propellant combustion
products are distinct gas populations; callers must not silently use one
population's parameters for the other. Future first-order virial and
higher-fidelity thermochemical reference backends remain permitted.

`bbtc_ib_first_order_virial_temperature_law_float_t`,
`bbtc_ib_first_order_virial_temperature_law_double_t`, and
`bbtc_ib_first_order_virial_temperature_law_long_double_t` borrow caller-owned
Chebyshev coefficient arrays for a mass-specific second density virial
coefficient `B(T)`. Their validators require finite ordered positive
temperature bounds, at least one finite coefficient, and a nonnull coefficient
pointer. Coefficient signs are unrestricted, and an identically zero law is the
explicit ideal-gas limit.

The matching evaluators return `B(T)`, `dB/dT`, and `d^2B/dT^2` analytically in
native precision over the closed represented temperature interval. The
first-order virial gas-model records additionally carry a positive
mixture-specific gas constant, a positive dilute-gas reference constant-volume
specific heat, and an ordered nonnegative calibrated density interval.

The first-order virial constitutive evaluator uses
`p = rho * R * T * (1 + B(T) * rho)`, requires both
`1 + B(T) * rho > 0` and
`(partial p / partial rho)_T > 0`, and rejects a nonpositive finite-density
constant-volume specific heat. Temperature-dependent `B(T)` contributes to
specific internal energy and finite-density heat capacity through its first and
second temperature derivatives.

`bbtc_ib_caloric_reference_float_t`,
`bbtc_ib_caloric_reference_double_t`, and
`bbtc_ib_caloric_reference_long_double_t` define the caller-selected dilute-gas
specific-internal-energy datum. BBTC supplies no silent 298.15 K default and
does not treat this reduced-gas datum as a chemical standard state or formation
energy.

`bbtc_ib_reduced_gas_thermodynamic_result_float_t`,
`bbtc_ib_reduced_gas_thermodynamic_result_double_t`, and
`bbtc_ib_reduced_gas_thermodynamic_result_long_double_t` return pressure,
specific internal energy, state constant-volume specific heat, the isothermal
pressure-density derivative, the constant-density pressure-temperature
derivative, and model-applicability flags. Density outside a first-order virial
model's documented calibration interval is a soft applicability condition when
the represented state remains mathematically and thermodynamically admissible.
The represented Chebyshev temperature interval remains a hard evaluation
domain because BBTC does not extrapolate the coefficient law.

Noble-Abel and first-order virial coexist as reduced backends. Neither model is
declared universally accurate; parameters require documented gas-population
identity, provenance, calibration conditions, and uncertainty before physical
accuracy claims are made.

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
