# BMAGS' Ballistic Trajectory Calculator

BBTC is a C23 ballistics library under clean reconstruction. The new design is
library-first, SI-based, allocation-conscious, explicit about model limits, and
intended to support both scientific experimentation and integration with
Cubes of Honor.

## Current status

The rewrite is in **IB0.2c**. This checkpoint contains:

- a strict C23 CMake/Ninja library target;
- the namespaced CMake alias `bbtc::bbtc`;
- top-level and subproject-aware test defaults;
- public `<bbtc/bbtc.h>`, `<bbtc/status.h>`, and
  `<bbtc/diagnostics.h>` headers;
- the fixed-width `bbtc_status_e` API-status contract;
- the fixed-width `bbtc_ib_termination_e` physical-termination contract;
- fixed `uint64_t` warning and model-applicability flag contracts;
- immutable, nonlocalized strings for statuses, termination reasons, and
  individual diagnostic flags; and
- CTest coverage for every defined value, representative unknown and combined
  values, and C++ header use.

There is still no ballistic solver, result record, result-field validity mask,
command-line application, or validated firing solution in this checkpoint.
These diagnostic types define how future results communicate; they do not
pretend that a result exists yet.

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
