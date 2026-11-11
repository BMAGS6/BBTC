# VV0.1-B2b.3c.3b — Evidence manifest

| Field | Value |
| --- | --- |
| Record kind | Captured compiler/test/disassembly evidence inventory |
| Subject source commit | `ee39cf2040574bb004e4465ae097340e431558a4` |
| Evidence source | `bbtc_vv0_1_b2b_3c3b_ee39cf2_evidence_complete.tar.gz` (retained externally) |
| Complete archive SHA-256 | `f5fcec44f6b75229a04ef27a4a1d543e42a3f46db7e318d07f1de7dd666548f9` |
| Archive contents | Three configurations, six evaluator disassemblies per configuration, compile databases, JSON capture, logs, hash records |
| Integrity boundary | Verified uploaded archive checksum, JSON/database hashes, records and disassembly text. Object and executable **bytes are not in the archive**. |
| Status | Evidence recorded; no automatic compiler/ABI/scientific qualification |

## Profile-level evidence

| Profile | Toolchain | Build target | CTest | Focused cmocka | Captured producer verdict |
| --- | --- | --- | --- | --- | --- |
| `gcc_strict` | `gcc (GCC) 16.2.1 20260810` | `x86_64-pc-linux-gnu` | 31/31 | 25/25 | `FLAGS_SCREENED_UNQUALIFIED` |
| `clang_strict` | `clang version 23.1.4` | `x86_64-pc-linux-gnu` | 31/31 | 25/25 | `FLAGS_SCREENED_UNQUALIFIED` |
| `clang_sanitizers` | `clang version 23.1.4` | `x86_64-pc-linux-gnu` | 31/31 | 25/25 | `FLAGS_SCREENED_UNQUALIFIED` |

All configurations: Linux x86-64; kernel `7.2.14-arch1-1`, glibc `2.44` as reported by the capture tool.
The source revision and six monitored source digests agree across the three profiles.
Strict producer flags: `-O1 -g -frounding-math -ffp-contract=off -fno-fast-math`; the sanitizer profile additionally uses `-fsanitize=address,undefined -fno-omit-frame-pointer` and sanitizer link flags.

## Compile database and producer identity

| Profile | `compile_commands.json` SHA-256 | Production evaluator object SHA-256 |
| --- | --- | --- |
| `gcc_strict` | `0961eaab12c06c1abd57952e5cee9ade186f5f491f2f41705689529650d4eda2` | `7e9285d31428fd7b6f7195f99d2c60855766680564c7d65d3b9d7f5c8d9930ff` |
| `clang_strict` | `c84c97419ccb5194ddf1688d3139e0b48d0e1bf2b29de78e40e8e6ab8f123fa0` | `03120b00e25c643abc4a36007de7707d09ed8c278310863c3dae73c17b4eaaa2` |
| `clang_sanitizers` | `dd33c2533925036162449e9bc8bf9779c5c2d6cde7f9e8629d6aced466e8ff8f` | `2bba151501fc9da2f0130e1598fa2ff13332c5f7199683a91dffdadd8b1e8a8c` |

Production source: `src/internal/first_order_virial_gas_model.c`.
Its recorded source SHA-256 is `da6c511a96e3d1b5ce38427ccc93b180df80b187d54ec8dc70276d73408ce518` in all profiles.

## Four hashed linked artifacts per profile

Hashes below were extracted from the recorded hash transcript. The separate `linked-hash-verification.log` reports `OK` for all twelve local files. The archived log is **not** a substitute for hashing the original binary bytes.

### `gcc_strict`

| Artifact | Recorded SHA-256 |
| --- | --- |
| `libbbtc.a` | `cffd6c4baf4ecb15bc769a42f4a5ff49abe7f4aad51239eaeb0213e0e6b95876` |
| `bbtc_vv_test_virial_reference_matrix` | `2573db47c53b681373d34087be1869894a9b6dcaca1f2849d3be292702d8647b` |
| `bbtc_vv_test_virial_fp_profile` | `32b9f23dacb9b8f264c5044a53c580fea1aa3cb540708304e80b7beb27595989` |
| `bbtc_vv_test_virial_fp_control` | `8735cf79b4fdde0f9544e1cf8e13d44bbc850a91172a3cc3929855915bba1356` |

### `clang_strict`

| Artifact | Recorded SHA-256 |
| --- | --- |
| `libbbtc.a` | `efde0d7191b4ac2f0ffc555131e65c3eaeed5da638aef77a6f584e2004705577` |
| `bbtc_vv_test_virial_reference_matrix` | `56e64f50afa978c8e3dfab54ca2c943826573d37089fc3fc77fac3e5572e4ec2` |
| `bbtc_vv_test_virial_fp_profile` | `dbe6dc4ce770f7115d819c45d1fb758523006a991c5d6e00040ebeebe1c45e0c` |
| `bbtc_vv_test_virial_fp_control` | `a209bdc71fbfd9386a870eb02d69a12c6b64c62b394131781ef22e416babc815` |

### `clang_sanitizers`

| Artifact | Recorded SHA-256 |
| --- | --- |
| `libbbtc.a` | `a8cf69c2edc384ba9ab13763b2411f3237dba7cb13122a8c641bbb9d9addc4c6` |
| `bbtc_vv_test_virial_reference_matrix` | `133d4e37681b03fb30b72f094beec20065fbd07462d74505e6a483126fdeb4fc` |
| `bbtc_vv_test_virial_fp_profile` | `27da32326613cd24d8ecdc54a935846d70a96e493e9fb0e702e098ee663b4360` |
| `bbtc_vv_test_virial_fp_control` | `68e1c5c0e5bc55a209a880ef20d4b536dfb11b9631c4a0c6872dd8bd8d487f7b` |

## Six monitored translation-unit object hashes

Each compiled-unit record has no required-flag omissions and no suspicious options under the screening policy. Order is by evidence role.

| Role | GCC strict object SHA-256 | Clang strict object SHA-256 | Clang sanitizer object SHA-256 |
| --- | --- | --- | --- |
| `b1_comparison` | `30f28dbc2d72f087c470a8b3725f644a759b134ca3d4403a1104b9319423e746` | `a5ce9eecd6159c53a0b15d9061cc92f94bad59a419bf391d66aedfdef42c469f` | `ca2dbde2b363d54083c7a2413be983ace1923a6446e64015cd21e176b18411f9` |
| `fp_preflight` | `d1b0774606b0c29d811e1878229191249fa4531dae550cb7e4219b33ec3acf28` | `eecd5b8ccfb2c7c2294eea071e601ccbe65c0f4fb67eeee040ede238a1d2fd1c` | `b4c246e23f6d2a3cc99ac3a7929775095bef8dad47b3a0e52ebcbcdb95a9c6c9` |
| `observation_adapter` | `823731b8404e0b86f80c2f528aeb00116cbe1380737939975bde38c1d9de7cbb` | `93478886d217778a6fc13c9a788ee8b7963e312ae668bffa8d5de4b9a55815c7` | `318c116ffe93e438166a79daca6f28b31a546e53ee77e3a74ea669c49b383227` |
| `production_virial` | `7e9285d31428fd7b6f7195f99d2c60855766680564c7d65d3b9d7f5c8d9930ff` | `03120b00e25c643abc4a36007de7707d09ed8c278310863c3dae73c17b4eaaa2` | `2bba151501fc9da2f0130e1598fa2ff13332c5f7199683a91dffdadd8b1e8a8c` |
| `reference_matrix` | `ebde9026dc39d268a27f3745ec481f97be8cf409785a20188100e8a286176797` | `c87353e078656031a045f5b338313b17126d6fbb685a644279bd4cb61361b65c` | `a552e64325c6afc10d2daa7340264502f2a97be39b2bdc6bb28085d12cfba585` |
| `runtime_control` | `11e6775253618fb421c58d754040c3a82d477ba76e054b702264688b03f449d9` | `04fb64c27dfe725d26811f0a781c3b842ab7a58ef5e3ab4e9edafce490ed0c2d` | `7ac1945fae9e705414af6035af126a616a52824eddc33a0b131d307abd76945d` |

## Disassembly coverage and comparison

The analysis compared the mnemonic of each parsed instruction in each function between production-object disassembly and final reference-executable disassembly. All nine pairs have equal mnemonic sequences. Every changed textual operand in the pairs falls into a branch/call address or RIP-relative reference category; no other operand differences were detected by this comparison. This is a *text-based review*, not a binary-equivalence proof.

| Profile | Native scalar | Matched instructions | Address/reference operand differences |
| --- | --- | ---: | ---: |
| `gcc_strict` | `float` | 163 | 41 |
| `gcc_strict` | `double` | 161 | 41 |
| `gcc_strict` | `long_double` | 223 | 32 |
| `clang_strict` | `float` | 215 | 44 |
| `clang_strict` | `double` | 223 | 45 |
| `clang_strict` | `long_double` | 435 | 38 |
| `clang_sanitizers` | `float` | 740 | 202 |
| `clang_sanitizers` | `double` | 588 | 173 |
| `clang_sanitizers` | `long_double` | 1121 | 216 |

**Reviewer caveats:** The disassemblies cover only the three named production evaluator functions, not the entire call graph. The archive contains neither `.o`/ELF bytes nor `.rodata` dumps. Relocations and constants therefore have not been independently re-hashed or reconstructed from binary sections by the reviewer. These limits must remain visible in any qualification decision.

## Runtime profile observations

All three focused test transcripts report `FLT_EVAL_METHOD=0`, C nearest rounding, x87 CW `0x037f`, MXCSR `0x00001fa0`, and hardware issue mask `0x00000000`.
The MXCSR difference from `0x1f80` is sticky inexact-result status bit 5; it is not a change in rounding, exception masks, FTZ or DAZ.

## Reproduction and change control

The complete archive is identified by SHA-256 above. `source-commit.txt`, `compile-evidence.json`, `compile_commands.json`, `reference-command-chain.txt`, `linked-artifact-hashes.txt`, `linked-hash-verification.log`, `ctest.log`, `focused-tests.log`, `configure.log`, `build.log`, and 18 function disassemblies are expected.
Preserve the original off-tree builds if independent binary hashing is needed. Source, toolchain, flags, target, ABI, fenv, linker, sanitizer, or artifact changes require an impact review and potentially a new evidence capture.
