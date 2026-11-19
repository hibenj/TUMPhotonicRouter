# Coding standards

Status: **draft, proposed 2026-11-19; not yet adopted.** The rules marked
*proposed* become binding when the repository owner adopts this document;
the rules marked *in force* are already checked by CI today.

This is the one normative document for how code in this repository is
written. `CONTRIBUTING.md` says how to build, test and commit;
`docs/ARCHITECTURE.md` says where things are; this page says what a change
must satisfy. Where a rule can be checked by a tool, the tool is the rule and
this page only names it. Section 8 lists where the code does not meet the
rules yet, with numbers that a check holds.

## 1. Reference standards

These are adopted as written. This page adds only what they do not decide.

| Area | Standard |
| --- | --- |
| Python style | [PEP 8](https://peps.python.org/pep-0008/), as implemented by `ruff format` (line length 100) |
| Python docstrings | [PEP 257](https://peps.python.org/pep-0257/), sections in the [Google style](https://google.github.io/styleguide/pyguide.html#38-comments-and-docstrings) |
| Python typing | [PEP 484](https://peps.python.org/pep-0484/) and successors, checked by `mypy` |
| Rust style | the [Rust Style Guide](https://doc.rust-lang.org/nightly/style-guide/), as implemented by `cargo fmt` |
| Rust API design | the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/) |
| Changelog | [Keep a Changelog 1.1](https://keepachangelog.com/en/1.1.0/) |
| Versions | [Semantic Versioning 2.0](https://semver.org/) from the first release on |

## 2. How a rule is enforced

Every rule below carries one of three levels. A rule without a level is not
a rule.

- **Tool**: a CI job fails (`.github/workflows/ci.yml`, locally
  `scripts/lint.sh` and `scripts/test_baseline.sh`). For a proposed Tool
  rule the job holds the count of violations (section 8) until it is zero.
- **Test**: a test in `tests/` fails.
- **Review**: a reviewer checks it. A pull request template that lists these
  rules is proposed and does not exist yet.

**Exceptions.** The limits are not loosened; a function that has a reason
to exceed one carries an exception at the function: a `# noqa: <rule>` or
`#[allow(clippy::<lint>)]` with a comment that names the reason (per-file
ignores in `pyproject.toml` for whole categories such as the tests). An
exception without a reason is a finding. Reasons that hold:

- a long, flat declaration without logic (the command-line arguments);
- a state machine or case analysis of the kernel that a split would tear
  apart (the primitive expansion of the search);
- a signature that the Python/Rust binding dictates.

"It grew that way" is not a reason; existing violations are covered by the
ratchet of section 8, not by exceptions. The number of exceptions is pinned
like the violation counts: a commit that adds one says why in its message.

## 3. Architecture

1. **The pipeline is a sequence of stages.** A stage is a function with a
   typed input and a typed result (a frozen dataclass, or a Rust struct). It
   does not read the command line, the environment or another stage's
   internals. `docs/ARCHITECTURE.md` section 2 is the list of stages.
   *Review; in force for new stages (`placement_analysis.py` is the
   pattern), proposed for the existing ones.*
2. **A stage hands its result on as a value.** New code does not add a field
   to a shared mutable record (`routing/state.py`) to pass data to a later
   stage. *Review, proposed.*
3. **One implementation per concern.** Before adding a helper, search for
   the one that exists. A second producer of the same result is a finding,
   also when both are correct. *Review, in force (`CONTRIBUTING.md`).*
4. **Dependencies point one way.** `cli` -> `flow` -> stage packages
   (`routing`, `crossings`, `plm`, `electrical`, `verify`, `gds`) ->
   `records`, `config`. No package imports from one to its left.
   `router-core` has no Python dependency. *Test for the Rust side (`cargo
   test -p router-core` needs no Python), proposed as a test for Python.*
5. **Names starting with an underscore stay in their module.** Another
   module imports only public names. *Test, proposed.*
6. **Configuration has one path.** Nothing below `config_loading.py` reads
   the environment and nothing below `cli.py` sees a command line. A field
   exists in both configuration mirrors (`config.py`,
   `crates/router-core/src/config.rs`). *Tool for the generated reference,
   review for the rest; in force.*
7. **The three paper configurations do not leak into each other.** The
   baseline never receives a crossing plan. *Test, in force
   (`tests/test_placement_analysis.py`).*

## 4. Python

1. Formatting and the lint set of `pyproject.toml` (`ruff format`,
   `ruff check`). *Tool, in force.*
2. Every function has annotated parameters and an annotated return type;
   `mypy` passes on the package. *Tool, proposed (mypy runs today but does
   not fail the build).*
3. A record that crosses a module boundary is a dataclass, not a
   `dict[str, object]`. A dictionary is for a mapping whose keys are data.
   *Review, proposed.*
4. A function has at most 7 parameters (ruff `PLR0913`); parameters that
   travel together become a dataclass. *Tool, proposed.* A record bundles
   what belongs together in the domain: moving fifty parameters into one
   record of fifty fields satisfies the tool and not the rule. A record
   with more than about 15 fields is questioned in review. *Review,
   proposed.*
5. A function has at most 50 statements (ruff `PLR0915`) and a cyclomatic
   complexity of at most 10 (ruff `C901`). *Tool, proposed.* The statement
   limit does not apply to `tests/`.
6. Library code reports through `logging`; `print` is for `cli.py` and the
   scripts (ruff `T201`). The `SUMMARY` line stays as it is. *Tool,
   proposed.*
7. Every public module, class and function has a docstring that says what
   it is for and what it assumes (ruff `D1xx`). *Tool, proposed.*
8. Comments explain why, not what. A comment that restates the code is
   removed. *Review, in force.*
9. An error message names what failed with the values that identify it (net
   name, coordinates, the benchmark). `except Exception` needs a comment
   saying why nothing narrower works (ruff `BLE001`). *Tool, proposed.*

## 5. Rust

1. `cargo fmt` and `cargo clippy --workspace --all-targets -- -D warnings`.
   *Tool, in force.*
2. The two workspace-level allows (`too_many_arguments`, `type_complexity`)
   are debt, not policy: new code does not rely on them. *Review, in force;
   removing the allows is proposed.*
3. Library code in `router-core` does not call `unwrap()`. An invariant is
   stated with `expect("why this cannot fail")`; a failure the caller can
   act on is a `Result`. *Tool (`clippy::unwrap_used`), proposed.*
4. Every `pub` item of `router-core` has a doc comment
   (`#![warn(missing_docs)]`). *Tool, proposed.*
5. A function body stays under 120 lines (`clippy::too_many_lines`).
   *Tool, proposed.*

## 6. Tests

1. Three tiers: the fast suite (`-m "not e2e"`, under 60 s), the end-to-end
   runs (`tests/e2e/`), and the gates (`gate_short.sh`, `plm_gate.sh`).
   *Tool, in force.*
2. Every stage has a contract test: what it accepts, what it returns, what
   it refuses, and who may call it. *Review, proposed
   (`tests/test_placement_analysis.py` is the pattern).*
3. A change that moves a pinned number re-pins it in the same commit with
   the reason. *Review, in force (`CONTRIBUTING.md`).*
4. Synthetic layouts live in `tests/fixtures/`; a test does not assert a
   whole benchmark's literal numbers when a behaviour says the same.
   *Review, in force for new tests.*

## 7. Documentation

1. `docs/ARCHITECTURE.md` describes the tree as it is; a change that moves
   a module or a stage updates it in the same commit. *Review, in force.*
2. Every stage has a contract section: input, result, assumptions,
   refusals. *Review, proposed.*
3. A user guide (install, route a benchmark, read the results, add a
   benchmark) and a developer guide (add a stage, a search policy, a
   verifier contract) exist and are tested by following them. *Review,
   proposed; neither exists yet.*
4. Generated files are never edited by hand (`CONTRIBUTING.md`,
   "Generated files"). *Tool, in force.*
5. Every behaviour change has a `CHANGELOG.md` entry. *Review, in force.*

## 8. Where the code stands

Production Python under `python/photonic_router/` (46,110 lines, 109 files)
and `crates/router-core`. `scripts/quality_ratchet.py` measures the counts
below and `tests/baselines/quality_ratchet.json` pins them; the numbers here
are that file's on 2026-11-19, and the file is the current truth.

| Rule | Measure (name in the pin) | 2026-11-19 |
| --- | --- | --- |
| 3.2 | fields of the shared session state (`session_state_fields`) | 82 |
| 3.5 | underscore names imported from another module (`private_imports`) | 247 |
| 4.2 | parameters without an annotation (`ruff_ANN001`) | 230 |
| 4.2 | functions without a return annotation (`ruff_ANN201`, `ruff_ANN202`) | 3, 6 |
| 4.2 | mypy errors on the whole package (`mypy_errors`) | 348 |
| 4.3 | `dict[str, object]` / `dict[str, Any]` annotations (`untyped_dict_annotations`) | 222 |
| 4.4 | functions over 7 parameters (`ruff_PLR0913`) | 58; the largest 50 |
| 4.5 | functions over 50 statements (`ruff_PLR0915`) | 43 |
| 4.5 | functions over complexity 10 (`ruff_C901`) | 70 |
| 4.6 | `print` calls in the package (`ruff_T201`) | 157 |
| 4.7 | public classes, methods, functions without a docstring (`ruff_D101`, `ruff_D102`, `ruff_D103`) | 56, 136, 77 |
| 4.9 | blind `except Exception` (`ruff_BLE001`) | 35 |
| 5.2 | workspace-level clippy allows (`rust_workspace_debt_allows`) | 2 |
| 5.3 | `unwrap()` in the library of `router-core`, tests excluded (`rust_unwrap_used`) | 1 |
| 5.4 | public items of `router-core` without a doc comment (`rust_missing_docs`) | 796 |
| 5.5 | functions of `router-core` over 120 lines (`rust_too_many_lines`) | 23 |
| 2 | exceptions written at a function (`python_exceptions`, `rust_exceptions`) | 0, 39 |
| 7.3 | user guide, developer guide (not measured) | none |

The 39 Rust exceptions are per-function `allow(clippy::too_many_arguments)`
attributes from before this document. mypy's informational run of
`pyproject.toml` covers a narrower file list (236 errors there); the rule is
about the package, so the package is what is pinned.

**Ratchet.** Until a proposed Tool rule holds everywhere, its count is
pinned and may only fall. `scripts/quality_ratchet.py` (run by
`scripts/lint.sh` and CI) fails when a count rose, and also when one fell
and the pin was not rewritten, so that a gain cannot be spent again:

```bash
.venv/bin/python scripts/quality_ratchet.py            # check
.venv/bin/python scripts/quality_ratchet.py --update   # pin the lower counts
```

`--update` refuses a higher count unless the measure is named with
`--allow-rise`; that is how a new exception of section 2 is recorded, and
the commit message gives its reason. The limits are configuration
(`[tool.ruff.lint.pylint]` and `[tool.ruff.lint.mccabe]` in
`pyproject.toml`, `clippy.toml`). When a count reaches zero the rule moves
into the lint configuration and leaves this table and the pin.
