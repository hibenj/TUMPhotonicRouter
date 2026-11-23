# Coding standards

Status: **adopted by the repository owner on 2026-11-22** (proposed
2026-11-19), on the condition that it stays close to the public standards
of section 1: a rule that departs from them is listed there as a deviation
with its reason, and a new deviation is the owner's decision. The rules
marked *in force* are checked today. The rules marked *proposed* are
binding for new code; where existing code does not meet one yet, section 8
holds the count and it may only fall.

This is the one normative document for how code in this repository is
written. `CONTRIBUTING.md` says how to build, test and commit;
`docs/ARCHITECTURE.md` says where things are; this page says what a change
must satisfy. Where a rule can be checked by a tool, the tool is the rule and
this page only names it. Section 8 lists where the code does not meet the
rules yet, with numbers that a check holds.

## 1. Reference standards

These documents are adopted as written, except for the deviations listed
below the table. This page adds only what they do not decide. Where this
page and a reference disagree, this page holds, and the disagreement is
listed here so that it is never silent. All of them were read on
2026-11-19.

| Area | Standard | Publisher |
| --- | --- | --- |
| Python style | [PEP 8](https://peps.python.org/pep-0008/), as implemented by `ruff format` | Python (PEP, active) |
| Python docstrings | [PEP 257](https://peps.python.org/pep-0257/), with the sections of the Google guide (`Args:`, `Returns:`, `Raises:`) | Python (PEP, active) |
| Python typing | [PEP 484](https://peps.python.org/pep-0484/) and successors, checked by `mypy` | Python (PEP, final) |
| Python, everything the PEPs leave open | [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html) | Google |
| Rust style | the [Rust Style Guide](https://doc.rust-lang.org/nightly/style-guide/), as implemented by `cargo fmt` | Rust project |
| Rust API design | the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/) | Rust project |
| Rust, everything those two leave open | [Pragmatic Rust Guidelines](https://microsoft.github.io/rust-guidelines/), version 2026.6; it states that it complements the two above | Microsoft |
| Changelog | [Keep a Changelog 1.1](https://keepachangelog.com/en/1.1.0/) | |
| Versions | [Semantic Versioning 2.0](https://semver.org/) from the first release on | |

**Deviations, Python.**

- Lines are 100 characters long. PEP 8 says 79 and lets a team agree on up
  to 99; the Google guide says 80. 100 is the width of the Rust Style
  Guide, so both languages share one number.
- The formatter and linter are `ruff`. The Google guide names `pylint` and
  Black or Pyink.
- A module may import names: `from module import Name`. The Google guide
  (2.2) allows imports of packages and modules only. The code is written
  with name imports throughout; rule 3.5 limits which names may be
  imported instead.
- Stricter than the Google guide: every function is annotated (rule 4.2;
  the guide asks for at least the public API), and function size has
  limits a tool checks (rule 4.5; the guide gives a hint of about 40
  lines and no limit).

**Deviations, Rust.**

- The lint set is the one of section 5, not the larger set the Microsoft
  guidelines recommend (`M-STATIC-VERIFICATION`: clippy's `pedantic` group
  and about twenty single lints).
- The Microsoft guidelines for things this repository does not have
  (async services, proc macros, DLL boundaries, telemetry) have nothing to
  apply to.

**Where the limits come from.** None of the numbers of sections 4 and 5 is
taken from a reference standard; neither the Google guide nor the Microsoft
guidelines limit parameters or complexity.

| Limit | Origin |
| --- | --- |
| 7 parameters (4.4) | Clippy's default for `too_many_arguments`, used for both languages; ruff's default is 5 |
| 50 statements (4.5) | ruff's default, taken over from pylint |
| complexity 10 (4.5) | ruff's default; McCabe's original threshold |
| 120 lines per Rust function (5.5) | this repository's choice; Clippy's default is 100 |
| about 15 fields per record (4.4) | this repository's choice |

**Read and not adopted.** The [Scientific Python Development
Guide](https://learn.scientific-python.org/development/) is a guide to
setting up a package; the tools it recommends (ruff, mypy, pytest) are the
ones in use here. The [ANSSI Secure Rust
Guidelines](https://anssi-fr.github.io/rust-guide/) are written for
security-critical applications and are stricter than this router needs
(checked arithmetic everywhere, no indexing). Microsoft's [Engineering
Playbook](https://microsoft.github.io/code-with-engineering-playbook/code-reviews/recipes/python/)
has a Python review recipe that points to PEP 8 with type hints and adds
nothing to the rows above.

**Checks between the stages (the logic harness).** No reference standard
says how a router checks what one stage hands to the next. The harness of
`python/photonic_router/logic_checks/` (rules 6.5 to 6.9,
`docs/ARCHITECTURE.md` section 8.1) follows practices that others
document; they were read on 2026-11-21, and the table says what is taken
from each.

| Practice | Source | What is taken here |
| --- | --- | --- |
| A verifier for the intermediate product, run after a transformation | LLVM, [Module Verifier](https://llvm.org/docs/Passes.html) ("useful to run after an optimization which is undergoing testing"; front ends are encouraged to verify their output before optimizing; it "just tries to ensure that code is well-formed") | one module of statements per stage; `--verify-stages` runs them inside a normal run; they say a product is well-formed, not that the routing is good |
| Checks that cost time are a switch, off by default | LLVM, [CMake options](https://llvm.org/docs/CMake.html) (`LLVM_ENABLE_ASSERTIONS`, on only in debug builds; `LLVM_ENABLE_EXPENSIVE_CHECKS`, "Defaults to OFF"); GCC, [`--enable-checking`](https://gcc.gnu.org/install/configure.html) (internal consistency checks that do not change the generated code and slow the compiler down) | the verifier is off by default and changes no routed result; the wide designs are routed by the harness only on request |
| State the precondition as a check with a message | LLVM, [Coding Standards](https://llvm.org/docs/CodingStandards.html), "Assert Liberally" ("Check all of your preconditions and assumptions"; "put some kind of error message in the assertion statement") | every finding names its check, its subject and the values that disagree |
| An invariant is different from something that is only expected | SQLite, [The Use Of assert()](https://www.sqlite.org/assert.html) (`assert(X)`: "X is an invariant"; `ALWAYS(X)`: true "as far as the developers know"; `testcase(X)`: "definitely not an invariant") | two severities: an *error* is a product that contradicts itself, a *note* is legal and has to be handled by a later stage |
| A failed check has a defined consequence; a check has no side effect | Holzmann, "The Power of 10: Rules for Developing Safety-Critical Code", IEEE Computer 39 (6), 2006, rule 5, read as quoted in the [Wikipedia article](https://en.wikipedia.org/wiki/The_Power_of_10:_Rules_for_Developing_Safety-Critical_Code) (the paper itself could not be retrieved) | a check is a pure function of the product; an error ends a verified run with `StageVerificationError` |
| Expectations on the data between pipeline stages, reviewed by a person and kept under version control | TensorFlow, [Data Validation guide](https://www.tensorflow.org/tfx/guide/tfdv) (a schema "codifies properties which the input data is expected to satisfy"; validation is "invoked multiple times" in a pipeline) | the findings per benchmark are a pinned file; a pin that moves is re-pinned with the reason |
| Show that a check can fail | SQLite, [How SQLite Is Tested](https://www.sqlite.org/testing.html), mutation testing ("verifies that the test suite catches the mutation") | every statement has a test that breaks one thing in a real product and expects exactly that finding |

Not taken over: assertion density as a number (Power of 10 asks for two
per function; here the statements sit at the stage boundaries, not in
every function), SQLite's rule that an `assert` is compiled out of a
release (a Python check that is not called costs nothing; the switch does
that job), and SQLite's 100 % branch coverage. Named by the owner's
question and not read: Meyer's design by contract and Microsoft's Code
Contracts. Fetched and not readable: Breck et al., "Data Validation for
Machine Learning" (MLSys 2019); the TensorFlow guide above stands in for
it.

## 2. How a rule is enforced

Every rule below carries one of three levels. A rule without a level is not
a rule.

- **Tool**: a CI job fails (`.github/workflows/ci.yml`). For a proposed Tool
  rule the job holds the count of violations (section 8) until it is zero.
- **Test**: a test in `tests/` fails.
- **Review**: a reviewer checks it. A pull request template that lists these
  rules is proposed and does not exist yet.

**One entry.** `scripts/check.sh` runs every Tool check locally: without a
flag the fast stage (`scripts/lint.sh` and the tests of the harness, about
ten seconds), with `--full` also the kernel build, `scripts/test_baseline.sh`
and the two gates (about fifteen minutes). `scripts/hooks/pre-commit` runs
the fast stage before every commit once `git config core.hooksPath
scripts/hooks` is set. `tests/test_coding_standards.py` holds this document
to its word: a rule marked *Tool, in force* must quote a check the harness
runs (a selected ruff rule, a clippy lint of the workspace, a measure of the
ratchet or a command of the two scripts), a rule marked *Tool, proposed*
must have its row in section 8, and that table lists exactly the pinned
measures.

**Exceptions.** The limits are not loosened; a function that has a reason
to exceed one carries an exception at the function: in Python a
`# noqa: <rule>` with a comment that names the reason, in Rust
`#[expect(clippy::<lint>, reason = "...")]` (Microsoft's
`M-LINT-OVERRIDE-EXPECT`: unlike `#[allow]`, the compiler reports an
`#[expect]` that is no longer needed). Per-file ignores in `pyproject.toml`
cover whole categories such as the tests. An exception without a reason is
a finding. Reasons that hold:

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
   `router-core` has no Python dependency (`cargo test -p router-core`
   needs no Python). *Tool, in force: the ratchet holds the imports from a
   lower layer into a higher one at zero (`upward_imports`).* The
   rule does not order the stage packages among themselves, and several of
   them import each other.
5. **Names starting with an underscore stay in their module.** Another
   module imports only public names. *Test, proposed.*
6. **Configuration has one path.** Nothing below `config_loading.py` reads
   the environment and nothing below `cli.py` sees a command line. A field
   exists in both configuration mirrors (`config.py`,
   `crates/router-core/src/config.rs`). *Tool for the generated reference
   (`scripts/generate_config_reference.py --check`), review for the rest;
   in force.*
7. **The three paper configurations do not leak into each other.** The
   baseline never receives a crossing plan. *Test, in force
   (`tests/test_placement_analysis.py`).*

## 4. Python

1. Formatting and the lint set of `pyproject.toml` (`ruff format`,
   `ruff check`). *Tool, in force.*
2. Every function of the package has annotated parameters and an annotated
   return type (ruff `ANN001`, `ANN201`, `ANN202`). *Tool, in force.* `mypy`
   passes on the package (`python -m mypy`). *Tool, in force.*
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
   it is for and what it assumes (ruff `D100` to `D104`). *Tool, in force.*
8. Comments explain why, not what. A comment that restates the code is
   removed. *Review, in force.*
9. An error message names what failed with the values that identify it (net
   name, coordinates, the benchmark). `except Exception` needs a comment
   saying why nothing narrower works (ruff `BLE001`). *Tool, proposed.*
10. Imports are absolute: `from photonic_router.records import x`, not
    `from .records import x` (Google guide 2.2; ruff `TID252`). *Tool, in
    force.*
11. A function reads every parameter it takes (ruff `ARG001`). A parameter
    that a callback signature dictates and the function does not need
    starts with an underscore. *Tool, in force.*
12. A function or class of the package is used by the package, a script or
    a benchmark. A name that only a test mentions is dead code with a test
    (`names_only_tests_use`). *Tool, proposed.*

## 5. Rust

1. `cargo fmt` and `cargo clippy --workspace --all-targets -- -D warnings`.
   *Tool, in force.*
2. The two workspace-level allows (`too_many_arguments`, `type_complexity`)
   are debt, not policy: new code does not rely on them. *Review, in force;
   removing the allows is proposed.*
3. Library code in `router-core` does not call `unwrap()`. An invariant is
   stated with `expect("why this cannot fail")`; a failure the caller can
   act on is a `Result`. *Tool (`clippy::unwrap_used` in the workspace
   lints; `clippy.toml` exempts the tests), in force.*
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
5. Every stage a run reports has its statements in one module of
   `logic_checks/`, listed in the module docstring as errors and notes.
   An error is a product that contradicts itself; a note is legal and has
   to be handled by a later stage. *Review, in force for the stages of
   `docs/ARCHITECTURE.md` section 8.1.*
6. A statement is checked on the product the stage really hands on, by a
   pure function that recomputes what it states by its own means and
   changes nothing. *Review, in force.*
7. Every statement has a test in `tests/logic/` that breaks one thing in a
   real product and expects exactly that finding. *Review, in force.*
8. The findings per public benchmark and router configuration (baseline,
   contribution 1, contribution 2) are pinned in
   `tests/baselines/logic_findings.json`; an error that stays in the code
   is named there per benchmark and configuration with what it is
   (`KNOWN_ERRORS`).
   *Test, in force (`tests/logic/test_logic_findings.py`).*
9. A stage that is added reports its product (`routing/stage_observer.py`)
   and gets its checks in `logic_checks/stages.py`, so that
   `--verify-stages` and `scripts/logic_check.py` read it on one path.
   *Review, in force.*

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
   "Generated files"; `scripts/generate_config_reference.py --check`,
   `scripts/generate_rust_stubs.py --check`). *Tool, in force.*
5. Every behaviour change has a `CHANGELOG.md` entry. *Review, in force.*

## 8. Where the code stands

Production Python under `python/photonic_router/` (46,110 lines, 109 files)
and `crates/router-core`. `scripts/quality_ratchet.py` measures the counts
below and `tests/baselines/quality_ratchet.json` pins them. The table shows
the first measurement and the counts after the first two work packages (both
2026-11-19); the file is the current truth. Rules 4.2 (annotations, and mypy with 348
errors at the first measurement), 4.7 (docstrings, ruff `D100` to `D104`;
56 classes, 136 methods and 77 functions were without one at the first
measurement), 4.10 (absolute imports) and 5.3 (no `unwrap()`) left the
table when their counts reached zero and the lint sets took them over.

| Rule | Measure (name in the pin) | at the first measurement | after S1 and S2 slice 3 |
| --- | --- | --- | --- |
| 3.2 | fields of the shared session state (`session_state_fields`) | 82 | 14 |
| 3.4 | imports from a lower layer into a higher one (`upward_imports`) | 3 | 0 |
| 3.5 | underscore names imported from another module (`private_imports`) | 247 | 27 |
| 4.3 | `dict[str, object]` / `dict[str, Any]` annotations (`untyped_dict_annotations`) | 222 | 216 |
| 4.4 | functions over 7 parameters (`ruff_PLR0913`) | 58; the largest 50 | 50 |
| 4.12 | functions and classes only tests name (`names_only_tests_use`) | 16 (2026-11-23) | 16 |
| 4.5 | functions over 50 statements (`ruff_PLR0915`) | 43 | 40 |
| 4.5 | functions over complexity 10 (`ruff_C901`) | 70 | 68 |
| 4.6 | print calls in the package (`ruff_T201`) | 157 | 157 |
| 4.9 | blind `except Exception` (`ruff_BLE001`) | 35 | 35 |
| 5.2 | workspace-level clippy allows (`rust_workspace_debt_allows`) | 2 | 2 |
| 5.4 | public items of `router-core` without a doc comment (`rust_missing_docs`) | 796 | 796 |
| 5.5 | functions of `router-core` over 120 lines (`rust_too_many_lines`) | 23 | 23 |
| 2 | exceptions written at a function (`python_exceptions`, `rust_exceptions`) | 0, 39 | 0, 39 |
| 7.3 | user guide, developer guide (not measured) | none | none |

The Rust exceptions were 39 per-function `allow(clippy::too_many_arguments)`
attributes from before this document. They are `#[expect]` with a reason
now (34; the compiler reported five as no longer needed), and the workspace
denies `clippy::allow_attributes` and
`clippy::allow_attributes_without_reason`, so an `#[allow]` or an exception
without a reason does not compile under `scripts/lint.sh`. The 34 disappear
when the functions are reworked.

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
