# Coding Standards

This document is short on purpose. General Python and Rust style is already well covered by existing, freely available references -- this file does not re-derive them. It exists only for the small number of conventions specific to this repository that a linter or formatter cannot enforce on its own.

Mechanical style (whitespace, import order, common bug patterns) is enforced by `ruff format`/`ruff check` for Python and `cargo fmt`/`cargo clippy` for Rust -- see `pyproject.toml`'s `[tool.ruff]` section and `rustfmt.toml`. See `.agent/execplans/2026-08-25-python-rust-linting-and-coding-standards.md` for how that tooling was adopted and what it currently does and does not yet cover.

## External references

Do not re-derive these; read them directly when a question isn't answered below.

- Python: [PEP 8](https://peps.python.org/pep-0008/) (style), [PEP 257](https://peps.python.org/pep-0257/) (docstrings), the [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html).
- Rust: the official [Rust Style Guide](https://doc.rust-lang.org/nightly/style-guide/) (what `rustfmt` implements), the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/), [Clippy's lint catalog](https://rust-lang.github.io/rust-clippy/).

## Comments explain why, not what

A comment is for the reason a reader could not otherwise infer: a hidden constraint, a workaround for a specific bug, an invariant that would surprise someone changing the code later. It is not for restating what well-named identifiers already say.

Bad (restates the code):

    # Loop over all nets and correct their endpoints
    for net in nets:
        correct_endpoint(net)

Good (explains a non-obvious constraint):

    # The middle section must stay untouched raw baseline geometry -- it is
    # what the crossing-legality check was run against, so correcting it
    # here would invalidate that check without re-running it.
    middle = baseline[first_cut:last_cut]

If removing a comment would not confuse a future reader, do not write it. This repository's own agent-facing instructions already state this principle for AI-assisted work; it applies equally to code written by hand.

## Group related parameters into a config object, don't grow a keyword-argument list

A function that keeps gaining independently-defaulted keyword parameters as it takes on more responsibility should group the related ones into a small `dataclass`/`struct` instead, once there are more than roughly five that travel together. This repository already does this in some places (`RipupRerouteConfig`, `ElectricalRoutingConfig`, and `StaticObstacleMapConfig` in `translation/`, all built once in `routing_flow.py`'s `main()` and passed as a single object) -- follow that existing pattern rather than adding another loose parameter to a function that already has many.

`routing_flow.py`'s `run_routing_flow` (~40 keyword parameters as of this writing) is the clearest example of what this looks like when the pattern is *not* applied consistently: roughly fifteen of its parameters are already grouped into the config objects above, and the rest are not. This document does not mandate an immediate rewrite of that function -- it names it so the pattern is understood concretely, not abstractly.

## Non-trivial multi-file work gets an ExecPlan first

Before starting a change that touches several files, changes behavior, or needs its own validation strategy, write an ExecPlan under `.agent/execplans/` following `.agent/PLANS.md`. This applies to human contributors the same way it already applies to AI-assisted work in this repository -- the discipline (a self-contained plan, a `Decision Log` recording why non-obvious choices were made, validation evidence before calling something done) is what keeps a change reviewable and keeps the reasoning behind it from being lost.

## Keep this document short

If a new entry here could instead be enforced by a `ruff`/`clippy` rule, add the rule (with a comment explaining the choice, in `pyproject.toml`/the CI configuration) instead of writing prose about it here. This file is for what tooling genuinely cannot check.
