#!/usr/bin/env python
"""Generate `docs/CONFIGURATION.md` from the configuration dataclasses.

Milestone 7 of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`:
the configuration reference is generated, not written by hand, so it cannot
drift from the code. Everything in the document comes from four places in the
repository and nothing is typed twice:

* `python/photonic_router/config.py` -- the `RoutingConfig` tree (the router's
  own settings, Milestone 1): field name, type annotation, default, and the
  `#:` comment or trailing docstring next to the field.
* `python/photonic_router/flow_options.py` -- the `FlowOptions` tree (the
  flow-level choices, Milestone 5 Slice 3), same four things per field.
* `python/photonic_router/env_overlay.py` -- `ENV_OVERLAY`, the one table
  mapping a `PHOTONIC_ROUTER_*` name to one field path of `RoutingConfig`,
  inverted here to print the overlay name next to its field.
* `python/photonic_router/cli.py` -- `_flow_options` (which `FlowOptions`
  field each parsed argument fills) and `_build_arg_parser` (each argument's
  option strings), read as source with `ast` so the flag column is the
  command line's own truth.

Usage, from the repository root:

    .venv/bin/python scripts/generate_config_reference.py            # write
    .venv/bin/python scripts/generate_config_reference.py --check    # verify

`--check` exits 1 and prints a unified diff when the checked-in document
differs from the generated one; `tests/test_config_reference.py` asserts the
same thing. The output is a pure function of the sources above (declaration
order everywhere, no timestamps), so two runs give the same bytes.
"""

from __future__ import annotations

import argparse
import ast
import dataclasses
import difflib
import inspect
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "python") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "python"))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from photonic_router import cli as cli_module  # noqa: E402
from photonic_router import config as config_module  # noqa: E402
from photonic_router import env_overlay as env_overlay_module  # noqa: E402
from photonic_router import flow_options as flow_options_module  # noqa: E402

OUTPUT_PATH = REPO_ROOT / "docs" / "CONFIGURATION.md"

#: The two `RoutingConfig` paths `env_overlay.py` applies from their own
#: function instead of from a `ENV_OVERLAY` entry (precedence, unit
#: conversion, validation that raises): `_apply_astar_timeout` and
#: `_apply_long_straight_congestion_weight`. The generator asserts that the
#: names below are exactly `env_overlay.SPECIALLY_HANDLED_NAMES`, so adding
#: or renaming one there fails here instead of silently vanishing from the
#: document.
SPECIAL_ENV_PATHS: dict[tuple[str, ...], tuple[str, ...]] = {
    ("router", "search", "astar_timeout_ms"): (
        "PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS",
        "PHOTONIC_ROUTER_ASTAR_TIMEOUT_S",
    ),
    ("router", "search", "long_straight_congestion_weight"): (
        "PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT",
    ),
}


# --- field documentation out of the source ------------------------------


def _field_docs(module: Any) -> dict[tuple[str, str], str]:
    """`(class name, field name) -> description` for every annotated field of
    every class in `module`.

    Two styles are read, because both are in use: a block of `#`/`#:` comment
    lines immediately above the field (`config.py`), and a bare string
    expression immediately below it (`state.py`'s style, supported so a group
    that switches keeps its documentation).
    """
    source = Path(inspect.getsourcefile(module)).read_text(encoding="utf-8")
    lines = source.splitlines()
    tree = ast.parse(source)
    docs: dict[tuple[str, str], str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        body = node.body
        for index, statement in enumerate(body):
            if not isinstance(statement, ast.AnnAssign):
                continue
            if not isinstance(statement.target, ast.Name):
                continue
            name = statement.target.id
            text = ""
            following = body[index + 1] if index + 1 < len(body) else None
            if (
                isinstance(following, ast.Expr)
                and isinstance(following.value, ast.Constant)
                and isinstance(following.value.value, str)
            ):
                text = " ".join(following.value.value.split())
            if not text:
                comment: list[str] = []
                line_number = statement.lineno - 1  # 0-based index of the field line
                while line_number > 0:
                    candidate = lines[line_number - 1].strip()
                    if not candidate.startswith("#"):
                        break
                    comment.append(candidate.lstrip("#").lstrip(":").strip())
                    line_number -= 1
                text = " ".join(" ".join(reversed(comment)).split())
            if text:
                docs[(node.name, name)] = text
    return docs


# --- the environment overlay names, inverted ----------------------------


def _env_names_by_path() -> dict[tuple[str, ...], tuple[str, ...]]:
    """`RoutingConfig` field path -> the `PHOTONIC_ROUTER_*` names that set it."""
    names: dict[tuple[str, ...], list[str]] = {}
    for entry in env_overlay_module.ENV_OVERLAY:
        names.setdefault(tuple(entry.path), []).append(entry.name)
    special = {name for group in SPECIAL_ENV_PATHS.values() for name in group}
    if special != set(env_overlay_module.SPECIALLY_HANDLED_NAMES):
        raise SystemExit(
            "env_overlay.SPECIALLY_HANDLED_NAMES changed: "
            f"{sorted(env_overlay_module.SPECIALLY_HANDLED_NAMES)} != {sorted(special)}; "
            "update SPECIAL_ENV_PATHS in scripts/generate_config_reference.py"
        )
    for path, group in SPECIAL_ENV_PATHS.items():
        names.setdefault(path, []).extend(group)
    return {path: tuple(value) for path, value in names.items()}


# --- the command-line flags, out of cli.py ------------------------------


def _option_strings_by_dest() -> dict[str, str]:
    """Each parser argument's flags, as the reference prints them; a positional
    argument (the benchmark name) prints as its placeholder."""
    parser = cli_module._build_arg_parser()
    rendered: dict[str, str] = {}
    for action in parser._actions:
        if action.dest == "help":
            continue
        if action.option_strings:
            rendered[action.dest] = " / ".join(f"`{name}`" for name in action.option_strings)
        else:
            rendered[action.dest] = f"`<{action.dest}>` (positional)"
    return rendered


def _args_attributes(node: ast.AST) -> list[str]:
    """Every `args.<dest>` read inside `node`, in source order, deduplicated."""
    found: list[str] = []
    for child in ast.walk(node):
        if (
            isinstance(child, ast.Attribute)
            and isinstance(child.value, ast.Name)
            and child.value.id == "args"
            and child.attr not in found
        ):
            found.append(child.attr)
    return found


def _cli_flags_by_path() -> dict[tuple[str, ...], str]:
    """`FlowOptions` field path -> the command-line flag(s) that fill it.

    `cli.py`'s `_flow_options` is one `FlowOptions(...)` literal: each keyword
    is a group, each keyword inside a group is a field. A field whose value is
    exactly `args.<dest>` gets that argument's flag; a conditional takes the
    flag of the argument its test reads (`--crossings`, whose absence means
    "the script default"); a nested constructor lists the flags of every
    argument it passes, in order.
    """
    option_strings = _option_strings_by_dest()
    source = Path(inspect.getsourcefile(cli_module)).read_text(encoding="utf-8")
    tree = ast.parse(source)
    flow_options_fn = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_flow_options"
    )
    call = next(
        node
        for node in ast.walk(flow_options_fn)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "FlowOptions"
    )
    flags: dict[tuple[str, ...], str] = {}
    for group_keyword in call.keywords:
        if group_keyword.arg is None or not isinstance(group_keyword.value, ast.Call):
            continue
        for field_keyword in group_keyword.value.keywords:
            if field_keyword.arg is None:
                continue
            value = field_keyword.value
            if isinstance(value, ast.IfExp):
                dests = _args_attributes(value.test)
            else:
                dests = _args_attributes(value)
            rendered = [option_strings[dest] for dest in dests if dest in option_strings]
            if rendered:
                flags[(group_keyword.arg, field_keyword.arg)] = ", ".join(rendered)
    return flags


# --- walking a configuration tree ---------------------------------------


@dataclasses.dataclass(frozen=True)
class Group:
    """One dataclass of a configuration tree, as the document prints it."""

    path: tuple[str, ...]
    cls: type
    doc: str


@dataclasses.dataclass(frozen=True)
class Leaf:
    """One configuration field of a group."""

    path: tuple[str, ...]
    name: str
    type_name: str
    default: str
    description: str


def _group_member(field: dataclasses.Field) -> type | None:
    factory = field.default_factory  # type: ignore[misc]
    if factory is not dataclasses.MISSING and dataclasses.is_dataclass(factory):
        return factory  # the group classes are their own default factories
    return None


def _render_default(field: dataclasses.Field) -> str:
    if field.default is not dataclasses.MISSING:
        return f"`{field.default!r}`"
    factory = field.default_factory  # type: ignore[misc]
    if factory is not dataclasses.MISSING:
        return f"`{factory()!r}`"
    return "(required)"


def _class_doc(cls: type) -> str:
    return " ".join((cls.__doc__ or "").split())


def _walk(cls: type, path: tuple[str, ...], docs: dict[tuple[str, str], str]) -> list[Any]:
    """The group at `path` followed by its leaves, then its sub-groups, depth
    first in declaration order."""
    items: list[Any] = [Group(path=path, cls=cls, doc=_class_doc(cls))]
    subgroups: list[tuple[str, type]] = []
    for field in dataclasses.fields(cls):
        member = _group_member(field)
        if member is not None:
            subgroups.append((field.name, member))
            continue
        items.append(
            Leaf(
                path=(*path, field.name),
                name=field.name,
                type_name=str(field.type),
                default=_render_default(field),
                description=docs.get((cls.__name__, field.name), ""),
            )
        )
    for name, member in subgroups:
        items.extend(_walk(member, (*path, name), docs))
    return items


# --- rendering -----------------------------------------------------------


def _cell(text: str) -> str:
    """One table cell: an empty value reads `--`, and a `|` is escaped so that a
    union type annotation (`int | None`) cannot split the row."""
    return text.replace("|", "\\|") if text else "--"


def _table(leaves: list[Leaf], extra_header: str, extra: dict[tuple[str, ...], str]) -> list[str]:
    lines = [
        f"| field | type | default | {extra_header} | description |",
        "| --- | --- | --- | --- | --- |",
    ]
    for leaf in leaves:
        lines.append(
            "| `{name}` | `{type_name}` | {default} | {extra} | {description} |".format(
                name=leaf.name,
                type_name=_cell(leaf.type_name),
                default=_cell(leaf.default),
                extra=_cell(extra.get(leaf.path, "")),
                description=_cell(leaf.description),
            )
        )
    return lines


def _section(
    items: list[Any],
    extra_header: str,
    extra: dict[tuple[str, ...], str],
    root_label: str,
) -> list[str]:
    lines: list[str] = []
    pending: list[Leaf] = []
    open_group = False

    def close_group() -> None:
        nonlocal pending, open_group
        if not open_group:
            return
        if pending:
            lines.extend(_table(pending, extra_header, extra))
        else:
            lines.append("No fields of its own.")
        lines.append("")
        pending = []
        open_group = False

    for item in items:
        if isinstance(item, Leaf):
            pending.append(item)
            continue
        close_group()
        label = ".".join(item.path) if item.path else root_label
        lines.append(f"### `{label}` -- `{item.cls.__name__}`")
        lines.append("")
        if item.doc:
            lines.append(item.doc)
            lines.append("")
        open_group = True
    close_group()
    return lines


def render() -> str:
    config_docs = _field_docs(config_module)
    flow_docs = _field_docs(flow_options_module)
    env_names = _env_names_by_path()
    cli_flags = _cli_flags_by_path()

    routing_items = _walk(config_module.RoutingConfig, (), config_docs)
    flow_items = _walk(flow_options_module.FlowOptions, (), flow_docs)
    routing_leaves = [item for item in routing_items if isinstance(item, Leaf)]
    flow_leaves = [item for item in flow_items if isinstance(item, Leaf)]

    env_column = {
        path: ", ".join(f"`{name}`" for name in names) for path, names in env_names.items()
    }
    # The command-line map is keyed by (group, field), which is exactly how a
    # `FlowOptions` leaf's path reads: the tree is one group deep.
    flag_column = cli_flags

    with_env = sum(1 for leaf in routing_leaves if leaf.path in env_column)
    with_flag = sum(1 for leaf in flow_leaves if leaf.path in flag_column)

    lines: list[str] = [
        "# Configuration reference",
        "",
        "<!-- GENERATED FILE. Do not edit by hand. Regenerate with",
        "     `.venv/bin/python scripts/generate_config_reference.py`",
        "     and verify with `--check` (also asserted by",
        "     `tests/test_config_reference.py`). -->",
        "",
        "Every knob the router takes, generated from the dataclasses themselves by",
        "`scripts/generate_config_reference.py`. Two trees configure a run and nothing",
        "else does:",
        "",
        "* `RoutingConfig` (`python/photonic_router/config.py`) -- the router's own",
        "  settings, one typed field per former `PHOTONIC_ROUTER_*` environment",
        "  variable (Milestone 1). Its `router` subtree mirrors Rust `RouterConfig`",
        "  (`src/config.rs`) field for field and default for default, and is what",
        "  `RouterConfig.to_rust` hands to `rust_backend.PyPhotonicRouter`.",
        "* `FlowOptions` (`python/photonic_router/flow_options.py`) -- the flow-level",
        "  choices: which benchmark, which stages run, which artifacts are written.",
        "  It is the second argument of `photonic_router.flow.route_benchmark`.",
        "",
        "Precedence, lowest to highest (`python/photonic_router/config_loading.py`,",
        "`python/photonic_router/cli.py::main`): the dataclass defaults below, then the",
        "benchmark module's `STABLE_ROUTING_ENV` block, then the process environment,",
        "then the command line. The environment column names the overlay variable that",
        "reaches a field (`python/photonic_router/env_overlay.py`'s `ENV_OVERLAY`); the",
        "command-line column names the flag `photonic_router.cli::_flow_options` fills a",
        "field from. A field with no variable and no flag is set programmatically only.",
        "A field of the `router` subtree with an empty description is documented by its",
        "Rust twin's doc comment in `src/config.rs`, which these dataclasses mirror.",
        "See `docs/ARCHITECTURE.md` for how the two trees reach the stages.",
        "",
        (
            f"Totals: {len(routing_leaves)} `RoutingConfig` fields ({with_env} with an "
            f"overlay variable), {len(flow_leaves)} `FlowOptions` fields ({with_flag} "
            f"with a command-line flag), {len(routing_leaves) + len(flow_leaves)} in total."
        ),
        "",
        "## `RoutingConfig` -- the router's settings",
        "",
    ]
    lines.extend(_section(routing_items, "environment variable", env_column, "RoutingConfig"))
    lines.append("## `FlowOptions` -- the flow-level choices")
    lines.append("")
    lines.extend(_section(flow_items, "command line", flag_column, "FlowOptions"))
    while lines and lines[-1] == "":
        lines.pop()
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit non-zero (with a diff) when the checked-in file is not the generated one",
    )
    args = parser.parse_args(argv)
    generated = render()
    if args.check:
        current = OUTPUT_PATH.read_text(encoding="utf-8") if OUTPUT_PATH.exists() else ""
        if current == generated:
            print(f"{OUTPUT_PATH.relative_to(REPO_ROOT)}: up to date")
            return 0
        sys.stdout.writelines(
            difflib.unified_diff(
                current.splitlines(keepends=True),
                generated.splitlines(keepends=True),
                fromfile=f"{OUTPUT_PATH.relative_to(REPO_ROOT)} (checked in)",
                tofile="generated",
            )
        )
        print(
            f"{OUTPUT_PATH.relative_to(REPO_ROOT)} is out of date; regenerate it with "
            "`.venv/bin/python scripts/generate_config_reference.py`"
        )
        return 1
    OUTPUT_PATH.write_text(generated, encoding="utf-8")
    print(f"wrote {OUTPUT_PATH.relative_to(REPO_ROOT)} ({len(generated.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
