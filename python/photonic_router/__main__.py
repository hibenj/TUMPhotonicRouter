"""`python -m photonic_router <command> ...` (Milestone 5, Slice 3; D6).

One command today:

    python -m photonic_router route <benchmark> [flags]

which is `photonic_router.cli.main` - see `python -m photonic_router route
--help` for the flags. Run without a command, this prints the usage and exits
with argparse's usage-error status.
"""

import argparse
import sys

from gdsfactory.component import Component

from photonic_router import cli


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m photonic_router",
        description="Photonic router entry point.",
    )
    commands = parser.add_subparsers(dest="command", metavar="route", required=True)
    route = commands.add_parser(
        "route",
        add_help=False,
        help="Route a benchmark: python -m photonic_router route <benchmark> [flags].",
    )
    route.add_argument("flags", nargs="*", metavar="<benchmark> [flags]")
    return parser


def main(argv: list[str] | None = None) -> Component:
    """Dispatch a subcommand; every routing flag belongs to `cli.main`.

    `route` is dispatched before argparse looks at the rest of the line, so
    every flag - `--help` included - is the routing command's own.
    """
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] == "route":
        return cli.main(argv[1:])
    # No command, an unknown one, or `-h`: argparse prints the usage and exits
    # (status 0 for `-h`, 2 for a usage error).
    _build_arg_parser().parse_args(argv)
    raise SystemExit(2)


if __name__ == "__main__":
    main()
