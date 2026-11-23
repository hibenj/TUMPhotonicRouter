"""Where the reports of a run go.

The package reports through `logging` (`docs/CODING_STANDARDS.md` rule
4.6): a module has its logger and says what it did with `_LOGGER.info`.
Nothing appears unless a handler is installed. The command line installs
this module's handler with `show_reports`, which shows the reports as
`print` showed them before: one line each on standard output, a warning or
an error on standard error, without a prefix. A program that uses the
package as a library configures `logging` as it likes instead.
"""

from __future__ import annotations

import logging
import sys

PACKAGE_LOGGER = "photonic_router"


class _ConsoleHandler(logging.Handler):
    """Writes each record's message as one line: reports to stdout, warnings to stderr.

    The stream is looked up when a record is written, so output that a
    caller redirects (`contextlib.redirect_stdout`, pytest's capture)
    follows the redirection.
    """

    def emit(self, record: logging.LogRecord) -> None:
        stream = sys.stderr if record.levelno >= logging.WARNING else sys.stdout
        try:
            stream.write(self.format(record) + "\n")
            stream.flush()
        except (OSError, ValueError):
            self.handleError(record)


def show_reports() -> None:
    """Show the package's reports on the console; calling it again changes nothing."""
    logger = logging.getLogger(PACKAGE_LOGGER)
    if not any(isinstance(handler, _ConsoleHandler) for handler in logger.handlers):
        logger.addHandler(_ConsoleHandler())
    if logger.level == logging.NOTSET or logger.level > logging.INFO:
        logger.setLevel(logging.INFO)
