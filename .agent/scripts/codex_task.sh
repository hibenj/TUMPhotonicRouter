#!/usr/bin/env bash
# Hand an Implementation Engineer task to the Codex CLI, non-interactively.
#
# This script is the mechanical half of the Claude+Codex flow described in
# .agent/CLAUDE_CODEX_FLOW.md. Claude (the orchestrator/planner/reviewer in
# this flow) calls this script to delegate scoped coding work to Codex.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: .agent/scripts/codex_task.sh <slug> <task-file> [options]

  <slug>       short kebab-case label for this run, used in the run
               directory name, e.g. "add-crossing-report-field".
  <task-file>  path to a Markdown file containing the task body: scope,
               files to touch, files not to touch, and acceptance criteria.
               This script wraps it with the repository's standard subagent
               contract and the Implementation Engineer role brief
               (.agent/roles/implementer.md) so the task file itself only
               needs to state the specific work.

Options:
  --sandbox MODE   codex sandbox policy: read-only | workspace-write
                   (default) | danger-full-access
  --read-only      shorthand for --sandbox read-only; use for audit-only or
                   plan-review tasks that must not edit files
  --model MODEL    override the codex model (default: codex's own configured
                   default; do not pass this unless the task has a specific
                   reason, per .agent/ORCHESTRATOR.md's model policy)
  -h, --help       show this help

Output:
  Writes .agent/codex_runs/<timestamp>-<slug>/{prompt.md,transcript.jsonl,last-message.txt,exit_code.txt}
  Prints the run directory and the final Codex message to stdout when the
  run finishes. A nonzero exit code means the codex run itself failed
  (crash, sandbox rejection, etc.), not that Codex declined the task -- read
  last-message.txt either way before deciding what to do next.
EOF
}

if [[ $# -lt 2 || "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit "$([[ $# -lt 2 ]] && echo 1 || echo 0)"
fi

slug="$1"; task_file="$2"; shift 2

sandbox="workspace-write"
model_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sandbox) sandbox="$2"; shift 2 ;;
    --read-only) sandbox="read-only"; shift ;;
    --model) model_args=(--model "$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ ! -f "$task_file" ]]; then
  echo "Task file not found: $task_file" >&2
  exit 1
fi

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="$repo_root/.agent/codex_runs/${timestamp}-${slug}"
mkdir -p "$run_dir"

prompt_file="$run_dir/prompt.md"
{
  echo "You are working in the TUMPhotonicRouter repository root ($repo_root)."
  echo "Do not assume a fixed absolute path; this checkout may be on Windows or Linux."
  echo "Read AGENTS.md, .agent/PROJECT_GOAL.md, .agent/WORKFLOW.md, and the active"
  echo "ExecPlan under .agent/execplans/ before acting."
  echo "Do not revert user or other-agent changes."
  echo "Keep edits within the assigned file scope stated in the Task section below."
  echo "Update the active ExecPlan if your task discovers durable facts or decisions."
  echo "Report changed files, commands run, pass/fail results, and residual risks"
  echo "in your final message, in the 'Expected output' shape from the role brief"
  echo "below."
  echo
  echo "## Role brief: Implementation Engineer"
  echo
  cat "$repo_root/.agent/roles/implementer.md"
  echo
  echo "## Task"
  echo
  cat "$task_file"
} > "$prompt_file"

echo "Running codex exec (sandbox=$sandbox) for task '$slug'..." >&2
echo "Prompt written to: $prompt_file" >&2

set +e
codex exec \
  --sandbox "$sandbox" \
  -c approval_policy=never \
  -C "$repo_root" \
  "${model_args[@]}" \
  --json \
  --output-last-message "$run_dir/last-message.txt" \
  < "$prompt_file" \
  | tee "$run_dir/transcript.jsonl" >&2
status=${PIPESTATUS[0]}
set -e

echo "$status" > "$run_dir/exit_code.txt"

echo "---"
echo "Codex exit code: $status"
echo "Run directory: $run_dir"
if [[ -f "$run_dir/last-message.txt" ]]; then
  echo "Last message:"
  cat "$run_dir/last-message.txt"
fi

exit "$status"
