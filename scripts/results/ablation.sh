#!/bin/bash
# Phase 1 of the do-over (working/.agent/TUMPhotonicRouter/DO_OVER_ANALYSIS.md,
# section 5): route the paper's cells with one negotiation tactic switched
# off per variant, each into its own root, so that ablation_table.py can
# show what every tactic contributes. The reference variant is the engine as
# shipped. Every variant runs the same reproduce_date2027.sh selection.
#
# Usage: scripts/results/ablation.sh <variant>|all [what]
#   variant  one of the names below, or `all` (sequential)
#   what     a reproduce_date2027.sh selection (default `short`, the nine
#            smallest cells; `ladder` is the four Benes sizes in all three
#            configurations, `all` every row of ours, about 2.5 h per variant)
# Roots: results_ablation/<variant>/   Table: ablation_table.py results_ablation
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$R" || exit 1
what="${2:-short}"
declare -A SWITCH
SWITCH[reference]=""
SWITCH[no_braid]="PHOTONIC_ROUTER_DISABLE_BRAID_REPAIR=1"
SWITCH[no_widening]="PHOTONIC_ROUTER_NEGOTIATED_DISABLE_SOURCE_COLUMN_WIDENING=1"
SWITCH[no_post_ripup_guidance]="PHOTONIC_ROUTER_NEGOTIATED_DISABLE_POST_RIPUP_GUIDANCE=1"
SWITCH[no_simple_probe]="PHOTONIC_ROUTER_NEGOTIATED_DISABLE_SIMPLE_ROUTE_PROBE=1"
SWITCH[history_first]="PHOTONIC_ROUTER_NEGOTIATED_HISTORY_ON_FIRST_ATTEMPT=1"
SWITCH[no_long_straight]="PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT=0"
ORDER="reference no_braid no_widening no_post_ripup_guidance no_simple_probe history_first no_long_straight"
run_variant() {
  local v="$1"; [ -v 'SWITCH[$v]' ] || { echo "unknown variant: $v (one of: $ORDER)" >&2; return 2; }
  local root="$R/results_ablation/$v"; mkdir -p "$root"
  echo "$(date -Is) ablation $v ($what) env='${SWITCH[$v]}'" | tee -a "$root/ablation.txt"
  env ${SWITCH[$v]} RESULTS_ROOT="$root" scripts/results/reproduce_date2027.sh "$what"
}
case "${1:-}" in
  all) for v in $ORDER; do run_variant "$v" || exit $?; done ;;
  "") echo "usage: ablation.sh <variant>|all [what]; variants: $ORDER" >&2; exit 2 ;;
  *) run_variant "$1" ;;
esac
