"""Compatibility facade for path-length matching helpers.

The implementation is split by responsibility:
- ``path_length_requirements`` builds timing requirements.
- ``path_length_candidates`` propagates delay insertion candidates upstream.
- ``path_length_diagnostics`` serializes reports and acceptance diagnostics.
"""

from __future__ import annotations

from translation.path_length_candidates import (
    build_requirement_delay_candidates,
    delay_candidate_to_dict,
    edge_key_to_dict,
    requirement_to_dict,
)
from translation.path_length_diagnostics import (
    analysis_to_info_dict,
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    matching_groups_to_info,
    node_timing_to_dict,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
)
from translation.path_length_requirements import (
    PATH_LENGTH_MATCH_TOLERANCE_UM,
    adjusted_output_arrivals_for_requirements,
    analyze_path_length_matching,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    merge_missing_length_requirements,
    minimum_four_bend_extra_length_um,
    routed_net_records_to_edge_lengths,
)

__all__ = [
    "PATH_LENGTH_MATCH_TOLERANCE_UM",
    "adjusted_output_arrivals_for_requirements",
    "analysis_to_info_dict",
    "analyze_path_length_matching",
    "build_requirement_delay_candidates",
    "compute_group_lifted_requirements",
    "compute_output_matching_requirements",
    "delay_candidate_to_dict",
    "edge_key_to_dict",
    "format_path_length_acceptance_failure",
    "matching_group_diagnostics_to_info",
    "matching_groups_to_info",
    "merge_missing_length_requirements",
    "minimum_four_bend_extra_length_um",
    "node_timing_to_dict",
    "output_matching_diagnostics_to_info",
    "path_length_acceptance_summary",
    "requirement_to_dict",
    "routed_net_records_to_edge_lengths",
]
