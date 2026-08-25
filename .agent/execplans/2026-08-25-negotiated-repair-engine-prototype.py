"""Milestone 2 bounded prototype for
.agent/execplans/2026-08-25-negotiated-repair-engine.md.

Not wired into the real router. Nets, cells, and conflicts are all
synthetic abstractions of the real problem (a net "uses" a set of
resource ids instead of real grid cells; a "conflict" is two nets
wanting the same resource without a legal crossing). The point of this
prototype is to de-risk the ALGORITHM SHAPE (topological-order priority
queue, mark-both-nets-on-conflict, rip-up-and-requeue with accumulating
history cost, periodic full reset, distance/slack-based conflict
resolution) before committing to a real Rust implementation against the
actual A* kernel and obstacle map -- not to validate any geometry.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field


@dataclass
class Net:
    name: str
    topo_depth: int
    # A route is a *sequence* of candidate resource-sets, from cheapest
    # (index 0, no detour) to most expensive (higher index = more slack
    # used, standing in for "took a longer path to avoid a conflict").
    # Each candidate's cost stands in for path length.
    candidates: list[tuple[frozenset[str], float]]
    failed_count: int = 0
    chosen_candidate_index: int = 0
    routed: bool = False
    # Resources this net has already lost a negotiation over this "epoch"
    # (since the last history reset) -- treated as a hard obstacle for
    # this net specifically, the same way a real A* search would treat
    # another net's already-committed route as a hard obstacle rather
    # than a soft cost. Cleared on history reset, matching the idea that
    # a reset is a genuine "start over" event, not just a cost decay.
    blocked_resources: set[str] = field(default_factory=set)

    @property
    def slack(self) -> float:
        # How much extra cost this net could absorb by using a worse
        # candidate before running out of options -- LiDAR's Euclidean
        # "distance slack" stands in for this.
        if len(self.candidates) <= 1:
            return 0.0
        return self.candidates[-1][1] - self.candidates[0][1]


def build_scenario() -> list[Net]:
    # A deliberately unsolvable-in-one-shot scenario: nets A and B both
    # want resource "r1" at their cheapest candidate, and only one of
    # them can have it. B has a second, costlier candidate that avoids
    # r1 entirely (representing "a legal crossing/detour exists"); A
    # does not (representing "this net has no alternative -- it is B
    # that must yield"). C and D form an unrelated second conflict pair
    # over "r2", but symmetric (either could equally yield), to exercise
    # the tie-break. All four are declared in an order that does *not*
    # match topology depth, to prove ordering-by-depth actually matters.
    return [
        Net("D", topo_depth=3, candidates=[(frozenset({"r2"}), 10.0), (frozenset({"r2b"}), 14.0)]),
        Net("A", topo_depth=0, candidates=[(frozenset({"r1"}), 10.0)]),
        Net("C", topo_depth=2, candidates=[(frozenset({"r2"}), 10.0), (frozenset({"r2c"}), 14.0)]),
        Net("B", topo_depth=1, candidates=[(frozenset({"r1"}), 10.0), (frozenset({"r1b"}), 20.0)]),
    ]


HISTORY_COST_PER_USE = 5.0
RESET_AFTER_ROUNDS_WITHOUT_PROGRESS = 2
MAX_ROUNDS = 8


def run_negotiated_repair(nets: list[Net]) -> tuple[bool, int, list[str]]:
    """Returns (converged, rounds_used, log)."""
    log: list[str] = []
    history: dict[str, float] = {}
    resource_owner: dict[str, str] = {}
    rounds_without_progress = 0
    for round_idx in range(1, MAX_ROUNDS + 1):
        # Build this round's queue in topological order (primary key:
        # topo_depth; tiebreak: declaration order, i.e. stable sort),
        # exactly like LiDAR's Nets.__lt__ -- not the codebase's current
        # raw declaration order.
        queue = sorted(
            (net for net in nets if not net.routed),
            key=lambda net: net.topo_depth,
        )
        if not queue:
            return True, round_idx - 1, log
        log.append(f"--- round {round_idx}: queue = {[n.name for n in queue]} ---")
        yielded_this_round: set[str] = set()
        for net in queue:
            # Pick the cheapest FEASIBLE candidate: total cost is base +
            # history on every resource it would use (this is the A*
            # kernel's own job in the real system; here it is a plain
            # min() over the synthetic candidate list), but a candidate
            # that uses a resource this net already lost a negotiation
            # over is treated as infeasible, not merely expensive --
            # exactly like a real A* search treats another net's
            # committed route as a hard obstacle, not a soft cost, unless
            # a legal crossing exists (a separate mechanism, out of scope
            # for this abstract prototype). Fall back to allowing blocked
            # candidates only if every candidate is blocked (no choice
            # but to contest again).
            feasible = [
                (index, resources, base_cost)
                for index, (resources, base_cost) in enumerate(net.candidates)
                if not (resources & net.blocked_resources)
            ]
            if not feasible:
                feasible = [
                    (index, resources, base_cost)
                    for index, (resources, base_cost) in enumerate(net.candidates)
                ]
            best_index, best_cost = None, None
            for index, resources, base_cost in feasible:
                cost = base_cost + sum(history.get(r, 0.0) for r in resources)
                if best_cost is None or cost < best_cost:
                    best_index, best_cost = index, cost
            resources, _ = net.candidates[best_index]
            owners = {resource_owner[r] for r in resources if r in resource_owner}
            if not owners:
                resource_owner.update({r: net.name for r in resources})
                net.routed = True
                net.chosen_candidate_index = best_index
                log.append(f"  {net.name}: routed via candidate {best_index} {sorted(resources)}")
                continue
            # Conflict against whichever net(s) already own the resources
            # this candidate needs. Pairwise distance/slack heuristic,
            # mirroring LiDAR's own per-pair `clear` condition exactly:
            # does the OWNER have much more slack than ME (the contester),
            # and have I not already failed before? If so, the OWNER
            # yields to me. Otherwise I (the contester, still unrouted)
            # am the one who yields -- I simply do not get this resource
            # this round and stay in the failed set for the next one.
            # Only ever one side of a conflict is unrouted, never both --
            # this is the bug the prototype itself caught on its first
            # run (see Surprises & Discoveries): an earlier version added
            # both sides to a "failed" set and unrouted everyone in it,
            # which unrouted nets that had already legitimately won this
            # same round and produced a non-terminating oscillation.
            owner_name = next(iter(owners))
            owner = next(n for n in nets if n.name == owner_name)
            contester_should_win = (
                net.failed_count == 0 and owner.slack > net.slack
            )
            if contester_should_win:
                for resource, current_owner in list(resource_owner.items()):
                    if current_owner == owner_name:
                        del resource_owner[resource]
                owner_resources, _ = owner.candidates[owner.chosen_candidate_index]
                for resource in owner_resources:
                    history[resource] = history.get(resource, 0.0) + HISTORY_COST_PER_USE
                owner.routed = False
                owner.failed_count += 1
                owner.blocked_resources |= resources
                yielded_this_round.add(owner_name)
                resource_owner.update({r: net.name for r in resources})
                net.routed = True
                net.chosen_candidate_index = best_index
                log.append(f"  {net.name}: takes {sorted(resources)} from {owner_name} (more slack, {owner.slack} > {net.slack})")
            else:
                net.failed_count += 1
                net.blocked_resources |= resources
                yielded_this_round.add(net.name)
                log.append(f"  {net.name}: yields to {owner_name} (insufficient slack advantage)")

        if not yielded_this_round:
            rounds_without_progress = 0
            continue

        rounds_without_progress += 1
        if rounds_without_progress >= RESET_AFTER_ROUNDS_WITHOUT_PROGRESS:
            log.append(f"  (round {round_idx}) no progress for {rounds_without_progress} rounds -- resetting history map")
            history.clear()
            for net in nets:
                net.blocked_resources.clear()
            rounds_without_progress = 0

    return all(net.routed for net in nets), MAX_ROUNDS, log


if __name__ == "__main__":
    nets = build_scenario()
    converged, rounds, log = run_negotiated_repair(nets)
    print("\n".join(log))
    print()
    print(f"converged={converged} rounds={rounds}")
    for net in sorted(nets, key=lambda n: n.name):
        resources = net.candidates[net.chosen_candidate_index][0] if net.routed else None
        print(f"  {net.name}: routed={net.routed} via={sorted(resources) if resources else None} failed_count={net.failed_count}")

    # A naive "route once in declaration order, never retry" baseline,
    # for direct comparison -- this is close to what today's dispatch
    # chain effectively does for a conflict its 17 special cases don't
    # happen to cover: whichever net is declared first (or has this
    # scenario's ad hoc priority integer) wins, the other fails outright,
    # with no negotiation.
    print()
    print("naive one-shot baseline (route in raw declaration order, no retry):")
    naive_nets = build_scenario()
    naive_owner: dict[str, str] = {}
    for net in naive_nets:
        resources, _ = net.candidates[0]
        if any(r in naive_owner for r in resources):
            print(f"  {net.name}: FAILED outright (no negotiation, no fallback candidate tried)")
            continue
        naive_owner.update({r: net.name for r in resources})
        print(f"  {net.name}: routed via candidate 0 {sorted(resources)}")
