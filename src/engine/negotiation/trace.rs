//! The negotiated loop's `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG` trace lines
//! (`RouterConfig::diagnostics.native_repair_diag`, carried on
//! `SearchArgs::trace`), gathered here so the loop itself prints nothing.
//! Every method prints unconditionally and every caller gates on the flag.
//! The per-attempt `native_negotiated_search` line stays in `attempt.rs`,
//! next to the attempt it describes and the tests that read it back. The
//! text of every line is unchanged since 2026-09-15.

use crate::engine::*;

use super::loop_::NegotiationCounters;

impl PyPhotonicRouter {
    /// The line that opens every net's turn.
    pub(super) fn trace_net_start(
        &self,
        position: usize,
        round: u32,
        net_id: u64,
        failed: u32,
        free: bool,
    ) {
        eprintln!(
            "{}native_negotiated_net index={} round={} net={} failed_count={} crossing_free={}",
            trace_t(self.negotiated_batch_start),
            position + 1,
            round,
            net_id,
            failed,
            u8::from(free)
        );
    }

    /// The mixed-case guidance: the probe's legal partners and the ids the
    /// source-column widening added to them.
    pub(super) fn trace_probe_widening(
        &self,
        net_id: u64,
        legal_partner_ids: &[u64],
        widened: &[u64],
    ) {
        let added_ids: Vec<u64> = widened
            .iter()
            .copied()
            .filter(|id| !legal_partner_ids.contains(id))
            .collect();
        eprintln!(
            "{}native_repair_probe_guided net={} partners={:?} widened={:?}",
            trace_t(self.negotiated_batch_start),
            net_id,
            legal_partner_ids,
            added_ids
        );
    }

    /// LiDAR's local rip-up rule: what it ripped for this net.
    pub(super) fn trace_local_ripup(&self, net_id: u64, ripped_ids: &[u64]) {
        eprintln!(
            "{}native_repair_local_ripup net={} ripped={:?}",
            trace_t(self.negotiated_batch_start),
            net_id,
            ripped_ids
        );
    }

    /// The round-end global rip-up: the nets that failed the round and the
    /// blockers it ripped for them.
    pub(super) fn trace_global_ripup(&self, round: u32, round_failed: &[u64], ripped: &[u64]) {
        eprintln!(
            "{}native_repair_global_ripup round={} failed={:?} ripped={:?}",
            trace_t(self.negotiated_batch_start),
            round,
            round_failed,
            ripped
        );
    }

    /// The global rip-up's once-per-batch epoch boundary.
    pub(super) fn trace_history_clear(&self, round: u32, global_ripups: u32) {
        eprintln!(
            "{}history_clear round={} global_ripups={}",
            trace_t(self.negotiated_batch_start),
            round,
            global_ripups
        );
    }

    /// The batch's closing line: how the loop ended and what its rules did.
    pub(super) fn trace_negotiated_done(
        &self,
        counters: &NegotiationCounters,
        round: u32,
        unrouted: usize,
    ) {
        eprintln!(
            "native_negotiated_done t={:.1} rounds={} global_ripups={} local_ripups={} unrouted={} probe_guided={} crossing_free={}",
            self.negotiated_batch_start
                .map(|start| start.elapsed().as_secs_f64())
                .unwrap_or(0.0),
            round,
            counters.global_ripups,
            counters.local_ripups,
            unrouted,
            counters.probe_guided,
            counters.crossing_free
        );
    }
}
