# Fleet-wide lease renewal (fleet/tasks/lease, reliable, ~1 Hz while owning).
# Status renews leases too, but V3+ sends Status only to neighbor cells, so
# robots elsewhere would let the lease expire and reclaim a task in progress.
FleetHeader header
OwnedTask[] owned
