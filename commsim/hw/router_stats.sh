#!/bin/sh
# Snapshot of one mesh router's radio + batman-adv state, for the hardware
# calibration runs (commsim/hw/README.md). Runs ON the router (OpenWrt ash),
# fed over ssh so nothing is installed there:
#
#   ssh root@10.42.0.253 'sh -s 10' < commsim/hw/router_stats.sh > r1_stats.txt
#
# $1 = seconds between the two survey reads (default 10). Channel busy % =
# delta busy time / delta active time over that window, per mesh interface.
# This is the hardware number for the model's background busy share
# (configs/base.yaml background.busy_share) when the fleet is idle.
W=${1:-10}
echo "== $(cat /proc/sys/kernel/hostname)  $(date -u +%FT%TZ)"
echo "== batctl n (neighbors: interface, last seen)"; batctl n
echo "== batctl o (originators: best next hop, TQ out of 255)"; batctl o
for d in $(ls /sys/class/net | grep '^mesh-'); do
  echo "== $d station dump (signal dBm, tx/rx bitrate, retries, failed)"
  iw dev "$d" station dump | grep -E 'Station|signal:|tx bitrate|rx bitrate|tx retries|tx failed|mesh plink'
  s0=$(iw dev "$d" survey dump | grep -A5 'in use' | grep -E 'active time|busy time' | awk '{print $4}' | tr '\n' ' ')
  sleep "$W"
  s1=$(iw dev "$d" survey dump | grep -A5 'in use' | grep -E 'active time|busy time' | awk '{print $4}' | tr '\n' ' ')
  set -- $s0 $s1
  if [ $# -eq 4 ] && [ "$3" -gt "$1" ]; then
    echo "== $d channel busy over ${W}s: $(( 100 * ($4 - $2) / ($3 - $1) ))%"
  else
    echo "== $d channel busy: survey counters not reported by this driver"
  fi
done
