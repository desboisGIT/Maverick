#!/usr/bin/env bash
# Apply a link profile to the wifi interface of THIS container (egress). Run it in both the
# bridge and the host container to shape both directions:
#   docker compose exec bridge bash /maverick/docker/netem/apply.sh bad
# Profiles: lab (no impairment), apartment (20 ms +- 10 ms, 1% loss),
#           bad (300 ms +- 200 ms, 20% loss), outage (100% loss), clear (same as lab).
set -euo pipefail
PROFILE="${1:-}"
WIFI_PREFIX="${WIFI_PREFIX:-10.231.2.}"   # wifi network prefix (compose.yaml sets it per stack)

IFACE="$(ip -o -4 addr show | awk -v p="$WIFI_PREFIX" 'index($4, p) == 1 {print $2; exit}')"
[ -n "$IFACE" ] || { echo "no interface in ${WIFI_PREFIX}0/24" >&2; exit 1; }

case "$PROFILE" in
    lab | clear) tc qdisc del dev "$IFACE" root 2>/dev/null || true ;;
    apartment) tc qdisc replace dev "$IFACE" root netem delay 20ms 10ms loss 1% ;;
    bad) tc qdisc replace dev "$IFACE" root netem delay 300ms 200ms loss 20% ;;
    outage) tc qdisc replace dev "$IFACE" root netem loss 100% ;;
    *) echo "usage: $0 lab|apartment|bad|outage|clear" >&2; exit 2 ;;
esac
echo "netem $PROFILE on $IFACE: $(tc qdisc show dev "$IFACE" | head -n1)"
