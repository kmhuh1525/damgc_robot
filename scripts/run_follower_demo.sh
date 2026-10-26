#!/usr/bin/env bash
# Demo and manual curves share the same follower curve receiver.
set -Eeuo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${demo_root}/scripts/run_follower_curve.sh" "$@"
