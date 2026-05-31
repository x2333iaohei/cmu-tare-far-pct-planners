#!/usr/bin/env bash
set -e

delay_seconds="$1"
shift

sleep "$delay_seconds"
exec "$@"
