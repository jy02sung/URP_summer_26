#!/usr/bin/env bash

delay_s="$1"
shift

sleep "$delay_s"
exec "$@"
