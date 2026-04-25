#!/usr/bin/env bash
set -eo pipefail

ss -lunp | grep -E '1456[1-2]|1454[0-1]' || true

