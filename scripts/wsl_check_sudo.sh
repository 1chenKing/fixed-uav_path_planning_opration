#!/usr/bin/env bash
set -eo pipefail

if sudo -n true 2>/dev/null; then
  echo "SUDO_NOPASS=1"
else
  echo "SUDO_NOPASS=0"
fi

