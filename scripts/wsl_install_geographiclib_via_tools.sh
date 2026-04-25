#!/usr/bin/env bash
set -eo pipefail

apt update
apt install -y geographiclib-tools

geographiclib-get-geoids egm96-5
geographiclib-get-gravity egm96
geographiclib-get-magnetic emm2015

test -f /usr/share/GeographicLib/geoids/egm96-5.pgm
echo "GEOGRAPHICLIB_READY=1"

