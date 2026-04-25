#!/usr/bin/env bash
set -eo pipefail

apt update
apt install -y python3-pip python3-jinja2 python3-yaml python3-jsonschema
python3 -m pip install --upgrade pip
python3 -m pip install kconfiglib pyyaml empy packaging
