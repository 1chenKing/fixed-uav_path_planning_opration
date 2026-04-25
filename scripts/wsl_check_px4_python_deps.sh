#!/usr/bin/env bash
set -eo pipefail

python3 - <<'PY'
modules = [
    ("kconfiglib", "kconfiglib"),
    ("jinja2", "jinja2"),
    ("yaml", "pyyaml"),
    ("em", "empy"),
    ("packaging", "packaging"),
    ("jsonschema", "jsonschema"),
]
for module, label in modules:
    try:
        __import__(module)
        print(f"{label}=OK")
    except Exception:
        print(f"{label}=MISSING")
PY
