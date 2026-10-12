#!/usr/bin/env bash
# Create (or update) the conda envs. Safe to re-run.
set -euo pipefail
cd "$(dirname "$0")/.."
eval "$(conda shell.bash hook)"
if conda env list | grep -qE '^openh\s'; then
  conda env update -n openh -f environment.yml
else
  conda env create -f environment.yml
fi
if [ "${1:-}" = "--with-export" ]; then
  if conda env list | grep -qE '^openh-export\s'; then
    conda env update -n openh-export -f environment_export.yml
  else
    conda env create -f environment_export.yml
  fi
fi
echo "done. Next: scripts/build.sh"
