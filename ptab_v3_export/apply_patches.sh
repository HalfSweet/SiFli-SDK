#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MAIN_PATCH="${SCRIPT_DIR}/patches/ptab_v3_main.patch"
SCHEMA_PATCH="${SCRIPT_DIR}/patches/ptab_v3_siliconschema.patch"

cd "${REPO_ROOT}"

if [[ ! -d ".git" ]]; then
  echo "ERROR: not a git repo root: ${REPO_ROOT}" >&2
  exit 1
fi

if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "ERROR: working tree has local changes (tracked/staged). Please stash/commit before applying patches." >&2
  exit 1
fi

if [[ ! -f "${MAIN_PATCH}" ]]; then
  echo "ERROR: missing patch: ${MAIN_PATCH}" >&2
  exit 1
fi
if [[ ! -f "${SCHEMA_PATCH}" ]]; then
  echo "ERROR: missing patch: ${SCHEMA_PATCH}" >&2
  exit 1
fi

if [[ ! -d "tools/SiliconSchema" ]]; then
  echo "ERROR: submodule folder not found: tools/SiliconSchema" >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

echo "[1/2] Applying superproject patch..."
git apply --index "${MAIN_PATCH}"

echo "[2/2] Applying SiliconSchema submodule patch..."
pushd tools/SiliconSchema >/dev/null
git apply --index "${SCHEMA_PATCH}"
popd >/dev/null

cat <<'EOF'

Patches applied.

Next steps:
  python3 -m pip install pyyaml Jinja2

  python3 tools/build/validate_ptab_v3.py customer/boards/sf32lb52-nano_a128r16/ptab.yaml
  python3 tools/build/validate_ptab_v3.py customer/boards/sf32lb52-nano_n16r16/ptab.yaml
  python3 tools/build/ptab_v3_regression.py --board sf32lb52-nano_52j --chip SF32LB52JUD6
EOF
