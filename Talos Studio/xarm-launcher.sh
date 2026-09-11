#!/usr/bin/env bash
# Lanceur du panneau de contrôle xArm.
# Utilise l'interpréteur du venv local ; ne dépend d'aucun réglage global.
set -euo pipefail
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
exec "$DIR/venv/bin/python" "$DIR/xarm_gui.py" "$@"
