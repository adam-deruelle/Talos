#!/usr/bin/env bash
# Lanceur de la CONSOLE web xArm.
# Démarre le serveur local (127.0.0.1) et ouvre le navigateur en mode application.
# Utilise l'interpréteur du venv local ; aucune dépendance globale.
set -euo pipefail
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
exec "$DIR/venv/bin/python" "$DIR/web/xarm_server.py" "$@"
