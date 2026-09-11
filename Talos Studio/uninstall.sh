#!/usr/bin/env bash
# Retire l'entrée de bureau xArm Control (ne touche ni au code ni au venv).
set -euo pipefail
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
rm -f "$APPS/xarm.desktop"
update-desktop-database "$APPS" 2>/dev/null || true
echo "✓ Entrée de bureau retirée."
echo "  Règle udev (si installée) : sudo rm /etc/udev/rules.d/99-xarm-hid.rules"
