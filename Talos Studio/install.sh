#!/usr/bin/env bash
# Installe xArm Control comme application de bureau (menu / dock).
# Aucune élévation nécessaire pour cette partie : tout va dans ~/.local.
# La règle udev (accès USB sans sudo) est proposée séparément à la fin.
set -euo pipefail

DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
DESKTOP="$APPS/xarm.desktop"

echo "→ Répertoire de l'application : $DIR"

# 1. Rendre les scripts exécutables
chmod +x "$DIR/xarm-web-launcher.sh" "$DIR/xarm-launcher.sh" \
         "$DIR/install.sh" "$DIR/uninstall.sh" 2>/dev/null || true

# 2. Générer le .desktop avec les chemins réels de CE dossier
mkdir -p "$APPS"
sed -e "s|^Exec=.*|Exec=$DIR/xArm|" \
    -e "s|^Icon=.*|Icon=$DIR/assets/xarm.svg|" \
    "$DIR/xarm.desktop" > "$DESKTOP"
chmod +x "$DESKTOP" 2>/dev/null || true

# 3. Rafraîchir le menu des applications
update-desktop-database "$APPS" 2>/dev/null || true

echo "✓ Entrée de bureau installée : $DESKTOP"
echo "  « xArm » apparaît maintenant dans le menu des applications."
echo
echo "Accès USB sans sudo (une seule fois, nécessite root) :"
echo "  sudo cp \"$DIR/99-xarm-hid.rules\" /etc/udev/rules.d/"
echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
echo "  puis débrancher/rebrancher le bras."
