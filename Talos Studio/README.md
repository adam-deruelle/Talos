# Talos Studio

Logiciel de contrôle du bras robotisé **Hiwonder xArm 1S** (6 servos sur bus série, liaison USB-HID).

Talos Studio correspond à la **phase 1 de la roadmap Talos — MK I, « Maîtrise du bras robotique »** :
piloter le bras de façon fiable et reproductible depuis un ordinateur, avant d'ajouter la vision,
la planification et l'autonomie. La vision d'ensemble du projet est décrite dans le
[README principal](../README.md).

---

## Répartition du travail

Ce dossier est le fruit de deux contributions distinctes.

**Adam DERUELLE — modèle cinématique et calculs.**
Mesure des segments du bras, établissement des paramètres de Denavit-Hartenberg, offsets
angulaires de chaque articulation et débattements utiles. Ce travail est consigné dans
`Kinematics.ods` et constitue le modèle physique du bras.

Le logiciel ne fait que l'implémenter : les constantes `FK_A`, `FK_D`, `FK_ALPHA`,
`FK_THETA_OFFSET` et `SEG_MM` de `xarm_gui.py` reprennent directement ces valeurs, et c'est sur
elles que reposent la cinématique directe (`fk_pose`), la cinématique inverse (`ik_solve`) et la
visualisation à l'échelle du bras. Sans ce modèle, l'interface ne saurait pas où se trouve la
pince dans l'espace.

**Harry Troule — logiciel de contrôle.**
Implémentation du protocole USB-HID, console native, console web, suivi par caméra,
empaquetage et intégration au bureau. Cette partie a été développée avec l'assistance de
**Claude (Claude Code)**, utilisé comme outil d'écriture et de relecture de code ; les choix
d'architecture, les essais sur le matériel réel et la validation restent le fait de l'auteur.

---

## Matériel

| | |
|---|---|
| Bras | Hiwonder xArm 1S, 6 degrés de liberté |
| Liaison | USB-HID, VID `0483` / PID `5750`, trames de 64 octets |
| Servos | Bus série LewanSoul, course `0–1000` ↔ `0–240°` (0,24°/unité) |
| Axes | 1 pince · 2 rotation poignet · 3 flexion poignet · 4 coude · 5 épaule · 6 base |

`xarm_fonctionnalites.html` recense les fonctions offertes par le contrôleur LSC et les servos,
d'après leurs protocoles constructeur.

## Composants

| Fichier | Rôle |
|---|---|
| `xarm_gui.py` | Console native (Tkinter) : commande par axe, vues de côté et de dessus à l'échelle, cinématique directe et inverse, poses, enregistrement/relecture de gestes, séquenceur, couple par axe, pilotage par téléphone, transports HID et Bluetooth-LE |
| `web/xarm_server.py` | Console web : serveur local sur `127.0.0.1`, un unique thread de travail possède le périphérique HID, les requêtes HTTP ne font qu'empiler des ordres |
| `web/static/` | Interface du panneau web (HTML/CSS/JS, sans dépendance externe) |
| `camera_tracker.py` | Suivi d'un marqueur ArUco depuis un flux MJPEG ; diffuse la position `P,x,y,z` en UDP vers le port 8899 écouté par la console native |
| `xArm.spec` | Recette PyInstaller du binaire autonome `xArm` |
| `install.sh` / `uninstall.sh` | Entrée de bureau (menu d'applications) |
| `99-xarm-hid.rules` | Règle udev : accès au bras sans `sudo` |
| `Kinematics.ods` | Modèle cinématique du bras (voir « Répartition du travail ») |
| `poses/` | Poses et séquences exportées |

## Installation

```bash
python3 -m venv venv
venv/bin/pip install -r requirements.txt
```

Accès USB sans privilèges (une seule fois) :

```bash
sudo cp 99-xarm-hid.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Débrancher puis rebrancher le bras pour que la règle s'applique.

## Utilisation

```bash
./xarm-launcher.sh          # console native
./xarm-web-launcher.sh      # console web (ouvre le navigateur)
```

La console web écoute sur `http://127.0.0.1:8787` ; `XARM_PORT` change le port et
`XARM_NO_BROWSER=1` empêche l'ouverture automatique du navigateur. Chaque session génère un
jeton aléatoire : toutes les requêtes du panneau doivent le présenter, et le serveur applique
lui-même le bornage de course, le client n'étant jamais considéré comme fiable.

Suivi par caméra (flux MJPEG d'un téléphone, par exemple DroidCam) :

```bash
XARM_CAMERA_URL=http://<ip-du-telephone>:4747/video venv/bin/python camera_tracker.py
```

### Raccourcis de la console native

| Touche | Effet |
|---|---|
| `1`–`6` | Sélectionner l'axe actif |
| `↑` `→` / `↓` `←` | Déplacer l'axe actif (`Maj` : pas de 1 · `Ctrl` : pas ×5) |
| `Espace` | Ouvrir / fermer la pince |
| `H` | Retour à la position neutre |
| `Échap` | Arrêt immédiat |

## Binaire et intégration au bureau

```bash
venv/bin/pyinstaller xArm.spec
cp dist/xArm ./xArm
./install.sh
```

`install.sh` crée l'entrée de menu pointant vers le binaire du dossier courant ;
`uninstall.sh` la retire. Le binaire et les répertoires `build/` et `dist/` sont des artefacts
de compilation : ils sont exclus du dépôt, comme `venv/`.

## Données locales

Poses, enregistrements, séquences et réglages sont stockés dans le dossier personnel
(`~/.xarm_poses.json`, `~/.xarm_recordings.json`, `~/.xarm_sequence.json`,
`~/.xarm_settings.json`) et n'entrent jamais dans le dépôt.
