<a id="top"></a>
# Talos

**[🇬🇧 English](#english) · [🇫🇷 Français](#francais)**

<!-- TODO: add a GIF or short video of the arm moving (e.g. docs/demo.gif) -->

---

<a id="english"></a>
## 🇬🇧 English

> Open-source robotic assistant — a long-term R&D project to build, step by step, a robot that can perceive its environment, understand natural instructions and physically manipulate objects.

**Status:** Phase 1 — *Mk I, mastering the robot arm* (in progress)
**Hardware:** Hiwonder xArm 1S (6 servos, USB-HID) · Linux PC
**Stack:** Python · Tkinter · HTTP/JS web console · Denavit–Hartenberg kinematics

### Why Talos

Talos is my personal robotics lab. The goal is not to build a spectacular robot fast, but a **robust, documented and modular platform** that I can extend over the years: new sensors, a new arm, vision, language, learning.

Every capability is **understood, implemented, then validated on real hardware** before moving to the next one. The full vision is described in [`MK I/Description du projet.pdf`](MK%20I/Description%20du%20projet.pdf) (FR).

### Architecture

Independent modules running in a closed loop:

```
Observe → Understand → Decide → Act → Verify
```

| Module | Role |
|---|---|
| Human–Machine Interface | Text, voice, GUI, robot state visualization |
| Request understanding | Turns natural requests into concrete goals |
| Decision core | Analyses the situation, picks a strategy, handles errors |
| Computer vision | Object detection, localization, pose estimation |
| Knowledge base | Robot parameters, known objects, procedures, logs |
| Action planning | Breaks a goal into feasible motion steps |
| Robot control | Positions, trajectories, speeds, sequences |
| Feedback loop | Checks each action's result and corrects deviations |

Each module has a single responsibility and can be replaced without touching the others.

### Roadmap

| Phase | Name | Goal | Status |
|---|---|---|---|
| 0 | Preparation | Vision, architecture, roadmap, dev environment | ✅ Done |
| 1 | **Mk I — Arm mastery** | Reliable manual & programmed motion, poses, sequences, calibration | 🔄 In progress |
| 2 | Mk II — Arm on rail | Larger workspace, higher payload | ⏳ Planned |
| 3 | Vision | Camera, object detection & localization | ⏳ Planned |
| 4 | Autonomous manipulation | Pick & place of identified objects | ⏳ Planned |
| 5 | User interaction | GUI, text and voice commands | ⏳ Planned |
| 6 | Decision intelligence | Reasoning about assigned tasks | ⏳ Planned |
| 7–8 | Advanced robotics / R&D platform | Open experimentation | ⏳ Planned |

### Phase 1 highlights

#### Kinematic model of the xArm 1S

I measured each link of the arm, defined the joint frames and derived the **Denavit–Hartenberg parameters**, joint offsets and useful ranges. The full model (DH table, homogeneous transforms, forward kinematics) is in [`Talos Studio/Kinematics.ods`](Talos%20Studio/Kinematics.ods).

| Joint | Servo | α (°) | d (mm) | a (mm) |
|---|---|---|---|---|
| J1 — base | 6 | 90 | 32 | 0 |
| J2 — shoulder | 5 | 0 | 0 | 33 |
| J3 — elbow | 4 | 0 | 0 | 101 |
| J4 — wrist flex | 3 | −90 | 0 | 90 |
| J5 — wrist rotation | 2 | 0 | 0 | 52 |
| Tool (gripper) | 1 | — | — | 85 |

Fully extended vertical pose → wrist frame at **z = 308 mm** above the base. This model is what lets the software know where the gripper is in space.

#### Talos Studio — control software

[`Talos Studio/`](Talos%20Studio/README.md) drives the arm from a computer, built directly on the kinematic model above:

- per-axis control with live side & top views at scale
- **forward kinematics** (`fk_pose`) and numerical **inverse kinematics** (damped least squares, `ik_solve`)
- saved poses, gesture recording & replay, block-based sequencer
- native console (Tkinter) and local web console (token-protected, server-side range clamping)
- phone control via IMU over UDP (joint mode or tool-tip mode through IK)

An ArUco-marker camera-tracking mode was prototyped then dropped (reasons in the Talos Studio README).

### Repository layout

```
Talos/
├── MK I/            Project description & phase 0 documents (FR)
├── MK II/           Coming soon
└── Talos Studio/    Arm control software + kinematic model
```

### Contributors

- **Adam Deruelle** — project vision, architecture & roadmap; kinematic model of the arm (DH parameters, offsets, ranges).
- **Harry Troule** — control software (Talos Studio), developed with the help of Claude Code.

### Contact

[LinkedIn](https://www.linkedin.com/in/adam-deruelle-/) · [Portfolio](https://adam-deruelle.github.io/) · adam.deruelle@gmail.com

[⬆ Back to top](#top)

---

<a id="francais"></a>
## 🇫🇷 Français

> Assistant robotique open source — un projet de R&D au long cours pour construire, étape par étape, un robot capable de percevoir son environnement, de comprendre des instructions naturelles et de manipuler des objets.

**Statut :** Phase 1 — *Mk I, maîtrise du bras robotique* (en cours)
**Matériel :** Hiwonder xArm 1S (6 servos, USB-HID) · PC sous Linux
**Stack :** Python · Tkinter · console web HTTP/JS · cinématique de Denavit–Hartenberg

### Pourquoi Talos

Talos est mon laboratoire de robotique personnel. L'objectif n'est pas de construire rapidement un robot spectaculaire, mais une **plateforme robuste, documentée et modulaire** que je pourrai faire évoluer pendant des années : nouveaux capteurs, nouveau bras, vision, langage, apprentissage.

Chaque fonctionnalité est **comprise, implémentée puis validée sur le matériel réel** avant de passer à la suivante. La vision complète est décrite dans [`MK I/Description du projet.pdf`](MK%20I/Description%20du%20projet.pdf).

### Architecture

Des modules indépendants qui fonctionnent en boucle :

```
Observer → Comprendre → Décider → Agir → Vérifier
```

| Module | Rôle |
|---|---|
| Interface Homme-Machine | Texte, voix, interface graphique, visualisation de l'état du robot |
| Compréhension de la demande | Traduit une demande naturelle en objectifs concrets |
| Cœur décisionnel | Analyse la situation, choisit une stratégie, gère les erreurs |
| Vision artificielle | Détection, localisation et estimation de pose des objets |
| Base de connaissances | Paramètres du robot, objets connus, procédures, historiques |
| Planification des actions | Décompose un objectif en mouvements réalisables |
| Commande robotique | Positions, trajectoires, vitesses, séquences |
| Boucle de retour | Vérifie le résultat de chaque action et corrige les écarts |

Chaque module a une responsabilité unique et peut être remplacé sans toucher aux autres.

### Roadmap

| Phase | Nom | Objectif | Statut |
|---|---|---|---|
| 0 | Préparation | Vision, architecture, roadmap, environnement de dev | ✅ Terminée |
| 1 | **Mk I — Maîtrise du bras** | Mouvements manuels et programmés fiables, poses, séquences, calibration | 🔄 En cours |
| 2 | Mk II — Bras sur rail | Zone de travail plus large, charge utile plus importante | ⏳ Prévue |
| 3 | Vision | Caméra, détection et localisation d'objets | ⏳ Prévue |
| 4 | Manipulation autonome | Pick & place d'objets identifiés | ⏳ Prévue |
| 5 | Interaction utilisateur | Interface graphique, commandes texte et vocales | ⏳ Prévue |
| 6 | Intelligence décisionnelle | Raisonnement sur les tâches confiées | ⏳ Prévue |
| 7–8 | Robotique avancée / plateforme de R&D | Expérimentation libre | ⏳ Prévue |

### Points forts de la phase 1

#### Modèle cinématique du xArm 1S

J'ai mesuré chaque segment du bras, défini les repères des articulations et établi les **paramètres de Denavit–Hartenberg**, les offsets angulaires et les débattements utiles. Le modèle complet (table DH, matrices de transformation homogènes, cinématique directe) est dans [`Talos Studio/Kinematics.ods`](Talos%20Studio/Kinematics.ods).

| Articulation | Servo | α (°) | d (mm) | a (mm) |
|---|---|---|---|---|
| J1 — base | 6 | 90 | 32 | 0 |
| J2 — épaule | 5 | 0 | 0 | 33 |
| J3 — coude | 4 | 0 | 0 | 101 |
| J4 — flexion poignet | 3 | −90 | 0 | 90 |
| J5 — rotation poignet | 2 | 0 | 0 | 52 |
| Outil (pince) | 1 | — | — | 85 |

Bras tendu à la verticale → repère du poignet à **z = 308 mm** au-dessus de la base. C'est ce modèle qui permet au logiciel de savoir où se trouve la pince dans l'espace.

#### Talos Studio — logiciel de contrôle

[`Talos Studio/`](Talos%20Studio/README.md) pilote le bras depuis un ordinateur, en s'appuyant directement sur le modèle cinématique ci-dessus :

- commande par axe avec vues de côté et de dessus à l'échelle, en temps réel
- **cinématique directe** (`fk_pose`) et **cinématique inverse** numérique (moindres carrés amortis, `ik_solve`)
- poses enregistrées, enregistrement et relecture de gestes, séquenceur par blocs
- console native (Tkinter) et console web locale (protégée par jeton, bornage de course côté serveur)
- pilotage par téléphone via l'IMU en UDP (mode articulations ou mode bout de bras par cinématique inverse)

Un mode de suivi par caméra avec marqueur ArUco a été prototypé puis abandonné (raisons détaillées dans le README de Talos Studio).

### Organisation du dépôt

```
Talos/
├── MK I/            Description du projet et documents de la phase 0
├── MK II/           À venir
└── Talos Studio/    Logiciel de contrôle du bras + modèle cinématique
```

### Contributeurs

- **Adam Deruelle** — vision, architecture et roadmap du projet ; modèle cinématique du bras (paramètres DH, offsets, débattements).
- **Harry Troule** — logiciel de contrôle (Talos Studio), développé avec l'assistance de Claude Code.

### Contact

[LinkedIn](https://www.linkedin.com/in/adam-deruelle-/) · [Portfolio](https://adam-deruelle.github.io/) · adam.deruelle@gmail.com

[⬆ Retour en haut](#top)
