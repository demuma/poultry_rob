# Mission Executor Rework TODO

## Zielbild

Der MissionExecutor soll Hennen nicht mehr in der Eingangreihenfolge anfahren, sondern kontinuierlich aus aktuellen Detektionen entscheiden, welches Ziel als naechstes den hoechsten Nutzen hat. Die Entscheidung soll Prioritaet, Entfernung, Alter der Detektion und Zielstabilitaet beruecksichtigen.

## Konzept

### Erkenntnisse aus dem Integrationstest

- Docker/Bridge/UDS funktionieren mit:
  - `docker run --rm --network host -v /tmp:/tmp --name dil dil-ros2-humble:latest`
  - danach im Container: `ros2 run poultry_rob_bridge uds_server`
- Der normale `uds_server` soll DIL-nah bleiben: Protobuf-Frames mit Hennen-ID, Position und Prioritaet ueber UDS senden, aber keine HAW-Missionslogik ausfuehren.
- Reproduzierbare HAW-Testfaelle laufen separat ueber `scenario_uds_server`.
- `/dil/frame` liefert aktuell Szenario-Hennen. Die Roboterposition kommt im Test ueber TF `map -> base_link` vom `fake_nav2_server`; echte Roboterpose spaeter ebenfalls bevorzugt ueber TF bzw. Roboter-Odom.
- Der aktuelle Executor startet nach jedem Frame sofort wieder eine neue Mission, weil der FakeNav2-Server direkt Erfolg meldet.
- `Object.priority` kommt im Topic an, wird im Executor aber nicht geloggt und nicht bewertet.
- Nach einem erreichten Target fehlt ein "visited/cooldown" Zustand, deshalb werden dieselben Hennen endlos erneut angefahren.
- TF fuer `base_link` ist nicht vorhanden; die Roboterposition aus dem Frame ist aktuell der sinnvolle Fallback.

### 1. Zielzustand statt statischer Wegpunktliste

- Fuehre eine Target-Tabelle pro Objekt-ID ein.
- Speichere pro Target:
  - `id`
  - `type`
  - `priority`
  - aktuelle `x/y` Position im `map` Frame
  - `first_seen`
  - `last_seen`
  - optional `miss_count`
- Status: `active`, `stale`, `visited`, `in_progress`
- Verwende die Historie nur noch fuer Analyse/Debugging, nicht als primaere Planungsbasis.
- Nach erfolgreichem Besuch soll ein Target fuer `visited_cooldown_sec` nicht erneut angefahren werden.

### 2. Gueltigkeit von Hennen pruefen

- Ein Ziel gilt nur als aktiv, wenn es innerhalb eines konfigurierbaren Zeitfensters wieder gesehen wurde.
- Wenn eine Henne laenger als `target_stale_timeout_sec` nicht gesehen wurde, wird sie nicht mehr angefahren.
- Wenn dieselbe ID an neuer Position auftaucht, wird das Target aktualisiert statt ein alter Wegpunkt abgefahren.
- Optional: Falls IDs in der echten Detektion nicht stabil sind, spaeter Positions-Gating ergaenzen.

### 3. Gewichtete Zielauswahl

Statt eine komplette Route einmalig zu berechnen, soll vor jedem neuen Goal das beste aktuelle Target bestimmt werden.

Vorschlag fuer einen einfachen Score:

```text
score = priority_weight * normalized_priority
      + dwell_weight * normalized_dwell_time
      - distance_weight * normalized_distance
      - stale_weight * normalized_age
```

Interpretation:

- Hohe Prioritaet erhoeht den Score.
- Lange Verweildauer erhoeht den Score.
- Grosse Entfernung senkt den Score.
- Alte Detektionen senken den Score.

Konfigurierbare Parameter:

- `priority_weight`
- `dwell_weight`
- `distance_weight`
- `stale_weight`
- `target_stale_timeout_sec`
- `goal_replan_period_sec`
- `goal_preempt_score_margin`
- `arrival_radius_m`
- `visited_cooldown_sec`
- `max_priority`
- `max_relevant_distance_m`
- `max_dwell_time_sec`

### 4. Replanning waehrend der Mission

- Neue Frames muessen auch verarbeitet werden, wenn bereits eine Mission aktiv ist.
- Vor dem Senden jedes neuen Goals wird neu bewertet.
- Waehrend ein Goal laeuft, kann optional neu geplant werden:
  - konservativ: aktuelles Goal erst beenden, dann neu bewerten.
  - dynamisch: laufendes Goal abbrechen, wenn ein anderes Target deutlich besser ist.
- Start mit konservativer Variante, danach Preemption ergaenzen.

### 5. Besuch/Abschluss eines Targets

- Wenn Navigation zu einem Target erfolgreich war, pruefe:
  - wurde diese Henne kurz vor Ankunft noch gesehen?
  - liegt die aktuelle Position noch nahe genug am angefahrenen Goal?
- Wenn ja: Target als `visited` markieren oder aus aktiven Targets entfernen.
- Wenn nein: Target neu bewerten oder als `stale` markieren.

### 6. Simulation verbessern

- `fake_nav2_server.py` soll Navigation realistischer simulieren:
  - konfigurierbare Robotergeschwindigkeit
  - simulierte Fahrzeit nach Distanz
  - optional periodisches Feedback
  - optional aktuelle Roboterposition publizieren oder intern merken
- Bridge-Simulation soll Szenarien liefern:
  - stehende Henne mit steigender Prioritaet
  - Henne bewegt sich waehrend Navigation
  - neue Henne erscheint naeher
  - neue Henne erscheint mit hoeherer Prioritaet
  - Henne verschwindet vor Ankunft

## Arbeitsschritte

- [x] Datenmodell fuer Targets im MissionExecutor einfuehren.
- [x] Prioritaet aus `Object.priority` uebernehmen und in Logs sichtbar machen.
- [x] Timestamp-Helfer fuer ROS-Zeitstempel und Node-Zeit vereinheitlichen.
- [x] Frame-Verarbeitung so umbauen, dass aktive Targets bei jedem Frame aktualisiert werden.
- [x] Endlos-Neustart derselben Mission durch `visited`/Cooldown verhindern.
- [x] Stale-Target-Filter mit `target_stale_timeout_sec` implementieren.
- [x] Gewichtete Scoring-Funktion implementieren und als Strategie verfuegbar machen.
- [x] Mission-Ausfuehrung von statischer `travel_plan` auf "waehle naechstes Target" umbauen.
- [x] Nach erfolgreicher Navigation Target-Validierung durchfuehren.
- [x] Konservative Neuplanung zwischen Goals implementieren.
- [ ] Optional: Preemption eines laufenden Goals bei deutlich besserem Target implementieren.
- [x] `mission_executor.yaml` um neue Parameter erweitern.
- [x] `fake_nav2_server.py` um simulierte Fahrzeit und Roboterposition erweitern.
- [x] Einfache reproduzierbare Simulationsszenarien fuer Bridge/Fake-Server dokumentieren.
- [x] Lokale Tests oder mindestens kleine pure-Python Unit-Tests fuer Scoring/Target-Filter ergaenzen.
- [x] ROS2-Startbefehle und erwartete Logs in der README oder einer kurzen Testnotiz dokumentieren.
- [x] RViz-Basisvisualisierung fuer Hennen, aktuelles Ziel, Roboterpose und Trajektorie ergaenzen.
- [x] FakeNav2 um Drehung auf der Stelle und angular.z-Odometrie erweitern.
- [x] Grosse deterministische Hennen-Szenarien fuer groesseren Stallbereich ergaenzen.
- [x] Planvorschau als `/mission/planned_target_sequence` fuer RViz ergaenzen.
- [x] Besuchte Hennen aus Zielauswahl und RViz ausblenden, bis sie sich sichtbar bewegen.
- [x] Simulation entfernt Hennen nach Roboterbesuch im Besuchsradius.
- [x] Nav2-Watchdog fuer Action-Server-Ausfall und Recovery nach Power-/Akkuwechsel ergaenzen.
- [x] DIL-nahe `uds_server`-Rolle von HAW-Szenario-/Analyseverhalten trennen.
- [x] Szenario-UDS-Server als separaten `scenario_uds_server` erhalten.
- [x] `target_manager` als separate HAW-Logikschicht fuer stabile interne Targets parallel einfuehren.
- [x] MissionExecutor von `/dil/frame` auf `/mission/tracked_targets` mit `/dil/frame`-Fallback umstellen.
- [x] RViz-Visualisierung von `/dil/frame` auf `/mission/tracked_targets` mit `/dil/frame`-Fallback umstellen.
- [x] TargetManager fuer dichten Stall defensiv konfigurieren: DIL-ID zuerst, raeumliches Merging nur sehr nah.
- [x] Launch-Profil fuer Simulation mit Fake-DIL, Bridge, FakeNav2 und Missionsnodes ergaenzen.
- [x] Launch-Profil fuer echte Roboter-/DIL-Integration ohne FakeNav2 ergaenzen.
- [x] Missions-Launch von Visualisierung/RViz trennen.
- [x] Separaten `visualization.launch.py` fuer RViz/RobotModel/Marker ergaenzen.
- [x] HAW-Dockerfile fuer Bridge, TargetManager, MissionExecutor und optionale Visualisierung ergaenzen.
- [x] Docker-Startskripte fuer Mission, Visualisierung und Simulation ergaenzen.
- [ ] Optional: RViz-Visualisierung um Score-Werte und Target-Status erweitern.
- [ ] Roboter über Rviz Set Goal zu einem Target schicken

## Erste Umsetzungsreihenfolge

1. Target-Datenmodell und Aktualisierung aus Frames.
2. Endlos-Neustart verhindern: visited/cooldown nach erfolgreicher Navigation.
3. Gewichtete Scoring-Funktion ohne Preemption.
4. Stale-Filter und Validierung nach Ankunft.
5. MissionExecutor auf dynamische Next-Target-Auswahl umbauen.
6. FakeNav2 realistisch genug machen, um Replanning beobachten zu koennen.
7. Optionales Goal-Abbrechen/Preemption.

## UDS-Rollen

- `uds_server`: minimaler Fake-DIL-Server. Sendet nur getrackte Hennen mit Detektor-ID, Position und Prioritaet.
- `scenario_uds_server`: HAW-Testdatenquelle mit reproduzierbaren Szenarien.
- `uds_bridge_node`: UDS/Protobuf nach ROS2 `/dil/frame`; macht selbst keine Tracking- oder Missionslogik.

Langfristiges Ziel:

```text
DIL oder Fake-DIL -> UDS/Protobuf -> uds_bridge_node -> /dil/frame
                                                       -> target_manager
                                                       -> /mission/tracked_targets
                                                       -> mission_executor
```

## Reproduzierbare Simulationsszenarien

Der `poultry_rob_bridge` Szenario-UDS-Server kann ueber `UDS_SCENARIO` gesteuert werden:

```bash
UDS_SCENARIO=basic ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=new_near_hen ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=priority_ramp ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=new_high_priority_far ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=hen_disappears_before_arrival ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=hen_moves ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=many_hens_uniform ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=many_hens_clusters ros2 run poultry_rob_bridge scenario_uds_server
UDS_SCENARIO=many_hens_hotspot ros2 run poultry_rob_bridge scenario_uds_server
```

- `basic`: zwei stehende Hennen, eine davon mit hoeherer Prioritaet.
- `new_near_hen`: zwei stehende Hennen zu Beginn; nach ca. 3 Sekunden erscheint eine neue niedrig priorisierte Henne naeher an der erwarteten Roboterroute.
- `new_near_low_priority`: Alias fuer `new_near_hen`.
- `priority_ramp`: eine sitzende Henne steigt alle 15 Sekunden von Prio 0 bis 3; eine zweite niedrig priorisierte Henne bleibt naeher.
- `new_high_priority_far`: eine nahe Prio-0-Henne ist zuerst da; nach ca. 3 Sekunden erscheint eine weiter entfernte Prio-3-Henne.
- `hen_disappears_before_arrival`: eine Prio-2-Henne verschwindet nach ca. 4 Sekunden; eine zweite Henne bleibt sichtbar.
- `hen_moves`: dieselbe Hennen-ID wechselt nach ca. 3 und 6 Sekunden ihre Position.
- `many_hens_uniform`: viele deterministische Hennen in einem groesseren Feld.
- `many_hens_clusters`: viele deterministische Hennen in mehreren Clustern.
- `many_hens_hotspot`: viele deterministische Hennen mit dichter, hoeher priorisierter Region.

Die Szenarien senden absichtlich kein `ROBOT`-Objekt mehr. Die aktuelle Roboterposition soll im Integrationstest vom `fake_nav2_server` ueber TF `map -> base_link` kommen.

Der FakeNav2-Server schreibt erreichte Zielpositionen nach `/tmp/poultry_robot_visits.jsonl`. Nur der `scenario_uds_server` liest neue Events und entfernt Hennen im Radius `UDS_VISIT_CLEAR_RADIUS_M` aus den folgenden Frames. Damit wird simuliert, dass Hennen nach Roboterbesuch verscheucht werden.

## RViz-Visualisierung

Die Launch-Datei startet zusaetzlich `mission_visualizer`. Die Node publiziert:

- `/visualization_marker_array`: Feldrahmen, Hennen, Prio-Labels, Roboterpose, aktuelles Ziel fuer RViz.
- `/mission/visualization_markers`: gleiches MarkerArray als projektspezifisches Debug-Topic.
- `/mission/robot_path`: akkumulierte Robotertrajektorie als `nav_msgs/Path`.
- `/mission/planned_target_sequence`: aktuelle Planvorschau als `nav_msgs/Path`; keine garantierte vollstaendige Route.
- `/mission/current_goal`: aktuelles Ziel aus dem MissionExecutor als `PoseStamped`.
- `/mission/visited_target_ids`: besuchte Hennen, die in RViz ausgeblendet und in der Zielauswahl ignoriert werden.
- `/odom`: FakeNav2-Odometrie; spaeter kann hier reale Roboter-Odometrie angezeigt werden.
- `/robot_description`: optionales URDF-RobotModel, wenn der Launch mit `use_robot_description:=true` gestartet wird.

RViz-Konfiguration:

```bash
rviz2 -d /home/maxdemu/Documents/ros2-ws/install/high_level_mission_planer/share/high_level_mission_planer/rviz/mission_visualization.rviz
```

URDF mit FakeNav2 koppeln:

```bash
ros2 launch high_level_mission_planer visualization.launch.py use_robot_description:=true
```

Der FakeNav2-Server liefert `map -> base_link`; `robot_state_publisher` haengt daran die URDF-Links an. Standard ist eine schlanke Visualisierungs-URDF im `high_level_mission_planer`-Package. Die originale Xacro-`robot_description` aus dem Roboter-Repo kann spaeter ueber deren eigenen Launch genutzt werden, sobald `xacro` und die Hardware-Abhaengigkeiten installiert sind.
