# Színes karikákon átrepülő autonóm drón (DJI Tello, ROS 2, Gazebo)

Szakdolgozat: egy szimulált DJI Tello drón a kamerája alapján megkeresi a színes karikákat, és előre megadott sorrendben (kék → piros → zöld) átrepül rajtuk. A rendszer ROS 2 Foxy alatt fut, Docker konténerben, így bármilyen Linuxos gépen felépíthető. A szimulációhoz videókártya nem kell.

- **Készítette:** Vitkolczi Dániel (NEPTUN: XPGMCH), Miskolci Egyetem, Gépészmérnöki és Informatikai Kar
- **Témavezető:** Bartók Roland

## Gyors indítás

**Feltétel:** Linux (Ubuntu), Docker, és a felhasználó a `docker` csoportban.

```bash
bash start.bash build      # a Docker image elkészítése (első alkalommal pár perc)
bash start.bash start      # a konténer indítása, a parancssor ezután a konténerben van
bash start.bash compile    # a konténerben: a ROS 2 munkaterület lefordítása
bash start.bash demo       # a konténerben: Gazebo, drón, detektor és automatikus misszió egyszerre
```

**Mit látsz a demóban:**
- **Gazebo ablak:** a pálya és a drón.
- **`donut_detector_fuzzy` ablak:** a drón kameraképe a felismert karikákkal és a bizalmukkal.
- **A terminál:** a misszió naplója (ráállás, közelítés, átrepülés).

A demó `Ctrl+C`-re mindent leállít. A szimuláció a processzorral renderel, ezért lassabb gépen akadozhat.

## A `start.bash` parancsai

Menüből (`bash start.bash`) vagy argumentumként (`bash start.bash <parancs>`) is használható.

| Parancs | Hol | Mit csinál |
|---|---|---|
| `build` | gazdagép | a Docker image elkészítése (`Docker/Dockerfile`) |
| `start` | gazdagép | a konténer indítása (a projektmappa fel van csatolva) |
| `start2` | gazdagép | újabb terminál a futó konténerben |
| `remove` | gazdagép | a konténer és az image törlése (a fájlokhoz nem nyúl) |
| `compile` | konténer | a munkaterület fordítása; a fájlok a gazdagép felhasználójáé maradnak |
| `demo` | konténer | minden egyszerre: Gazebo, drón, detektor, misszió |
| `gazebo` | konténer | csak a szimuláció |
| `drone` | konténer | a drón betöltése, illetve visszarakása a kezdőpontra |
| `camera1` | konténer | a nyers kamerakép (rqt) |
| `camera2` / `detector` | konténer | a kontúr alapú detektor megjelenítéssel, illetve anélkül |
| `hough` | konténer | a Hough-kör-kereső detektor (összehasonlításhoz) |
| `hand1` / `hand2` | konténer | billentyűs vezérlés a szimulációban, illetve a valódi drónnal |
| `automatic` | konténer | csak az automatikus misszió |

## Felépítés

```
Kamera (Gazebo) ──/topic_ns/image_raw──▶ tello_camera_contour ──donut_detector/circles──▶ tello_mission
      ▲                                    (kontúrok + FRI bizalom)        (JSON)          (állapotgép + P-szabályzó)
      │                                                                                       │ step()
      └──────────────── /gazebo/set_entity_state (teleportálás) ◀── TelloController ◀─────────┘
                                                     valódi drónnál: /cmd_vel ──▶ tello_driver
```

| Csomag | Feladat |
|---|---|
| `tello_controller` | a drón mozgatása: szimulációban a helyzet közvetlen beállításával, valódi drónnál `/cmd_vel` sebességparanccsal |
| `tello_camera` | `tello_camera_contour`: színmaszk, kontúrok, és a bizalom az egyetem FRI motorjával (`rules/donut_confidence.fbdl`); `tello_camera_hough`: alternatív Hough-kör-kereső |
| `tello_mission` | keresés, ráállás, közelítés, átrepülés; diszkrét PID-osztály (`pid.py`), alapból P-szabályzóként |
| `tello_hand_move` | billentyűs vezérlés |
| `tello_ros` | átvett csomag (drónmodell, üzenettípusok, a valódi drón drivere) |

**Fontosabb paraméterek** (`--ros-args -p név:=érték`):
- **`tello_camera_contour`:**
  - `fuzzy_method` (`fri` az alapértelmezett, `manual` az eredeti kézi függvényekhez);
  - `rules_file` (saját FBDL szabályfájl);
  - `visualize`;
  - `image_topic`.
- **`tello_mission`:**
  - `simulation`;
  - `kp_yaw`, `ki_yaw`, `kd_yaw`, `kp_z`, `ki_z`, `kd_z`, `i_limit`;
  - `skip_stale_frames` (befagyott képkockák kihagyása);
  - `log_file` (képkockánkénti CSV napló).

## Mérési eredmények

| Mappa | Tartalom |
|---|---|
| [`results/pid`](results/pid/README.md) | P, PD és PID szabályzó összehasonlítása; a befagyott képkockák kezelésének hatása |
| [`results/fuzzy`](results/fuzzy/README.md) | kézi fuzzy kontra FRI szabálybázis, 300 képes, ismert helyzetű adathalmazon |
| [`results/hough`](results/hough/README.md) | a Hough-detektor átnézése, összehasonlítás a kontúr alapúval |

## Fizikai szimuláció a `TelloPlugin`-nel

A `tello_ros` Gazebo-pluginje fizikai modellel repülteti a drónt, és ugyanazokat a topicokat és szolgáltatásokat használja, mint a valódi drón drivere (`/cmd_vel`, `/flight_data`, `/tello_action`). A misszió nem ezt használja (lásd lent), de a valódi drónos ág így szimulációban is kipróbálható:

```bash
bash start.bash gazebo     # 1. terminál
bash start.bash drone      # 2. terminál, utána felszállás:
ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'takeoff'}"
```

Felszállás után a drón a `/cmd_vel` parancsokra repül; a `TelloController` `simulation=False` módja is ide küldi a parancsait. A `/cmd_vel` értéke a valódi drónhoz hasonlóan botkitérés: a 0,1 vízszintesen 0,8 m/s, függőlegesen 0,4 m/s, forgásnál 0,31 rad/s.

Két hibát kellett javítani, hogy működjön:
- **A drón modellfájljában a plugin névtere idézőjelekkel szerepelt** (`"topic_ns"`). Ez nem érvényes ROS-név, ezért a plugin betöltéskor hibát dobott, és a Gazebo kikapcsolta.
- **A világfájl mentett állapota kb. 13 959 s-ról indította a szimuláció óráját.** A plugin az akkumulátort a szimuláció abszolút idejéből számolja (6000 s élettartammal), így induláskor lemerültnek látta, és nem engedett felszállni. Az óra most nulláról indul, az akkumulátor 100 percig bírja (szimulációs időben); a Gazebo újraindítása „feltölti”. A mentett állapotot nem szabad törölni, mert a karikák helye csak abban van meg.

## Ismert korlátok

- **Kinematikus szimuláció:** a misszió a drónt a `TelloController`-rel minden lépésben odateszi, ahová lépnie kell (teleportálás). Nincs tehetetlenség és ütközés. A `TelloPlugin` a misszió alatt leszállt állapotban marad, és nem zavarja a teleportálást. A misszió átállítása a fizikai szimulációra (felszállás, újrahangolás, a mérések megismétlése) továbbfejlesztési lehetőség.
- **A célpont szándékosan el van tolva:** a mission a karikát a (450; 290) képpontra igazítja, nem a kép közepére (480; 360). Ez a közelítés során tapasztalt állandó eltérést ellensúlyozza.
- **A bizalmi értéket csak a detektor használja,** a jelöltek közötti választásra; a mission csak a kiválasztott kör helyét nézi.
- **Valódi drón:** a parancsút (`/cmd_vel` → `tello_driver` → `rc` parancsok) a driver emulátorával, a szimulációban pedig a `TelloPlugin`-nel is ki van próbálva, valódi repülés nem volt. A misszió paraméterei a kinematikus szimulációra vannak hangolva.

## Felhasznált külső szoftverek

- [tello_ros](https://github.com/clydemcqueen/tello_ros): Clyde McQueen és Peter Mullen, BSD licenc (`drone_ws/src/tello_ros`).
- [fribe-python](https://github.com/piller-imre/fribe-python) (GPL-3.0) és [exprail-python](https://github.com/piller-imre/exprail-python) (MIT): Piller Imre, Miskolci Egyetem. A Docker image építésekor töltődnek le, rögzített verzióval.
- ROS 2 Foxy, Gazebo 11, OpenCV.
