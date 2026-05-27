# Kooperatív 2 robotos  Térképezési Projekt kiterjesztése több világra
## Multi-robot exploration
**Important:** first launch the launch file *(multirobot_navigation_slam_toolbox.launch.py)*, and then run the exploration logic **in a separate terminal** *(explore_map2)*

**First:**

```bash
ros2 launch multi_robot_navigation multirobot_navigation_slam_toolbox.launch.py
```

**Then in a separate terminal window:**
```bash
ros2 run multi_robot_explore explore_map2
```

## Python dependency
I'm not sure, but if stuff isn't working out for you, you could try installing the Python package shapely.

First make sure that your system is up-to-date:

```bash
sudo apt update
```

If it is needed, you can upgrade:

```bash
sudo apt upgrade
```

Then install dependency:

```bash
sudo apt install python3-shapely
```

Don't forget to build (```colcon build```) and source the bash file (```source install/setup.bash```)!

## Improved explore: any robots
The exploration logic was modified: instead of two hardcoded robots, this new exploration logic now scans for robots automatically.
I created a new file for this exploration logic: ***multi_robot_explore/multi_robot_explore/explore_map_any_robots.py***, which can be started via this command:
```bash
ros2 run multi_robot_explore explore_map_any_robots
```
**As it handles available robots automatically, it is not important if you start this node before or after the launchfile.**

### Scanning procedure:
The scanning procedure is started every 10 seconds via a timer callback. From all available topics it finds the ones ending with *"/robot_description"*, assuming every robot publihes this kind of topic. If the robot name is not contained in the robot names dictionary, the robot is added to the appropriate internal dictionaries, to handle this new robot.

### Marker colors
It is important to have distinct marker colors for each robot. Instead of the hardcoded colors for *robot_1* and *robot_2*, a distinct color is generated from the robot name. A uniform hash function is used to map the robot names to hue values between 0 and 1. This means, that every robot will have a different marker color, and the same robot will always get the same marker color, at every launch.

### Blacklisting
Originally a potentially unreachable goal position was only blacklisted, if the robot was closer to it, then 1 m. However, this made robots stuck, as there can be unreachable positions further away, so this threshold was increased to 5 m, which resolved the robot-is-stuck issue.


# Az explore_map_any_robots.py Továbbfejlesztése és Holtpont-kezelése

A projekt során az eredeti `explore_map_any_robots.py` kód jelentős architekturális és logikai átalakításon esett át. Az eredeti implementáció (amely a Nav2 Action Servert használta és statikus feketelistákat alkalmazott) szűk folyosókon és dinamikus akadályok (pl. a másik robot) esetén elakadást idézett elő (deadlock). Valamint az eredeti alapkódban volt egy hiba ami a sarkokba való beragadáshoz vezetett. 

Az alábbi dokumentáció részletezi a robusztusabb, hiba-ellenálló működés érdekében bevezetett módosításokat.

---

## Tartalomjegyzék

1. [Könyvtárak és Kommunikációs Architektúra Cseréje](#1-könyvtárak-és-kommunikációs-architektúra-cseréje)
2. [Új Adatstruktúrák az Állapotkezeléshez (State Management)](#2-új-adatstruktúrák-az-állapotkezeléshez)
3. [Új és Módosított Függvények (A Maglogika)](#3-új-és-módosított-függvények)
   - [clean_expired_blacklists](#clean_expired_blacklists)
   - [check_and_blacklist_stuck_targets](#check_and_blacklist_stuck_targets)
   - [get_home_pose](#get_home_pose)
   - [map_callback](#map_callback)
   - [publish_selected_frontier & publish_blacklist_markers](#publish_selected_frontier-és-publish_blacklist_markers)
4. [Navigációs Paraméterek Javítása](#4-navigációs-paraméterek-javítása)
5. [Összegzés](#5-összegzés)

---

## 1. Könyvtárak és Kommunikációs Architektúra Cseréje

* **Új importok:** Bekerült a `ClearEntireCostmap` szerviz hívása. Erre a robotok "memóriájának" programozott tisztításához. Szintén bekerült a `hashlib` és a `colorsys` a dinamikus, robot-specifikus vizualizációhoz.

```python
from nav2_msgs.srv import ClearEntireCostmap
import hashlib
import colorsys

```

* **Action Server helyett Publisher:** Az eredeti kód `ActionClient`-et használt a `MapsToPose` híváshoz. Ezt lecseréltük egy közvetlen `Publisher`-re, amely a `/{robot_name}/goal_pose` topicra küldi a célokat.
* Ez azért vol hasznos mert az Action Server szigorú állapotgépe hajlamos volt beragadni. A közvetlen topic-publikálás azonnali útvonal-újratervezést kényszerít ki a Nav2-ből, felgyorsítva a felderítést.



## 2. Új Adatstruktúrák az Állapotkezeléshez

Az osztály `__init__` függvényében a statikus változókat lecseréltük dinamikus, időalapú szótárakra (Dictionaries), amelyek tárolják az állapotokat:

```python
# Célpontok kizárása időbélyeggel: {robot_name: {(y, x): timestamp}}
self.blacklists = {}          

# Menekülési fázis időzítője: {robot_name: timestamp}
self.retreat_until = {}       

```

## 3. Új és Módosított Függvények

### `clean_expired_blacklists`

```python
def clean_expired_blacklists(self):

```

* **Működése a következő:** Minden térképfrissítéskor végigiterál a `blacklists` szótáron, és törli azokat a bejegyzéseket, amelyek régebbiek 60 másodpercnél.
* Így az eredeti kód véglegesen kizárt területeket. Az időbélyeges "felejtés" garantálja, hogy a dinamikus akadályok távozása után a rendszer újra megpróbálja feltérképezni a kimaradt részeket.

### `check_and_blacklist_stuck_targets`

```python
def check_and_blacklist_stuck_targets(self, map_msg):

```

Ez a rendszer legfontosabb része a módosítsoknak, amely a holtpontok feloldásáért felel. Három lépésben avatkozik be, ha egy robot 10 másodpercig megakad (Deadlock):

1. **Costmap Törlése:** Meghívja a `ClearEntireCostmap` szervizt a `local_costmap` és a `global_costmap` esetében is, törölve a fals akadályokat a memóriából.
2. **Időleges Balcklist:** Ha a robot a cél 5 méteres körzetében ragadt be, a célpontot hozzáadja a `blacklists` szótárhoz a jelenlegi időbélyeggel.
3. **Menekülő Manőver:** Beállítja a `retreat_until` időzítőt `aktuális_idő + 15 másodperc` értékre, és új célként a robot kezdőpontját (`home_pose`) adja meg.

*Példa a Costmap törlési hívásra a függvényből:*

```python
client_local = self.create_client(ClearEntireCostmap, f'/{robot}/local_costmap/clear_entirely_local_costmap')
if client_local.wait_for_service(timeout_sec=0.5):
    client_local.call_async(ClearEntireCostmap.Request())

```

### `get_home_pose`

```python
def get_home_pose(self, map_msg, robot_name):

```

* **Működése:** Kiszámolja a robot saját `/map` koordináta-rendszerének origóját (0,0), és a `tf2` segítségével transzformálja azt a közös `world` keretbe.
* Ez biztosítja a garantált visszavonulási pontot. Mivel a robotok a térkép különböző pontjairól indulnak, a "hazaküldésük" szétválasztja őket egy szűk folyosón történt találkozás esetén de természetesn nem mennek el tlejenen a kiinduló pozícióig csak 15 másodpercig mennek annak az irányábamajd új "úticélt" kapnak. Ez lehetőséet ad hogy új célokkal új útvonalakon próbálkozva addig próbáljanak elemnni egymás mellett amíg nem sikerül.

### `map_callback`

```python
def map_callback(self, msg):

```

* Elsőként meghívja a tisztító és az elakadásellenőrző rutinokat.
* De még mielőtt új felderítési célt adna egy robotnak, ellenőrzi a `retreat_until` szótárat. Ha a robot épp 15 másodperces visszavonulásban van, **nem kap új célt** (`continue`), így a Nav2 zavartalanul végrehajthatja a kikerülési manővert.

### `publish_selected_frontier` és `publish_blacklist_markers`

* Ez a vizuális diagnosztikai eszközök (Markers) publikálása az RViz számára.
* Ezek a markerek teszik lehetővé számunkra, fejlesztőknek, hogy valós időben lássuk a szoftver döntéseit. A kiválasztott célpontok a robot saját egyedi színével nagy gömbként, míg a tiltólistás pontok kis piros gömbökként jelennek meg.

---

## 4. Navigációs Paraméterek Javítása

A robotok sarokban történő beragadásának egyik okaként azonosítottuk, hogy a `global_costmap` és a `local_costmap` konfigurációjában inkonzisztencia volt, amely a `navigation_1.yaml` és `navigation_2.yaml` fájlokban egyaránt jelentkezett.

### A probléma gyökere: Nem Egyező Robot-rádiusz
Eredetileg a robot-rádiusz (`robot_radius`) paraméterek eltértek a két réteg között:
* **Local Costmap:** `0.2m` (A robot itt "kicsinek" érezte magát, ezért bátran bement a szűk sarkokba.)
* **Global Costmap:** `0.4m` (A globális tervező viszont "nagynak" látta a robotot, és a sarokba érve azt érzékelte, hogy a robot félig belelóg a falba.)

**Eredmény:** Amikor a robot beállt a sarokba, a globális útvonaltervező (Global Planner) ütközést észlelt a `0.4m`-es sugár miatt, és **megtagadta a további útvonaltervezést**. A robot így véglegesen beragadt a sarokban, mivel nem tudott kijelölni utat a biztonságos területre.

### Alkalmazott javítások
A konfigurációs fájlokban (`navigation_1.yaml`, `navigation_2.yaml`) az alábbi egységesítést és optimalizálást hajtottuk végre:

```yaml
# Robot-rádiusz egységesítése a tervezési hibák elkerüléséhez
local_costmap:
  robot_radius: 0.2
  
global_costmap:
  robot_radius: 0.2  # Módosítva 0.4-ről, hogy konzisztens legyen

# Mozgási optimalizációk
footprint_clearing_enabled: true  # Dinamikus akadályok törlése
inflation_radius: 0.4           # A biztonsági zóna szűkítése a jobb mozgékonyságért
cost_scaling_factor: 7.0        # Meredekebb költség-gradiens a falak mentén

## 5. Összegzés

A fenti kiegészítésekkel az `explore_map_any_robots.py` igyekeztünk az eddigi elakadásokat dinamikusan beavatkozással elkerülni. Így már a robotok kooperatív kikerülési és elakadási manővereket tudnak végrehajani és lekezelik a nem állandó akadályok (pl másik robot de akár emberk) jelenlézéz is ezzel javítva a sikeresebb és pontosabb  a térképezés végrehajtását.
```

```