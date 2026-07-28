build release cho android# Auto Marker & Drop Point

Two mission item types that make a vehicle stop over a point and cycle a relay to drop a physical marker.

- **Auto Marker** — a survey style pattern. Fills a polygon with a grid of markers.
- **Drop Point** — a single waypoint that drops one marker.

This document is written so the feature can be re-implemented on another platform. Part 1 is the behaviour
specification and is the part that matters for porting: it is pure MAVLink and has nothing to do with
QGroundControl. Part 2 covers the geometry. Part 3 onwards is the QGroundControl implementation and is only
useful if you are working in this codebase or one shaped like it.

---

## 1. Behaviour specification (platform independent)

### 1.1 The marker action

Both item types perform the same action at each marker. One marker expands to **exactly three MAVLink mission
items**:

| # | Command | Frame | param1 | param2 | param3 | param4 | param5/6/7 |
|---|---------|-------|--------|--------|--------|--------|------------|
| 1 | `MAV_CMD_NAV_WAYPOINT` (16) | see 1.4 | `1` (hold secs) | `0` | `0` | `NaN` (yaw unchanged) | lat / lon / alt |
| 2 | `MAV_CMD_DO_REPEAT_RELAY` (182) | `MAV_FRAME_MISSION` (2) | relay number | cycle count `N` | cycle time `N` secs | `0` | `0` / `0` / `0` |
| 3 | `MAV_CMD_NAV_WAYPOINT` (16) | see 1.4 | `N + 0.5` (hold secs) | `0` | `0` | `NaN` (yaw unchanged) | lat / lon / alt |

Both waypoints are at **the same coordinate and altitude**. The vehicle settles on the point (item 1), the
relay cycles (item 2), and the vehicle holds while the relay does its work (item 3).

`N` is the **action number** — see 1.2 and 1.3. The relay number is user settable, range **0–5**, default `0`.

> The `DO_REPEAT_RELAY` parameter names follow the MAVLink spec: param1 = relay instance, param2 = cycle
> count, param3 = cycle time in seconds.

### 1.2 Auto Marker — how N is chosen

Markers are numbered **1, 2, 3, …** in the order they are flown, continuously across the whole mission (the
count does not restart per transect). The action number is:

```
N = ((markerIndex - 1) mod 6) + 1        // markerIndex is 1 based
```

So markers 1–6 use N = 1–6, marker 7 goes back to N = 1, and so on. The intent is that consecutive markers
behave differently enough to be told apart on the ground.

| Marker | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|--------|---|---|---|---|---|---|---|---|
| N      | 1 | 2 | 3 | 4 | 5 | 6 | 1 | 2 |
| Cycles | 1 | 2 | 3 | 4 | 5 | 6 | 1 | 2 |
| Cycle time (s) | 1 | 2 | 3 | 4 | 5 | 6 | 1 | 2 |
| Exit hold (s) | 1.5 | 2.5 | 3.5 | 4.5 | 5.5 | 6.5 | 1.5 | 2.5 |

**Not confirmed by the product owner:** only N = 1, 2, 3 were specified. N = 4, 5, 6 are extrapolated from the
same rule. Confirm before flying.

### 1.3 Drop Point — how N is chosen

`N` is entered directly by the user as **Position**, range **1–6**. There is no sequence: each Drop Point is
independent.

### 1.4 Altitude and frame

- **Auto Marker** inherits the survey altitude modes: relative, absolute, or terrain following. The waypoint
  frame follows the altitude mode, exactly as a normal Survey does.
- **Drop Point** is **relative altitude only**. Frame is always `MAV_FRAME_GLOBAL_RELATIVE_ALT` (3) and
  param7 is metres above home.

### 1.5 Auto Marker: what is *not* emitted

Auto Marker has no camera. It never emits `MAV_CMD_DO_SET_CAM_TRIGG_DIST`, `MAV_CMD_IMAGE_START_CAPTURE`, or
any other camera command, and the reported photo count is always 0.

Transect entry/exit points and turnaround points are **plain waypoints** (`MAV_CMD_NAV_WAYPOINT`, hold 0).
Only the interior points spaced along each transect are markers.

---

## 2. Geometry: where the markers go

This is the part most likely to be got wrong. Read it before re-implementing.

### 2.1 The two spacings

- **Spacing** — distance between transect lines (perpendicular to the lines).
- **Marker Dist** — distance between markers **along** each line.

Together they make a grid of markers over the polygon.

### 2.2 Markers must be measured from a shared origin, not per transect

A survey flies transects **boustrophedon** (alternating direction): line 1 bottom→top, line 2 top→bottom, and
so on. Transects can also start at different offsets when the polygon is not a rectangle.

If markers are placed at `k × MarkerDist` measured **from each transect's own entry point**, they do not line
up between neighbouring lines. Line 1 gets markers at 10 m, 20 m from the bottom; line 2 gets them at 10 m,
20 m from the *top*, i.e. `L−10`, `L−20` from the bottom. They only coincide when the line length `L` is an
exact multiple of `MarkerDist` — effectively never.

**The fix:** define a grid axis and an origin shared by every transect in a pass, and place markers at
multiples of `MarkerDist` measured along that axis.

```
For each transect (entry E, exit X):
    az       = azimuth(E -> X)

    if no grid yet, or az is not parallel to gridAz (mod 180, tol 0.5 deg):
        gridOrigin = E              # this transect's entry; moves/rotates with the polygon
        gridAz     = az

    # signed distance along the grid axis, relative to gridOrigin
    project(P) = distance(gridOrigin, P) * cos(radians(azimuth(gridOrigin, P) - gridAz))

    sE   = project(E)
    sX   = project(X)
    lo   = min(sE, sX);  hi = max(sE, sX)

    # every grid position which falls inside this transect
    S = { k * MarkerDist : integer k, lo < k*MarkerDist < hi }
    drop any s within 1 cm of lo or hi      # would duplicate the entry/exit waypoint

    if sX < sE: reverse S                   # follow the direction of travel

    for s in S:
        d = s - sE
        # walk along THIS transect's azimuth, not the grid axis
        marker = E.atDistanceAndAzimuth(sX < sE ? -d : d, az)
```

Three details that are easy to get wrong:

1. **Order.** The transects alternate direction, so `S` must be reversed on reversed transects or the vehicle
   flies backwards through the markers.
2. **Place along the transect's own azimuth**, not the grid axis. They are parallel but may point opposite
   ways, and if they differ by even a fraction of a degree the marker lands *off* the flight line.
3. **Refly at 90°.** The second pass is perpendicular, so it cannot share the first pass's axis. Detecting
   "not parallel to the current axis" and starting a new origin handles this automatically.

### 2.3 The grid must be re-anchored on every rebuild

Do **not** carry the grid origin over between rebuilds. If the origin stays put while the polygon moves, the
markers slide along the old grid lines instead of travelling with the pattern — and setting the anchor (2.4)
then does not put the first marker where the user asked for it.

### 2.4 Anchor

The first marker doubles as an anchor, but **no anchor is ever stored**. It is purely derived:

- **Read** — reports where the first marker currently is.
- **Write** — moves the *whole pattern* by (new position − current first marker). The rebuild then puts the
  first marker exactly on the requested position and carries every other marker along with it.
- Shown on the map as a larger dot. Deliberately **not draggable** — the pattern is moved by dragging the
  polygon.

> **Do not store the anchor and use it as the grid origin.** That was tried and is wrong: a stored anchor does
> not follow the polygon, so as soon as the user drags or rotates the pattern the anchor is left stranded
> behind, and the grid is then anchored to a dead coordinate. Keeping the anchor derived means there is no
> second source of truth to go stale. The grid origin is always the first transect's entry point (2.2), which
> moves and rotates with the polygon for free.

### 2.5 Imported markers

When markers are imported from a file they **replace the generated grid outright**. The vehicle flies them in
file order; Spacing, Marker Dist, Angle and the anchor no longer apply. No polygon is required.

---

## 3. CSV format

Import and export use the same format. Header line, then one row per marker:

```
index,latitude,longitude,altitude
1,10.762622000,106.660172000,50.00
2,10.762800000,106.660172000,50.00
```

- Latitude/longitude written with 9 decimal places, altitude with 2.
- **Import ignores `index` and `altitude`.** Only columns 2 and 3 (lat, lon) are read. Altitude comes from the
  item's Altitude field — the item has a single altitude and does not support per-marker altitudes.
- The header is detected by columns 2/3 failing to parse as numbers on line 1. A file with no header parses
  fine.
- Rows need at least 3 columns. Any malformed row aborts the whole import with the line number reported, and
  the existing markers are left untouched.

---

## 4. QGroundControl implementation

### 4.1 Files

**New:**

| File | Purpose |
|------|---------|
| `src/MissionManager/AutoMarkerComplexItem.{h,cc}` | Auto Marker item. Derives from `SurveyComplexItem`. |
| `src/MissionManager/AutoMarker.SettingsGroup.json` | `MarkerDistance`, `RelayNumber` fact metadata |
| `src/MissionManager/DropPointComplexItem.{h,cc}` | Drop Point item. Derives from `ComplexMissionItem`. |
| `src/MissionManager/DropPoint.SettingsGroup.json` | `Altitude`, `RelayNumber`, `Position` fact metadata |
| `src/QmlControls/AutoMarkerItemEditor.qml` | Auto Marker editor |
| `src/QmlControls/AutoMarkerMapVisual.qml` | Marker dots + anchor dot |
| `src/QmlControls/DropPointItemEditor.qml` | Drop Point editor |
| `src/QmlControls/DropPointMapVisual.qml` | Drop Point indicator + drag |

**Modified:** `MissionController.{h,cc}` (registration), `SurveyComplexItem.{h,cc}` (made subclassable),
`TransectStyleComplexItem.{h,cc}` (marker hooks), `CameraCalc.h` (`setManualCamera`),
`MissionCommandDialog.qml` + `MissionItemEditor.qml` (Drop Point entry), `TransectStyleComplexItem*.qml`
(camera-free / marker count), `CameraCalcGrid.qml`, `qgroundcontrol.qrc`, two `CMakeLists.txt`.

### 4.2 Auto Marker reuses Survey rather than copying it

`AutoMarkerComplexItem : SurveyComplexItem`. All the transect maths, polygon handling, terrain following and
map visuals come for free. To make `SurveyComplexItem` subclassable it was necessary to:

- relax `final` → `override` on `patternName`, `load`, `save`, `commandDescription`, `commandName`,
  `abbreviation`, `mapVisualQML`, `_rebuildTransectsPhase1`;
- move `_saveCommon` / `_loadV4V5` / `_rebuildTransectsPhase1` to `protected`;
- add `virtual QString complexItemTypeValue()` — `_saveCommon` and `_loadV4V5` used a compile time constant to
  write and check the plan file's `complexItemType`, so a subclass's plans would fail to load;
- add `virtual void _appendInteriorTransectPoints(...)` — Survey's hover-and-capture point insertion moved
  into it verbatim, Auto Marker overrides it to insert markers.

### 4.3 The camera is disabled through QGC's own mechanism

`TransectStyleComplexItem::triggerCamera()` is simply `triggerDistance() != 0`, and **every** camera command
emission is behind that check. So the Auto Marker constructor does:

```cpp
_cameraCalc.setManualCamera();
_cameraCalc.adjustedFootprintFrontal()->setRawValue(0);   // == triggerDistance, so triggerCamera() is false
hoverAndCapture()->setRawValue(false);
cameraTriggerInTurnAround()->setRawValue(false);
```

No fork of the mission generation was needed. `_recalcCameraShots` already guards `triggerDistance == 0` and
reports 0 photos.

`CameraCalc` stores hand entered grid values in manual mode: altitude in `distanceToSurface`, transect spacing
in `adjustedFootprintSide`, trigger distance in `adjustedFootprintFrontal`. `_recalcTriggerDistance()` returns
early when `isManualCamera()`, which is why the user's spacing is never overwritten.

### 4.4 Marker points are real flight path points

Rather than overlaying waypoints, markers are inserted into the transects as a new coord type, mirroring how
hover-and-capture already works:

- `TransectStyleComplexItem::CoordType::CoordTypeInteriorMarker` (runtime only, never serialised).
- `virtual void _appendItemsForInteriorMarker(...)` — emits the three items. Base default: a plain waypoint.
- `virtual int _itemCountForInteriorMarker()` — Auto Marker returns 3.

Terrain following and the map polyline pick the markers up automatically because they are ordinary path
points.

> **The single most dangerous invariant in this feature:** `lastSequenceNumber()` counts mission items
> *without building them*, and the base class comments say it must match `_buildAndAppendMissionItems`. If the
> count and the generation disagree, every sequence number after the item is wrong. Both paths are routed
> through the hook pair above so they cannot drift apart. Preserve this if you refactor.

`_buildAndAppendMissionItems` is virtual so Auto Marker can reset the marker counter at the start of each
build. QGC generates items more than once (save, upload, path calc) — without the reset the second build
carries on counting and the same marker gets a different action.

### 4.5 Settings groups must be separate

`SettingsFact::_rawValueChanged` writes to `QSettings` under its settings group. Auto Marker originally
inherited Survey's group `"Survey"`, which meant forcing the manual camera wrote
`Survey/CameraName = "Manual (no camera specs)"` — so **every newly created Survey defaulted to manual
camera**, and zeroing the trigger distance would have made new Surveys take no photos.

`SurveyComplexItem` therefore has a protected constructor taking a settings group name; Auto Marker passes
`"AutoMarker"`. Presets are namespaced the same way (a Survey preset loaded into Auto Marker would fail the
complex item type check).

### 4.6 Drop Point is a "complex" item but a single waypoint

`ComplexMissionItem` in QGC does **not** mean "area pattern". It means *one visual item → several MAVLink
items*. Drop Point needs three commands, so `SimpleMissionItem` (which is 1:1 with a MAVLink command) cannot
express it. To the user it is a single waypoint: `isSingleItem() == true`, one coordinate, draggable.

It is **not** in `MissionController::complexMissionItemNames()`, so it does not appear in the Pattern menu —
the same way landing patterns are excluded.

### 4.7 Adding Drop Point to Select Mission Command

Every other entry in `MissionCommandDialog` does `missionItem.command = mavCmdInfo.command` — one MAVLink
command on a `SimpleMissionItem`. Drop Point cannot come from the command tree.

It is a separate hard coded entry above the command list. Selecting it calls the new
`MissionController::replaceVisualItemWithComplexItem(visualItem, complexItemName)`, which takes the old item's
coordinate, removes it, and inserts the complex item at the same index. The entry hides itself
(`visible: !!missionController`) if the dialog is used somewhere that does not pass a controller.

**Consequence:** the dialog is shared by all vehicle types, so the Drop Point entry currently shows for every
waypoint on every vehicle, including ones with no relay hardware. Filter it if that matters.

---

## 5. Porting gotchas

Things that cost time here and will cost time again.

**Mission structure**

- Keep the item count and the item generation in one place, or they will drift (4.4).
- Reset any per-build counter at the start of every build — item generation runs more than once (4.4).
- Re-anchor the marker grid per rebuild or moving the pattern leaves markers behind (2.3).

**Geometry**

- Alternating transect direction is the reason naive per-transect spacing does not line up (2.2).
- Place markers along the transect's own azimuth, not the grid axis (2.2).

**QML (only relevant if porting to Qt/QML)**

- `Component.onCompleted` in a derived `.qml` **overrides** the base's handler, it does not run alongside it.
  Overriding it in a map visual silently deletes the base's visuals. Use `Instantiator` / a child item's own
  handler instead.
- `object` in a mission map visual is a **context property**, not a property of the item. Bare `object` works,
  `myRoot.object` is `undefined` — and it fails silently at runtime while the build stays green.
- An item editor is loaded by a `Loader` that has **no size of its own**. The editor root must set
  `width: availableWidth` and an explicit `height`, and provide its own background `Rectangle`. Anchoring to
  `parent` gives a zero sized parent and everything draws on top of everything else.
- A `Q_PROPERTY` whose value is derived from other state needs its `NOTIFY` signal emitted when that state
  changes, or bindings silently keep the stale value. This bit twice: a permanently disabled button, and a map
  label that never updated.
- Declare a fact's metadata map **before** the facts in the member list — a member used in an initialiser list
  before it is assigned gives the facts null metadata and crashes on first use.

---

## 6. Status

**Verified:** `SurveyComplexItemTest` 9/9, `CorridorScanComplexItemTest` 7/7, `MissionControllerTest` 8/8,
`CameraCalcTest` 5/5. Debug and Release build clean. GUI starts with no QML errors. The Survey and Corridor
tests passing is the evidence that the shared base class refactors did not change existing behaviour.

**Not verified — read this before trusting the feature:**

- **There are no tests for Auto Marker or Drop Point.** Everything above was verified by hand. All of these
  are testable at the C++ level without a GUI and are worth writing first if you continue this work: markers
  line up across transects; setting the anchor lands the first marker on it; item count == 3 × marker count;
  export → import round trips; a malformed CSV leaves existing markers intact; plan file round trips.
- Marker actions N = 4, 5, 6 are extrapolated, not confirmed (1.2).
- Reported but never reproduced: "import CSV changes nothing". Suspected to be importing a file exported from
  the same unchanged pattern, in which case identical markers is correct behaviour — unconfirmed.

**Known limitations**

- Drop Point's map label shows `D` in the circle and `D1` beside it. `MissionItemIndexLabel` only ever puts
  `label.charAt(0)` in the indicator; the full label goes to the side. Left as is.
- The Drop Point entry shows for every waypoint on every vehicle type (4.7).
- CSV import ignores the altitude column (3).
- Drop Point supports relative altitude only (1.4).

**Unit tests:** build with `-DCMAKE_BUILD_TYPE=Debug -DQGC_BUILD_TESTING=ON`, then
`QGroundControl --unittest:SurveyComplexItemTest`. `--unittest` is compiled out of Release builds, where the
flag is silently ignored and the normal GUI starts instead.
