# NTRIP / RTCM Correction Injection — Developer Guide

This document describes the **network RTCM correction** feature (NTRIP / TCP / UDP → drone) added to
this QGroundControl build, written so it can be **re-implemented on another platform / GCS**.

- **Part 1** is the behaviour + protocol specification. It is the part that matters for porting: it is
  pure network + NMEA + RTCM3 + MAVLink and has nothing to do with QGroundControl.
- **Part 2** lists the configuration fields.
- **Part 3** onwards is the QGroundControl implementation map, useful only if you work in this codebase
  or one shaped like it.

The whole feature is one idea: **get raw RTCM3 bytes from a network source, forward them to the vehicle
as MAVLink `GPS_RTCM_DATA`, and (optionally) send the rover position back up to the caster as NMEA GGA.**

```
 NTRIP caster / TCP / UDP ──raw RTCM3──► [source] ──► [RTCM→MAVLink] ──GPS_RTCM_DATA──► vehicle(s)
                                              │
                                              ├──► [RTCM 1005/1006 parser] ──► base station lat/lon/alt (map marker)
                                              └──► [file logger] ──► *.rtcm3
 vehicle position ──► [GGA builder, every ~10 s] ──NMEA GGA──► NTRIP caster   (VRS / network RTK only)
```

---

## 1. Behaviour specification (platform independent)

### 1.1 The one thing that must be correct: RTCM → MAVLink `GPS_RTCM_DATA`

This is the injection endpoint. Any correction source (serial GPS, NTRIP, TCP, UDP, file replay) ends
here. If you get only this right, RTK works.

**MAVLink message:** `GPS_RTCM_DATA` (**id 233**). Fields:

| Field | Type | Meaning |
|-------|------|---------|
| `flags` | `uint8` | fragmentation/sequence bitfield (below) |
| `len`   | `uint8` | number of valid bytes in `data` (0–180) |
| `data`  | `uint8[180]` | RTCM3 bytes (zero-padded) |

`180` is the fixed payload length (`MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN`).

**`flags` bitfield** (LSB first):

| Bits | Name | Meaning |
|------|------|---------|
| bit 0 | fragmented | 1 = this RTCM message is split across multiple `GPS_RTCM_DATA` messages |
| bits 1–2 | fragment id | 0..3, position of this fragment within the sequence |
| bits 3–7 | sequence id | 0..31, increments once per **input RTCM chunk**, wraps at 32 |

**Algorithm** (feed it whatever bytes arrive from the source; chunk boundaries do not matter):

```
on_rtcm_bytes(buf):
    if len(buf) < 180:
        msg.flags = (seqId & 0x1F) << 3          # not fragmented
        msg.len   = len(buf)
        msg.data  = buf (zero padded to 180)
        send_to_all_vehicles(msg)
    else:
        fragmentId = 0
        start = 0
        while start < len(buf):
            n = min(len(buf) - start, 180)
            msg.flags = 0x01                       # fragmented
            msg.flags |= (fragmentId++) << 1
            msg.flags |= (seqId & 0x1F) << 3
            msg.len   = n
            msg.data  = buf[start : start+n]
            send_to_all_vehicles(msg)
            start += n
    seqId = (seqId + 1) & 0xFF                     # one increment per input chunk
```

Notes for porting:
- Send to **every connected vehicle** (or the target vehicle) on its primary MAVLink link, using that
  link's channel for encoding.
- You do **not** need to parse RTCM to forward it — pass the bytes through verbatim. Both PX4 and
  ArduPilot accept this and reassemble fragments themselves.
- This is exactly the same message the serial-RTK-base path uses, so a receiver that already supports a
  USB RTK base needs no firmware change.

### 1.2 NTRIP client (the "200 over TCP, not HTTP" case matters)

NTRIP is HTTP-like but has two response dialects. Open a **plain TCP socket** to `host:port`
(do NOT use a high-level HTTP client — the body is an infinite stream). On connect, send the request,
then parse the response, then treat everything after the header as raw RTCM3.

**Request** (works for both v1 and v2 casters):

```
GET /<mountpoint> HTTP/1.1\r\n
Host: <host>:<port>\r\n
Ntrip-Version: Ntrip/2.0\r\n
User-Agent: NTRIP <YourApp>\r\n
Authorization: Basic <base64(username + ":" + password)>\r\n     ← only if credentials given
Connection: close\r\n
\r\n
```

**Response parsing** — accept all three cases:

| First status line | Dialect | Where RTCM begins |
|-------------------|---------|-------------------|
| `ICY 200 OK`  (or any 200 line **not** starting with `HTTP`) | NTRIP v1 / ICY (raw TCP, **not HTTP**) | **immediately after the status line's EOL** — there is NO blank separator line |
| `HTTP/1.1 200 OK` … | NTRIP v2 (HTTP) | after the end-of-headers blank line (`\r\n\r\n` or `\n\n`) |
| contains `SOURCETABLE` | wrong/empty mountpoint | error — no stream |
| any non-`200` (`401`, `404`, …) | rejected | error |

Robustness rules learned from real casters:
- Decide "HTTP vs not" by `statusLine.startsWith("HTTP")`. **Everything else that says `200` is treated
  as NTRIP v1 and streams right after the status line.** Do not require the literal string `ICY` — some
  casters emit other non-HTTP 200 banners.
- Accept **bare `\n`** as well as `\r\n` for line endings (non-compliant casters/proxies exist).
- Keep a bounded header buffer (e.g. 8 KB); if no status terminator arrives within it, treat as invalid.
- After the header, **every byte is RTCM3** — feed it straight into §1.1. Chunk boundaries from the
  socket are arbitrary; never assume one read = one RTCM message.

### 1.3 GGA upstream (VRS / network RTK)

Some mountpoints (VRS, network-RTK/MAC) require the client to periodically send its position so the
caster can generate corrections for that location.

- Send an **NMEA `GGA` sentence every ~10 seconds** while the `Send GGA` option is on.
- Take the position from the connected rover (vehicle). **Cache the last valid position**: if the
  vehicle link drops mid-flight, keep sending the last known position so the caster keeps the stream
  alive (and logging continues). Stop only when the whole correction stream is stopped.

**GGA format** (fields; `*CS` is the NMEA XOR checksum of everything between `$` and `*`):

```
$GPGGA,hhmmss.ss,llll.llllll,N,yyyyy.yyyyyy,E,q,ss,h.h,alt,M,geoid,M,,*CS\r\n
        UTC time  lat ddmm.m   NS lon dddmm.m EW fix sats hdop altM  sep
```

- `lat` = `dd` + `mm.mmmmmm` (degrees then decimal minutes); hemisphere `N`/`S`. Same for lon with
  `ddd` degrees and `E`/`W`.
- `q` = fix quality (1 = GPS is fine), `ss` = satellite count (use a nominal value if unknown, e.g. 10),
  `h.h` = HDOP (nominal 0.9), `alt` = altitude in metres, geoid separation may be `0.0`.
- Write the sentence to the same NTRIP socket.

### 1.4 Source table (the "Get Mountpoints" button)

To list mountpoints, open a TCP socket to the caster and request the **root**:

```
GET / HTTP/1.0\r\n
User-Agent: NTRIP <YourApp>\r\n
Authorization: Basic <...>\r\n     ← only if the caster requires auth for the table
Connection: close\r\n
\r\n
```

The caster replies `SOURCETABLE 200 OK` then lines terminated by `ENDSOURCETABLE`. Read until the socket
closes (HTTP/1.0 `Connection: close`) or `ENDSOURCETABLE` appears, then parse: each **line starting with
`STR;`** is a stream; the **2nd `;`-separated field is the mountpoint name**. Deduplicate + sort.

### 1.5 Raw TCP and UDP sources

- **Raw TCP:** identical to NTRIP but with **no handshake** — connect and treat every received byte as
  RTCM3. (In this implementation it is the NTRIP client with the handshake disabled.)
- **UDP:** bind a local UDP port; every datagram is RTCM3. Feed each into §1.1.

### 1.6 Base station position from RTCM 1005 / 1006 (for a map marker)

To show where the base station is, decode RTCM message types **1005** (stationary ARP) and **1006**
(ARP + antenna height) out of the same stream (a passive tap — never modify the bytes going to the
vehicle).

**RTCM3 frame:**

```
byte 0      : 0xD3 (preamble)
bits        : 6 reserved, then 10-bit big-endian payload length  (bytes 1–2: len = ((b1 & 0x03)<<8) | b2)
payload     : <len> bytes
last 3 bytes: CRC-24Q over (preamble + 2 length bytes + payload)
```

- **CRC-24Q** (a.k.a. Qualcomm): polynomial `0x1864CFB`, init `0`, no reflection, process MSB-first,
  24-bit result. Use it to validate each frame; on mismatch skip 1 byte and resync on the next `0xD3`.
- **Bit fields** (MSB-first bit offsets into the payload):
  - message number: bits `0..11` (12 bits) → must be 1005 or 1006
  - reference station id: bits `12..23`
  - ECEF **X**: bits `34..71` (38-bit **signed**, resolution **0.0001 m**)
  - ECEF **Y**: bits `74..111` (38-bit signed, 0.0001 m)
  - ECEF **Z**: bits `114..151` (38-bit signed, 0.0001 m)
- Convert metres to WGS84 lat/lon/alt (ECEF→geodetic, e.g. Bowring's method: `a=6378137`,
  `f=1/298.257223563`).

### 1.7 File logging

Write the raw RTCM3 byte stream to a file **verbatim** (no framing) so it can be replayed by any RTCM
tool. **Flush after every write** so a crash loses at most the last chunk. Auto-name by timestamp
(`rtcm_YYYY-MM-DD_hh-mm-ss.rtcm3`) in the app's user-visible save folder.

---

## 2. Configuration fields

| Field | Type | Notes |
|-------|------|-------|
| source type | enum | None / Serial / **NTRIP / TCP / UDP** |
| ntrip host / port | string / uint16 | caster address (default port 2101) |
| ntrip mountpoint | string | stream name |
| ntrip username / password | string | Basic auth; password field masked in UI |
| ntrip send GGA | bool | enable VRS position upload |
| tcp host / port | string / uint16 | raw RTCM3 TCP source |
| udp port | uint16 | local listen port |
| log to file | bool | toggle (can be toggled live while streaming) |

Status exposed to UI: connected, source label, stream label, bytes/sec, base lat/lon/alt + station id,
log file name + bytes written.

---

## 3. QGroundControl implementation map

New module: **`src/RTK/`** (compiled unconditionally; links Qt Network + Positioning).

| File | Role |
|------|------|
| `RTCMStreamManager.{h,cc}` | app singleton; owns the worker thread + `RTCMMavlink` + base parser + file logger + GGA timer; `Q_INVOKABLE startStream()/stopStream()/fetchMountpoints()` |
| `RTCMNetworkSource.h` | abstract worker interface: `signals rtcmData / connectedChanged / errorOccurred / bytesReceived` |
| `NtripClient.{h,cc}` | §1.2 handshake + §1.3 GGA; `useNtripHandshake=false` makes it the §1.5 raw-TCP source |
| `UdpRtcmReceiver.{h,cc}` | §1.5 UDP source |
| `NtripSourceTable.{h,cc}` | §1.4 source-table fetch/parse |
| `RtcmBaseParser.{h,cc}` | §1.6 RTCM 1005/1006 decode (CRC-24Q, ECEF→WGS84) |
| `RTCMFileLogger.{h,cc}` | §1.7 logging |
| `NetworkRTCMFactGroup.{h,cc}` + `NetworkRTCMFact.json` | runtime status facts for QML |

Reused, unchanged: **`src/GPS/RTCMMavlink.{h,cc}`** — the §1.1 forwarder (moved above the
`QGC_NO_SERIAL_LINK` guard in `src/GPS/CMakeLists.txt` so it builds without serial support). The serial
RTK path (`src/GPS/GPSRtk.cc`) feeds the exact same class, which is why the network path needs no
firmware-side change.

Settings: `src/Settings/RTKSettings.*` + `RTK.SettingsGroup.json`. Exposed to QML as
`QGroundControl.rtcmStreamManager` in `src/QmlControls/QGroundControlQmlGlobal.*`.
UI: **`src/UI/AppSettings/RTKNtripSettings.qml`** (Application Settings → "RTK / NTRIP"), registered in
that dir's `CMakeLists.txt` + `SettingsPagesModel.qml`. Read-only status also shown in the GPS/RTK
toolbar popup `src/QmlControls/GPSIndicatorPage.qml`. Base marker "RTK Base" in
`src/FlightDisplay/FlyViewMap.qml`.

Threading model (mirror `GPSRtk`): the network source runs on its own worker `QThread`; its `rtcmData`
signal is delivered to `RTCMMavlink` (on the main thread) via a **queued connection** (a value copy of
the `QByteArray` crosses the thread boundary). The GGA timer lives on the main thread and posts the
sentence to the worker.

---

## 4. Recommended UI — values to display for an NTRIP connection

What a good NTRIP/RTCM status panel should surface, grouped by purpose. The **Source** column says where
each value comes from so it can be reproduced on any platform (§ refers to Part 1).

### 4.0 Currently implemented panel (validated live)

The QGC build already ships this status panel (Application Settings → **RTK / NTRIP** → *Status*, and a
read-only copy in the GPS/RTK toolbar popup). Verified end-to-end against the **vngeonet.vn** VRS network
(a Vietnamese CORS/VRS caster) — a real working example:

| Field | Live example | Source |
|-------|--------------|--------|
| Source | `NTRIP` | config source type |
| Stream | `vngeonet.vn:2101/VRS.105_45M3` | config host/port/mountpoint |
| Status | `Connected` | socket state (§1.2) |
| Data Rate | `1399.79 B/s` | bytes/interval (§1.1 input) |
| Base Station | `10.8322363, 106.8223535` | decoded from RTCM **1005/1006** (§1.6) |
| Base Altitude | `2.62 m` | RTCM 1005/1006 (§1.6) |
| Log File | `rtcm_2026-07-20_09-47-10.rtcm3` | file logger (§1.7) |
| Logged | `2.8 KB` | bytes written to log |

This confirms the full chain works: NTRIP v2 connect → RTCM forwarded as `GPS_RTCM_DATA` → 1005/1006
base decode → GGA/VRS upload (the `VRS.*` mountpoint only yields a base near the rover if GGA is being
sent) → file logging. The values below (§4.1–4.6) are the **recommended additions** on top of this
baseline.

### 4.1 Connection
| Value | Meaning | Source |
|-------|---------|--------|
| State + colour dot | Idle / Connecting / Streaming / Error | socket state machine (§1.2) |
| Caster `host:port` | where you are connected | config |
| Mountpoint | which stream | config |
| NTRIP version | v1 (ICY) / v2 (HTTP) | status-line dialect detected in §1.2 |
| Uptime | e.g. `connected 00:03:21` | timer since stream established |
| **Last error** | `401` / `404` / `SOURCETABLE` / socket error | the `errorOccurred` signal — **must be shown; a silent connect-fail is the worst UX gap** |

### 4.2 Stream health
| Value | Meaning | Source |
|-------|---------|--------|
| Data rate (KB/s) | is the stream alive | bytes/interval (§1.1 input) |
| Total received (MB) | volume | running byte counter |
| Last data "N s ago" + **stale warning** | flag when the feed stops | timestamp of last received chunk (warn if > ~5 s) |

### 4.3 Correction content (RTCM message types) — strongest diagnostic
| Value | Meaning | Source |
|-------|---------|--------|
| List of message numbers seen, e.g. `1005/1006` (base), `1074/1077` (GPS MSM), `1084/1087` (GLONASS), `1094` (Galileo), `1124` (BeiDou), `1230` (GLONASS bias) | is the caster sending a **complete** correction set | read the 12-bit message number of every frame using the §1.6 framer (no CRC needed just to list types) |
| **Warning: base present but no observation (MSM) messages** | 1005 alone cannot produce an RTK fix | derived from the type set above |

### 4.4 Base station
| Value | Source |
|-------|--------|
| Station ID, base lat/lon/alt | RTCM 1005/1006 (§1.6) |
| **Baseline distance rover↔base** (km) + warning if large (> ~30–50 km) | great-circle distance between base pos and vehicle pos |

### 4.5 GGA upstream (VRS)
| Value | Meaning |
|-------|---------|
| GGA being sent? + last sent position / time | confirms VRS / network-RTK is working (§1.3) |

### 4.6 Result on the vehicle (closes the loop)
| Value | Meaning | Source |
|-------|---------|--------|
| Vehicle fix type: 3D / DGPS / **RTK Float / RTK Fixed** | did the injected RTCM actually raise the rover to RTK | `GPS_RAW_INT.fix_type` (5 = RTK Float, 6 = RTK Fixed) |
| Satellites, HDOP | fix quality | vehicle GPS telemetry |

**Minimum high-value subset to build first:** last error (4.1), RTCM message-type list + missing-MSM
warning (4.3), vehicle RTK fix type (4.6), baseline distance (4.4). These answer the two questions a user
actually has: *"why didn't it connect?"* and *"is it working / did the drone reach RTK Fixed?"*

---

## 5. Porting checklist (to another GCS / app)

1. **Transport:** a raw TCP client (NTRIP + raw-TCP) and a UDP listener. Not a high-level HTTP client.
2. **NTRIP handshake + response parser** per §1.2 (handle ICY/non-HTTP 200, bare `\n`, SOURCETABLE).
3. **RTCM → `GPS_RTCM_DATA`** per §1.1 — the only firmware-facing piece; get the flags bitfield and the
   180-byte fragmentation right.
4. **GGA upload** per §1.3 if you need VRS, including the last-position cache on link loss.
5. Optional: source-table fetch (§1.4), base-position decode (§1.6), file logging (§1.7).
6. Everything except §1.1 is ordinary networking; §1.1 is the piece that must match the MAVLink spec
   exactly.
