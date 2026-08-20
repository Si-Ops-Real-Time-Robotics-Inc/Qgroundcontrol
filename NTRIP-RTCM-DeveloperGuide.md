# NTRIP / RTCM Correction Injection — Developer Guide

This document describes the **RTCM correction injection** feature (NTRIP / TCP / UDP / Bluetooth → drone)
added to this QGroundControl build, written so it can be **re-implemented on another platform / GCS**.

- **Part 1** is the behaviour + protocol specification. It is the part that matters for porting: it is
  pure network + NMEA + RTCM3 + MAVLink and has nothing to do with QGroundControl.
- **Part 2** lists the configuration fields.
- **Part 3** onwards is the QGroundControl implementation map, useful only if you work in this codebase
  or one shaped like it.

The whole feature is one idea: **get raw RTCM3 bytes from some source, forward them to the vehicle
as MAVLink `GPS_RTCM_DATA`, and (optionally) send the rover position back up to the caster as NMEA GGA.**

```
 NTRIP caster / TCP / UDP ──raw RTCM3──┐
 Bluetooth SPP receiver ──raw RTCM3────┴─► [source] ──► [RTCM→MAVLink] ──GPS_RTCM_DATA──► vehicle(s)
                                              │
                                              ├──► [RTCM 1005/1006 parser] ──► base station lat/lon/alt (map marker)
                                              └──► [file logger] ──► *.rtcm3
 vehicle position ──► [GGA builder, every ~10 s] ──NMEA GGA──► NTRIP caster   (VRS / network RTK only)
```

Every source is interchangeable below the `[source]` box: pick a transport, and the rest of the chain
(§1.1 forwarding, §1.6 base decode, §1.7 logging) is identical.

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
| bits 3–7 | sequence id | low 5 bits of a free-running counter that increments once per **input RTCM chunk** |

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
- **The fragment id is only 2 bits, so one input chunk may not exceed 4 × 180 = 720 bytes.** The loop
  above does not enforce that: a longer chunk lets `fragmentId` reach 4, which shifts into the
  sequence-id bits and corrupts reassembly on the vehicle. Reads from a TCP/NTRIP socket can exceed
  720 bytes after a stall or with large MSM7 sets, so slice the input into ≤ 720-byte chunks (each
  with its own sequence id) before entering the loop. QGC did have this bug; `RTCMMavlink::
  RTCMDataUpdate()` now does exactly that slicing and delegates each chunk to `_sendSequence()`.
- Send to **every connected vehicle** (or the target vehicle) on its primary MAVLink link, using that
  link's channel for encoding.
- You do **not** need to *decode* RTCM to forward it — the payload passes through verbatim. You
  should still **frame-validate** it first (§1.6): whatever arrives on the socket is not necessarily
  RTCM, and a caster error page, an NMEA stream or line noise forwarded blindly goes straight into the
  vehicle's GPS. Validate framing + CRC, forward only whole valid frames, drop the rest. Both PX4 and
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

The frame walk below serves two purposes: it decodes RTCM **1005** (stationary ARP) and **1006**
(ARP + antenna height) for a map marker, and — more importantly — it is the **validation gate**
that decides what is allowed onto the MAVLink link at all (see the note in §1.1). Forward only
frames that pass it; count everything else as discarded and surface that in the UI.

**RTCM3 frame:**

```
byte 0      : 0xD3 (preamble)
bits        : 6 reserved, then 10-bit big-endian payload length  (bytes 1–2: len = ((b1 & 0x03)<<8) | b2)
payload     : <len> bytes
last 3 bytes: CRC-24Q over (preamble + 2 length bytes + payload)
```

- **CRC-24Q** (a.k.a. Qualcomm): polynomial `0x1864CFB`, init `0`, no reflection, process MSB-first,
  24-bit result. Use it to validate each frame; on mismatch skip 1 byte and resync on the next `0xD3`.
- Check the **6 reserved bits** (they are always zero) and reject a zero length before trusting the
  header. Without that, a stray `0xD3` can claim a ~1 kB payload and stall forwarding until that many
  more bytes arrive, instead of resyncing on the very next byte.
- **Bit fields** (MSB-first bit offsets into the payload):
  - message number: bits `0..11` (12 bits) → must be 1005 or 1006
  - reference station id: bits `12..23`
  - ECEF **X**: bits `34..71` (38-bit **signed**, resolution **0.0001 m**)
  - ECEF **Y**: bits `74..111` (38-bit signed, 0.0001 m)
  - ECEF **Z**: bits `114..151` (38-bit signed, 0.0001 m)
- Convert metres to WGS84 lat/lon/alt (ECEF→geodetic, e.g. Bowring's method: `a=6378137`,
  `f=1/298.257223563`).

### 1.7 File logging

Write the RTCM3 byte stream to a file **verbatim** (no framing of your own) so it can be replayed by
any RTCM tool. Tap it **downstream of the §1.6 validation gate**, the same place the MAVLink forward
hangs off: the log is then a clean RTCM3 recording that replays, rather than one with a caster error
page or NMEA spliced into the middle. **Flush after every write** so a crash loses at most the last
chunk. The formats are not exclusive - writing the raw `.rtcm3` alongside the decoded `.obs`/`.nav`
costs a few KB/s and keeps everything the decoder chose not to represent, which is the difference
between diagnosing a bad session later and guessing. Derive all three names from one base so they
obviously belong to the same session. When no path is configured, auto-name by timestamp (`rtcm_YYYY-MM-DD_hh-mm-ss.rtcm3`) inside an
`RTCMLogs/` folder under the app's **user-visible save folder** — not a private app directory, or the
user cannot get the file off a tablet (fall back to app-data, then temp, if that folder is unset).
Opening in append mode plus a live on/off toggle means logging can be started mid-stream without
losing what is already on disk.

### 1.7b RINEX observation output (PPK)

Raw RTCM3 is a correction stream, not an archive format. Post-processing tools want RINEX, and
getting there is a **full decode**, not a reformat: the observables live inside the MSM messages and
have to be unpacked satellite by satellite, signal by signal.

What the decode involves, in the order the bits appear:

- **MSM header** - 12 bit message number, 12 bit station id, 30 bit GNSS epoch time, the
  multiple-message bit (DF393), then 19 bits of indicators, a 64 bit satellite mask, a 32 bit signal
  mask, and a satellites x signals cell mask. The masks are what make the message variable length.
- **Satellite blocks** - rough range integer ms (8 bits), extended satellite info (4 bits, MSM5/7
  only), rough range mod 1 ms (10 bits), rough phase range rate (14 bits, MSM5/7 only). Each field
  is emitted for *every* satellite before the next field starts, not interleaved per satellite.
- **Signal blocks** - fine pseudorange, fine phase range, lock time, half-cycle ambiguity, CNR, and
  (MSM5/7) fine phase range rate, one array per field over the set cells. MSM6/7 use wider fields
  and finer scaling than MSM4/5.
- **Reconstruction** - `range = (roughMs + roughMod * 2^-10) * c/1000`, then add
  `fine * 2^-24 * c/1000` for the pseudorange and `fine * 2^-29 * c/1000` for the phase range
  (MSM6/7: `2^-29` and `2^-31`). Carrier phase in cycles is the phase range divided by the
  wavelength, so **every signal needs its carrier frequency**.
- **Signal id -> RINEX code** - a per-constellation table (RTCM signal 2 is GPS `1C`, 10 is `2W`,
  and so on). Get this wrong and the file is plausible but useless.
- **GLONASS is the awkward one** - FDMA, so L1/L2 frequency depends on the satellite's channel
  number: `1602.0 + k*0.5625 MHz` and `1246.0 + k*0.4375 MHz`. `k` comes from the extended
  satellite info, which **only MSM5 and MSM7 carry**. With MSM4/MSM6 you can still emit GLONASS
  pseudoranges but not carrier phase, unless you also decode message 1020.

Two things the format does not give you:

- **No week number.** MSM carries a time of week only, so the GPS week has to come from the system
  clock. A machine whose clock is more than half a week out will write the wrong date.
- **Epoch assembly.** Each constellation arrives as its own message for the same instant. Group them
  and emit when DF393 clears or when the time of week moves on, whichever comes first. Normalise
  BeiDou first - its time of week runs 14 s behind GPS.

The header needs 1005/1006 for `APPROX POSITION XYZ`, 1008/1033 for the antenna and receiver
descriptors, and the observation-type list per constellation - which is only known after data has
been seen, so buffer the first few epochs before committing to a header.

**Scope note:** this yields the *base station's* observations. PPK needs the rover's raw
observations too, and those come off the vehicle's receiver, not the correction stream.

### 1.7c RINEX navigation output (broadcast ephemeris)

An `.obs` on its own will not post-process: the solver needs broadcast ephemeris to compute
satellite positions. That rides in the same stream, in its own message per constellation:

| Message | Constellation | Bits | Notes |
|---------|---------------|------|-------|
| 1019 | GPS | 488 | Keplerian, 10 bit week (wraps - un-roll it) |
| 1020 | GLONASS | 360 | state vector, **sign-magnitude fields** |
| 1042 | BeiDou | 511 | wider correction terms, 8 s time resolution |
| 1044 | QZSS | 485 | GPS parameters in a different field order |
| 1045 | Galileo F/NAV | 496 | one broadcast group delay |
| 1046 | Galileo I/NAV | 503 | two group delays, wider health word |

The traps, in the order they bite:

- **GLONASS is sign-magnitude.** The top bit is a sign flag, not a two's-complement sign. Reading
  it the usual way gives plausible magnitudes with wrong signs, and an orbit radius that is
  nonsense. Everything else in the family is two's complement.
- **The week wraps.** GPS and QZSS broadcast 10 bits, so the week has to be lifted into the current
  era using the wall clock. Galileo's week is offset instead: `gps_week = galileo_week + 1024`.
  BeiDou counts its own weeks from 2006-01-01.
- **Time systems differ per constellation.** RINEX records each in its own system - GPS/QZSS and
  Galileo in GPS time, BeiDou in BeiDou time, GLONASS in **UTC** (its `tb` is a Moscow-time slot,
  so subtract three hours, and the date comes from `N4`/`Nt`).
- **Satellites rebroadcast constantly.** De-duplicate on satellite + reference time + issue of
  data, or a few minutes of stream produces hundreds of identical records.
- **Numbers are Fortran `D19.12`,** i.e. `-0.123456789012D-04`, not C's `1.23E-05`.

Record layouts differ: GPS/QZSS/Galileo/BeiDou use eight lines (epoch + clock, then seven broadcast
orbit lines); GLONASS uses four. The header depends on nothing data-derived, so unlike the
observation file it can be written the moment the file opens.

### 1.8 Bluetooth source (classic RFCOMM / Serial Port Profile)

For an RTK receiver or a base-station radio that exposes a **Bluetooth serial bridge**. This is the
tablet-in-the-field case: no network, no cable — the receiver sits on the tripod and the GCS pulls its
RTCM over Bluetooth.

Once the RFCOMM stream is open it is **exactly the raw-TCP case (§1.5)**: every received byte is RTCM3,
feed it straight into §1.1. Everything specific to Bluetooth is in getting the stream open.

**Prerequisite:** the device must already be **paired at the operating-system level**. Do not implement
pairing/PIN entry in the GCS — pairing is an OS dialog on every platform, and a socket to an unpaired
device fails with a permission/authentication error.

**Discovery (device picker):** run a device inquiry and list `name` + `address` (`AA:BB:CC:DD:EE:FF`).
Store both: the address is what you connect to, the name is what the user recognises. Persist the
address in settings — the user should not rescan on every app start. On iOS there is no visible BD
address; store the platform device UUID instead and match on that.

**Connecting — the part that bites.** The SPP service UUID is
`00001101-0000-1000-8000-00805F9B34FB`. Two rules learned from a real receiver (GNSS-5804510):

1. **A receiver commonly publishes more than one SPP record** — e.g. `COM0` on RFCOMM channel 1 and
   `COM1` on channel 2, often carrying different data (corrections on one, NMEA/config on the other).
   So do a **service discovery restricted to that device + the SPP UUID, collect every record, and then
   connect once to a deterministic one — the lowest RFCOMM channel.** Connecting to "whichever record
   the discovery callback delivered first" is a trap: the order varies between runs, so the user
   silently gets the wrong port some of the time.
2. **Do not let the socket API run its own discovery** if it has a "connect to address + UUID" helper
   that does so internally. In Qt 6.8 that helper (`QBluetoothSocket::connectToService(address, uuid)`)
   crashes on exactly the multi-record devices above: its `serviceDiscovered()` slot connects, then sets
   its internal discovery-agent pointer to null **without disconnecting the agent**, so the second
   record re-enters the slot and dereferences the null pointer (access violation inside QtCore, 100%
   reproducible). Driving the discovery yourself avoids the whole class of problem and gives you rule 1
   for free.

**Platform notes:**

| Platform | Note |
|----------|------|
| Windows | The Bluetooth stack initialises COM **on the main thread only**. Run the socket on the main thread (a correction stream is a few KB/s — it does not need a worker thread), or the backend operates without a valid COM apartment and crashes. |
| Android | Connecting directly to address + SPP UUID is native and safe (no service discovery involved). Needs the **runtime** permissions `BLUETOOTH_SCAN` + `BLUETOOTH_CONNECT` (API 31+) / `ACCESS_FINE_LOCATION` (API ≤ 30) — request them before scanning **and** before connecting, and retry the action from the permission callback. **Do not probe for a local adapter before that grant:** the platform reports *no adapter at all* to an app without `BLUETOOTH_CONNECT` (Qt logs `Local device allDevices() failed due to missing permissions` and returns an empty list), so an "is Bluetooth available?" check at page load always answers *no* on a fresh install. See the availability rule in §4.7. |
| iOS | Classic RFCOMM is restricted to MFi-programme accessories; treat as best-effort. Match the device by its UUID during service discovery since no BD address is exposed. |

**No GGA upstream:** §1.3 does not apply — a Bluetooth receiver is a local base, not a caster.

**Reconnection:** none is implemented; a dropped link stops the stream and the user reconnects. If you
add auto-retry, back off (the OS stack rejects rapid reconnect attempts).

#### 1.8.1 Connection sequence

Two user actions, two flows. Both start with the same permission gate, which on Android is
**asynchronous** — treat "permission not yet granted" as *defer*, not *fail*, or the first tap of every
button after a fresh install does nothing.

```
ensure_permission(action):                       # returns immediately if already granted
    status = check_bluetooth_permission()
    if status == GRANTED:   return true
    if status == DENIED:    show_error("Bluetooth permission denied"); return false
    request_bluetooth_permission(callback: granted ->
        if granted: action()                     # re-run the whole action, do not resume mid-way
        else:       show_error("Bluetooth permission denied"))
    return false                                 # caller stops here; the callback restarts it

# ── Flow A: "Scan Devices" ─────────────────────────────────────────────
on_scan_clicked():
    if not ensure_permission(on_scan_clicked): return
    clear device list; clear error
    device_inquiry.start()                       # results stream in one by one
on_device_found(info):
    if info.name empty or address already listed: return
    append {name, address} → UI list updates live (do not wait for "finished")

# ── Flow B: "Connect" ──────────────────────────────────────────────────
on_connect_clicked():
    if source_type != BLUETOOTH: ...             # other sources, see §1.2/§1.5
    if config.address empty: show_error("Select a Bluetooth device first"); return
    if not ensure_permission(on_connect_clicked): return
    stop_device_inquiry()                        # an active inquiry starves RFCOMM (esp. Android)
    open_socket()                                # see below; report state via the UI status block

open_socket():                                   # ── desktop (own discovery, §1.8 rule 2)
    socket = new_rfcomm_socket()
    agent  = new_service_discovery_agent()
    agent.remote_address = config.address        # iOS: no address → match device UUID in the callback
    agent.uuid_filter    = SPP_UUID
    best = null
    agent.on_service_found(rec):
        if rec.rfcomm_channel <= 0: return
        if best == null or rec.rfcomm_channel < best.rfcomm_channel: best = rec
    agent.on_finished():
        if best == null: error("no serial port service found on <name>"); return
        agent.disconnect_signals(); agent.stop()  # nothing may arrive after this point
        socket.connect_to_service(best)           # exactly one connect call, ever
    agent.start(FULL_DISCOVERY)

open_socket():                                   # ── Android (no discovery needed)
    socket = new_rfcomm_socket()
    socket.connect_to_service(config.address, SPP_UUID)

on_socket_connected():   status = CONNECTED; start byte-rate timer
on_socket_data(bytes):   total += len(bytes); forward to §1.1 + §1.6 tap + §1.7 logger
on_socket_error(err):    if err == OPERATION_ERROR and socket.state != UNCONNECTED: ignore   # stray 2nd connect
                         else: status = ERROR; show_error(err)
on_disconnect_clicked(): stop inquiry; socket.abort(); destroy socket + agent; status = IDLE
```

Timings measured on Windows with a paired receiver: service discovery ≈ 0.9 s, RFCOMM connect ≈ 0.4 s,
first RTCM within ~1 s of connect, first 1005/1006 base fix within ~3 s (base messages are ~1 Hz or
slower). Budget ~2 s from tap to "Connected"; anything beyond ~10 s means the receiver is off or out of
range — show that instead of spinning forever.

**API mapping** for the four primitives, if you are porting off Qt:

| Primitive | Qt 6 | Android (Java) | Windows (WinRT) |
|-----------|------|----------------|-----------------|
| Device inquiry | `QBluetoothDeviceDiscoveryAgent` | `BluetoothAdapter.startDiscovery()` + `ACTION_FOUND` receiver, or `getBondedDevices()` for paired-only | `DeviceInformation.FindAllAsync(BluetoothDevice.GetDeviceSelector())` |
| Service discovery | `QBluetoothServiceDiscoveryAgent` (`setRemoteAddress` + `setUuidFilter`) | not needed | `BluetoothDevice.GetRfcommServicesForIdAsync(RfcommServiceId.SerialPort)` |
| Open stream | `QBluetoothSocket::connectToService(serviceInfo)` | `BluetoothDevice.createRfcommSocketToServiceRecord(SPP_UUID).connect()` | `StreamSocket.ConnectAsync(service.ConnectionHostName, service.ConnectionServiceName)` |
| Read | `readyRead` → `readAll()` | `socket.getInputStream().read(buf)` on a thread | `DataReader.LoadAsync` loop |

---

## 2. Configuration fields

| Field | Type | Notes |
|-------|------|-------|
| source type | enum | None / Serial / **NTRIP / TCP / UDP / Bluetooth** (0…5) |
| ntrip host / port | string / uint16 | caster address (default port 2101) |
| ntrip mountpoint | string | stream name |
| ntrip username / password | string | Basic auth; password field masked in UI |
| ntrip send GGA | bool | enable VRS position upload |
| tcp host / port | string / uint16 | raw RTCM3 TCP source |
| udp port | uint16 | local listen port |
| bluetooth device name | string | display name of the paired device (§1.8) |
| bluetooth device address | string | `AA:BB:CC:DD:EE:FF` — what the socket connects to (device UUID on iOS) |
| log to file | bool | toggle (can be toggled live while streaming) |
| rtcm log path | string | empty = auto-named file in the user save folder (§1.7) |

Status exposed to UI (fact names as implemented): `connected`, `sourceType`, `mountpoint` (stream label
— reused for the Bluetooth device label), `bytesPerSecond` (validated RTCM only, not raw socket
bytes), `rtcmValid` / `discardedBytes` (§1.6 validation gate), `baseValid` / `baseLatitude` /
`baseLongitude` / `baseAltitude` / `baseStationId`, `logFileName`, `logBytes`.

---

## 3. QGroundControl implementation map

New module: **`src/RTK/`** (compiled unconditionally; links Qt Network + Positioning, plus Qt Bluetooth
when `QGC_ENABLE_BLUETOOTH` is on — that flag is defined on the project target by `src/Comms/CMakeLists.txt`).

| File | Role |
|------|------|
| `RTCMStreamManager.{h,cc}` | app singleton; owns the active source (+ its worker thread for the network transports), `RTCMMavlink`, base parser, file logger and GGA timer; `Q_INVOKABLE startStream()/stopStream()/fetchMountpoints()/scanBluetoothDevices()/stopBluetoothScan()/selectBluetoothDevice()` |
| `RTCMNetworkSource.h` | abstract worker interface: `signals rtcmData / connectedChanged / errorOccurred / bytesReceived` |
| `NtripClient.{h,cc}` | §1.2 handshake + §1.3 GGA; `useNtripHandshake=false` makes it the §1.5 raw-TCP source |
| `UdpRtcmReceiver.{h,cc}` | §1.5 UDP source |
| `BluetoothRtcmSource.{h,cc}` | §1.8 source + `BluetoothRtcmScanner` device picker (built only with `QGC_ENABLE_BLUETOOTH`) |
| `NtripSourceTable.{h,cc}` | §1.4 source-table fetch/parse |
| `RtcmStreamParser.{h,cc}` | §1.6 RTCM3 framing/CRC-24Q validation gate + 1005/1006 decode (ECEF→WGS84) |
| `RTCMFileLogger.{h,cc}` | §1.7 raw RTCM3 logging (can run alongside the RINEX writers) |
| `RtcmObsDecoder.{h,cc}` | §1.7b MSM4/5/6/7 -> observation epochs + station metadata |
| `RinexObsWriter.{h,cc}` | §1.7b streaming RINEX 3.04 observation writer |
| `RtcmNavDecoder.{h,cc}` | §1.7c 1019/1020/1042/1044/1045/1046 -> broadcast ephemeris |
| `RinexNavWriter.{h,cc}` | §1.7c streaming RINEX 3.04 navigation writer (de-duplicating) |
| `RtcmBitReader.h` | shared MSB-first bit reader, incl. the GLONASS sign-magnitude helper |
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

Threading model (mirror `GPSRtk`): each **network** source runs on its own worker `QThread`; its
`rtcmData` signal is delivered to `RTCMMavlink` (on the main thread) via a **queued connection** (a value
copy of the `QByteArray` crosses the thread boundary). The GGA timer lives on the main thread and posts
the sentence to the worker. The **Bluetooth** source is the exception: it stays on the main thread for
the COM reason in §1.8, so `RTCMStreamManager` creates no `QThread` for it (`active()` therefore tests
`_worker`, not `_workerThread`) and starts it with a queued `invokeMethod` so it cannot report back into
the middle of `startStream()`.

Android permissions need no manifest edit: `androiddeployqt` merges `BLUETOOTH_SCAN` /
`BLUETOOTH_CONNECT` / `ACCESS_FINE_LOCATION` from Qt Bluetooth's `-android-dependencies.xml`. The
runtime request is `RTCMStreamManager::_withBluetoothPermission()`, which re-runs the pending action
(scan or connect) from the grant callback.

---

## 4. UI — what to display, and what is already built

§4.0 is the **status panel as shipped** (validated live); §4.1–4.6 are what a good panel should surface
on top of it, grouped by purpose, with a **Source** column saying where each value comes from so it can
be reproduced on any platform (§ refers to Part 1). §4.7 documents the **Bluetooth controls** as shipped
and the rules behind them, and §4.8 is the hands-on "try it yourself" walkthrough.

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
sent) → file logging.

The same panel, validated on the **Bluetooth** source against a paired receiver (Windows, 2026-08-12):

| Field | Live example | Source |
|-------|--------------|--------|
| Source | `Bluetooth` | config source type |
| Stream | `GNSS-5804510 (81:8D:10:0B:1C:08)` | configured device name + address (§1.8) |
| Status | `Connected` | RFCOMM socket state — ~1.3 s from Connect to Connected, of which ~0.9 s is service discovery |
| Data Rate | `4913.17 B/s` | bytes/interval (§1.1 input) |
| Base Station / Altitude | `10.8418817, 106.7750761` / `22.86 m` | RTCM **1006**, station id 478 (§1.6) |

That receiver publishes two SPP records (`COM0` ch 1, `COM1` ch 2), which is what motivated the
lowest-channel rule in §1.8. The values below (§4.1–4.6) are the **recommended additions** on top of this
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

### 4.7 Bluetooth source UI (as implemented)

The Bluetooth source adds four things to the same settings page — a hint, the selected-device row, the
scan row and an error line; everything else (log toggle, Connect button, Status block) is shared with the
network sources. Layout as shipped:

```
┌ Network RTCM Source ─────────────────────────────────────────────────────┐
│ Correction Source                                        [ Bluetooth ▾ ] │   ← enum, 6 entries
│ The correction stream runs independently of the vehicle connection …     │   ← shared hint
│ Pair the RTK receiver / base radio with this device in the operating     │   ← Bluetooth-only hint
│ system first, then scan and select it here.                              │
│ Device                            GNSS-5804510 (81:8D:10:0B:1C:08)       │   ← or "None selected"
│ [ Scan Devices ]   [ GNSS-5804510 (81:8D:10:0B:1C:08)              ▾ ]   │   ← combo hidden until results
│ Log RTCM to file                                              (  ○)      │   ← shared, live-togglable
│ Connection                                          [   Connect    ]     │   ← shared, toggles label
└──────────────────────────────────────────────────────────────────────────┘
┌ Status ────────────────────────────── (visible only while streaming) ────┐
│ Source        Bluetooth        Data Rate    4913.17 B/s                  │
│ Stream        GNSS-5804510 …   Base Station 10.8418817, 106.7750761      │
│ Status        Connected        Base Altitude 22.86 m                     │
└──────────────────────────────────────────────────────────────────────────┘
```

Control behaviour — the rules that make it usable:

| Control | Rule |
|---------|------|
| Bluetooth rows | visible only when source type == Bluetooth; the shared rows stay visible for every non-serial source |
| `Device` row | shows `name (address)` from **settings**, not from the scan list — so the choice survives an app restart. Falls back to the bare address, then to "None selected" |
| `Scan Devices` | label becomes `Scanning…` and the button disables while an inquiry runs — and **only** then. Never gate it on an "adapter available" check: on Android that check cannot succeed until Bluetooth access is granted, and this button is what asks for it, so gating it deadlocks the flow (button greyed out + "No Bluetooth adapter available" forever on a fresh install) |
| adapter availability | treat it as **optimistic until proven otherwise**: assume present, probe the adapter only *after* the permission grant inside the scan/connect flow, and expose it as a notifying property, not a constant — a value read once at page load is read too early on Android. Desktop may probe at startup (no permission needed there) |
| results combo | hidden while the list is empty (a permanently empty dropdown reads as "broken"); appears and grows **during** the scan; selecting an entry writes name+address into settings immediately |
| error line | one warning-coloured line under the rows, showing **one string owned by the backend** (permission denied, no adapter, no SPP service, socket error, "select a device first"). Do not compose it in the view from other properties — that is how the page ends up asserting "No Bluetooth adapter available" before anything has actually been checked. **Cleared when a new scan/connect starts** so a stale error never sits under a working connection |
| `Connect` | shared with the other sources; label flips to `Disconnect` while a stream is active. Pressing it with no device selected sets the error line rather than silently doing nothing |
| Status block | appears only while streaming; `Status` reads `Connecting…` until the socket reports connected |

Backend surface the UI binds to (`QGroundControl.rtcmStreamManager`) — replicate these names or map them
to your own view-model:

| Member | Type | Purpose |
|--------|------|---------|
| `scanBluetoothDevices()` | invokable | permission gate + start inquiry (§1.8.1 flow A) |
| `stopBluetoothScan()` | invokable | cancel inquiry (also called automatically before connecting) |
| `selectBluetoothDevice(index)` | invokable | write the chosen entry's name+address into settings |
| `bluetoothDevices` | `string[]` | live list of `name (address)` labels, notified per discovery |
| `scanningBluetooth` | `bool` | drives the button label/enabled state |
| `bluetoothAvailable` | `bool` (**notifying**) | is there a local adapter — optimistic until the post-permission probe corrects it |
| `bluetoothError` | `string` | last error, empty when fine |
| `active`, `startStream()`, `stopStream()` | — | shared with every source |
| `rtcmFactGroup` | fact group | shared status values (connected, sourceType, mountpoint/stream, bytesPerSecond, base*, log*) |

Settings keys are `rtcmSourceType = 5`, `bluetoothDeviceName`, `bluetoothDeviceAddress` (Part 2).

### 4.8 Trying it as a developer

1. Pair the receiver in the OS Bluetooth panel first (Windows Settings → Bluetooth, or Android
   Settings → Bluetooth). The GCS never pairs.
2. Turn the module's logging on — this is the fastest way to see where a connect stalls. Two ways:
   - **In-app, no rebuild or shell needed** (the only practical option on a tablet): Application
     Settings → **Console** → *Logging categories* → tick the `qgc.rtk.*` categories. The Console page
     also shows the messages, so the whole loop stays on the device.
   - **Environment**, for a desktop dev loop: `QT_LOGGING_RULES="qgc.rtk.*=true"`. On Windows a GUI
     build writes nothing to a console unless you also set `QT_FORCE_STDERR_LOGGING=1` and
     `QT_ASSUME_STDERR_HAS_CONSOLE=1` and redirect stderr. On Android, `adb logcat | grep qgc.rtk`.

   Add `qt.bluetooth*=true` when the failure looks like it is inside the Qt stack rather than the app.
3. Application Settings → **RTK / NTRIP** → Correction Source = **Bluetooth** → *Scan Devices* → pick the
   receiver → **Connect**.
4. A healthy connect looks exactly like this (real log, Windows):

```
Started RTCM stream: "Bluetooth" "GNSS-5804510 (81:8D:10:0B:1C:08)"
Connecting to "GNSS-5804510" "81:8D:10:0B:1C:08"
Found service "COM0" channel 1
Found service "COM1" channel 2
Connecting to service "COM0" channel 1        ← one connect, lowest channel (§1.8 rule 1)
Connected to "GNSS-5804510"
Source connected: true
Base station 1006 id 478 lat 10.8419 lon 106.775 alt 22.8649
```

5. No vehicle is needed to test the source: `Data Rate` > 0 proves bytes are flowing and the base fields
   prove they are valid RTCM. Tick *Log RTCM to file* and open the `.rtcm3` with any RTCM parser to check
   the message-type mix (§4.3). Connect a vehicle only for the last hop (§4.6, fix type → RTK Fixed).
6. Symptom table:

| Symptom | Cause |
|---------|-------|
| `No Bluetooth adapter available` on Android with Bluetooth clearly on | the adapter was probed before `BLUETOOTH_CONNECT` was granted (§1.8 Android note) — probe after the grant, not at page load |
| Scan finds nothing | device off / not discoverable; on Android also the `neverForLocation` scan-flag caveat and, on API ≤ 30, location services switched off |
| `no serial port service found` | the device is paired but exposes no SPP record — it is a BLE-only device, or pairing was lost |
| Connect hangs, no error | receiver out of range / already connected to another host (RFCOMM is single-client) |
| Connected but `Data Rate` stays 0 | connected to the wrong serial port of a multi-port receiver, or the receiver is not configured to output RTCM3 on it |
| `Data Rate` fine, base fields empty | stream has observations but no 1005/1006 — normal for some setups, but then the map marker cannot be drawn |

---

## 5. Porting checklist (to another GCS / app)

1. **Transport:** a raw TCP client (NTRIP + raw-TCP) and a UDP listener. Not a high-level HTTP client.
   Add an RFCOMM/SPP client (§1.8) if you want the cable-free local-base case.
2. **NTRIP handshake + response parser** per §1.2 (handle ICY/non-HTTP 200, bare `\n`, SOURCETABLE).
3. **RTCM → `GPS_RTCM_DATA`** per §1.1 — the only firmware-facing piece; get the flags bitfield and the
   180-byte fragmentation right.
4. **GGA upload** per §1.3 if you need VRS, including the last-position cache on link loss.
5. Optional: source-table fetch (§1.4), base-position decode (§1.6), file logging (§1.7),
   Bluetooth device picker + deterministic SPP record choice (§1.8).
6. Everything except §1.1 is ordinary transport code; §1.1 is the piece that must match the MAVLink spec
   exactly.
