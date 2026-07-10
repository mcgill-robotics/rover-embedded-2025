# CAN FD Data Logger and Dashboard — Architecture

## 1. Overview

The data logger and dashboard system captures, stores, and visualises every CAN FD frame on the ESC bus in real time. It consists of three layers that run on the PC (master) side alongside the CANable adapter:

```
 B-G431B-ESC1 boards                          PC (Master)
 ┌──────────────┐                     ┌─────────────────────────────────┐
 │  ESC 1       │                     │                                 │
 │  telemetry   │───┐                 │   can_logger.py                 │
 │  + response  │   │  CAN FD bus     │     │                           │
 └──────────────┘   │  500 kbps /     │     │ python-can                │
 ┌──────────────┐   ├──2 Mbps BRS ──▶ │     ▼                           │
 │  ESC 2       │   │   via CANable   │   datalogger.py                 │
 │  telemetry   │───┘                 │     │  batched writes           │
 │  + response  │                     │     ▼                           │
 └──────────────┘                     │   can_log.db  (SQLite, WAL)     │
                                      │     ▲  read-only                │
                                      │     │                           │
                                      │   can_dashboard.py  (Flask)     │
                                      │     │  SSE stream               │
                                      │     ▼                           │
                                      │   Browser  localhost:5000       │
                                      └─────────────────────────────────┘
```

The data flow is strictly one-directional for logging: CAN frames arrive from the bus, get decoded and written to SQLite, and the dashboard reads from the same file to push updates to the browser. The logger and dashboard run as two separate processes on the same machine and share the database file safely through SQLite's WAL (Write-Ahead Logging) mode.

---

## 2. File structure

```
esc_can/
    __init__.py
    bus.py                 ← CANable bus open/close helpers
    protocol.py            ← CAN ID encoding/decoding, payload pack/unpack
    datalogger.py          ← SQLite logger library (CANDataLogger class)
scripts/
    can_logger.py          ← CLI tool: captures bus traffic into the database
    can_dashboard.py       ← Flask web server: live dashboard UI
    multi_field_extractor.py
    test_canfd.py
```

The `datalogger.py` module lives inside the `esc_can` package so it can be imported by any script that needs logging or querying. It depends only on `protocol.py` from the same package and Python's built-in `sqlite3` module. The two scripts in `scripts/` are standalone entry points — one writes to the database, the other reads from it.

---

## 3. How the CAN bus feeds the system

### 3.1 What the ESC broadcasts

Each B-G431B-ESC1 runs a telemetry scheduler (`can_telemetry.c`) that autonomously transmits sensor data at fixed intervals without being polled. The default telemetry table produces approximately 38 frames per second per ESC:

| Signal         | ReadSpec value | Period   | Frames/sec |
|----------------|---------------|----------|------------|
| Motor Speed    | CALIBRATION (0)| 100 ms  | 10         |
| Position       | POSITION (1)   | 100 ms  | 10         |
| Phase Currents | CURRENT (3)    | 100 ms  | 10         |
| Bus Voltage    | VOLTAGE (2)    | 200 ms  | 5          |
| Motor State    | CURRENT_STATE (4)| 500 ms | 2          |
| Temperature    | TEMPERATURE (5)| 1000 ms | 1          |

On top of scheduled telemetry, the ESC also responds to on-demand read requests from the master. These request-response frames use the same CAN ID format and are structurally identical on the bus — the only difference is that the master initiated them.

All frames use the 11-bit CAN ID encoding defined in `can_common.h` and mirrored in `protocol.py`:

```
Bit:  [10]    [9]     [8]       [7]        [6:4]          [3:0]
      Sender  Action  Config    MotorType  Specification  DeviceID
```

### 3.2 How the logger receives frames

`can_logger.py` opens the CANable adapter via `python-can`'s SLCAN interface at the matching bus parameters (500 kbps nominal, 2 Mbps data, CAN FD with BRS). It then enters a receive loop:

```python
while not stop:
    msg = bus.recv(timeout=0.1)       # blocks up to 100 ms
    if msg is None:
        continue
    logger.log_frame(msg, is_rx=True) # hand to datalogger
```

Every `can.Message` received from the adapter is passed directly to the `CANDataLogger` instance. The logger decodes the CAN ID on the spot, extracts the protocol fields (sender, action, motor config, motor type, spec, device ID), and for single-motor float frames also decodes the 4-byte little-endian payload into a Python float. All of this is buffered in memory until a flush occurs.

The logger does not filter any frames — it captures everything on the bus, including master-originated commands (Sender = MASTER), ESC responses (Sender = SLAVE), and multi-motor broadcasts. This ensures the database is a complete record of all bus activity.

---

## 4. Database design

### 4.1 Why SQLite

SQLite was chosen for several reasons specific to this use case. The system has a single writer (the logger process) and one or more readers (the dashboard, analysis scripts), which matches SQLite's concurrency model perfectly. There is no need to install or run a separate database server — the entire database is a single file on disk. With WAL mode and batched inserts, SQLite sustains over 30,000 inserts per second on typical hardware, far exceeding the ~38 frames/sec per ESC that the bus produces. The database file is also portable — it can be copied to another machine, opened in any SQLite browser, or queried from a Jupyter notebook without any infrastructure.

### 4.2 Performance settings

When `CANDataLogger` opens the database, it configures three SQLite pragmas:

```python
self._conn.execute("PRAGMA journal_mode=WAL")
self._conn.execute("PRAGMA synchronous=NORMAL")
self._conn.execute("PRAGMA cache_size=-8000")   # 8 MB page cache
```

**WAL mode** is the critical setting. In the default rollback journal mode, SQLite locks the entire database file during writes, which would block the dashboard from reading while the logger is inserting. WAL mode eliminates this contention — writers append to a separate write-ahead log file, and readers see a consistent snapshot of the database at the moment they start their transaction. This is what allows `can_logger.py` and `can_dashboard.py` to operate on the same `.db` file simultaneously without any coordination.

**Synchronous NORMAL** reduces the number of `fsync` calls. In WAL mode, NORMAL is safe against data loss from application crashes (only an OS crash or power failure could lose the last few uncommitted frames, which is acceptable for telemetry logging). The alternative, FULL, would force a `fsync` on every commit and significantly reduce write throughput.

**Cache size of 8 MB** keeps frequently accessed pages in memory, reducing disk reads for the dashboard's repeated queries.

### 4.3 Schema

The database contains three tables:

**`sessions`** — one row per logging session (each time `can_logger.py` is started). Stores a timestamp, a human-readable description, and the bus bitrate parameters. This allows multiple capture sessions to coexist in the same database file.

```sql
CREATE TABLE sessions (
    session_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at   TEXT    NOT NULL,
    description  TEXT,
    bus_nominal  INTEGER,     -- 500000
    bus_data     INTEGER      -- 2000000
);
```

**`raw_frames`** — one row per CAN frame. This is the main table and grows linearly with capture time. Each row stores both the raw frame data and the decoded protocol fields:

```sql
CREATE TABLE raw_frames (
    frame_id      INTEGER PRIMARY KEY,
    session_id    INTEGER NOT NULL,
    timestamp_s   REAL    NOT NULL,   -- seconds since session start

    -- Raw CAN frame
    arb_id        INTEGER NOT NULL,   -- 11-bit CAN ID
    dlc           INTEGER NOT NULL,
    data_bytes    BLOB    NOT NULL,   -- raw payload (up to 64 bytes)
    is_fd         INTEGER NOT NULL,
    brs           INTEGER NOT NULL,
    is_rx         INTEGER NOT NULL,   -- 1 = received, 0 = transmitted

    -- Decoded protocol fields (denormalised)
    sender        TEXT,       -- 'MASTER' or 'SLAVE'
    action        TEXT,       -- 'RUN' or 'READ'
    motor_config  TEXT,       -- 'SINGLE' or 'MULTIPLE'
    motor_type    TEXT,       -- 'DRIVE' or 'STEERING'
    spec_value    INTEGER,    -- raw 3-bit spec
    spec_name     TEXT,       -- e.g. 'POSITION', 'TEMPERATURE'
    device_id     INTEGER,    -- 0–15

    -- Decoded payload
    decoded_float REAL,       -- float from bytes 0–3 (NULL if not applicable)

    FOREIGN KEY (session_id) REFERENCES sessions(session_id)
);
```

The protocol fields are intentionally denormalised (stored as text strings like `'SLAVE'` and `'POSITION'` rather than integer foreign keys to lookup tables). This trades a small amount of storage space for significantly simpler and faster queries — the most common query pattern is "give me all POSITION readings from ESC 3 between t=5s and t=10s", which becomes a straightforward `WHERE` clause without any joins.

**`schema_version`** — a single-row table tracking the schema version number for future migrations.

### 4.4 Indexes

Three indexes are created to support the most common query patterns:

```sql
-- "All POSITION values from ESC 3" — the primary dashboard query
CREATE INDEX idx_signal_device ON raw_frames (spec_name, device_id, timestamp_s);

-- "Everything in session 5" — session replay
CREATE INDEX idx_session_time  ON raw_frames (session_id, timestamp_s);

-- "All frames with arb_id 0x721" — raw protocol debugging
CREATE INDEX idx_arb_id        ON raw_frames (arb_id, timestamp_s);
```

The `idx_signal_device` index is the most important. It allows the dashboard's `/api/stream` and `/api/history` queries to efficiently locate the latest value for a specific signal on a specific ESC without scanning the entire table.

### 4.5 Write buffering and flush strategy

Inserting each frame individually with an immediate `COMMIT` would be extremely slow — SQLite can only do about 50–100 individual transactional inserts per second because each `COMMIT` requires an `fsync` to guarantee durability. The logger avoids this by buffering frames in a Python list and writing them in batches:

```
CAN frame arrives
    │
    ▼
log_frame() decodes CAN ID, appends tuple to in-memory buffer
    │
    ├── buffer reaches batch_size (default 64)?  ──▶  flush()
    │
    └── background timer fires (every 250 ms)?   ──▶  flush()
```

`flush()` swaps out the buffer under a lock, then calls `executemany()` to insert all buffered rows in a single transaction followed by one `COMMIT`. This reduces the number of `fsync` calls from N-per-frame to roughly 4 per second (at the default 250 ms timer interval), which is the key to achieving high sustained throughput.

The buffer is protected by a `threading.Lock` because the auto-flush runs on a background daemon thread. The lock is only held briefly — just long enough to swap the list reference — so it does not block the main receive loop.

### 4.6 CAN ID decoding at insert time

When `log_frame()` receives a `python-can` Message, it immediately calls `decode_can_id()` from `protocol.py` to extract the 11-bit ID into its six component fields. It also calls `_resolve_spec_name()` to convert the 3-bit spec integer into a human-readable string (e.g. `ReadSpec.TEMPERATURE` becomes `"TEMPERATURE"`).

For single-motor frames (MotorConfig = SINGLE) with at least 4 bytes of payload, the logger also calls `unpack_single_float()` to extract the IEEE-754 float from bytes 0–3. This decoded value is stored in the `decoded_float` column. Multi-motor frames, run commands without float payloads (like STOP), and frames with less than 4 data bytes get `NULL` in this column.

This decode-at-insert strategy means queries never need to parse raw bytes — the decoded values are always ready to use.

---

## 5. Query API

The `CANDataLogger` class exposes four query methods that can be used either from the dashboard server or from standalone analysis scripts:

**`query_telemetry(device_id, signal, t_start, t_end)`** — returns a list of `(timestamp_s, decoded_float)` tuples for a specific signal from a specific ESC. This is the workhorse for time-series analysis:

```python
positions = logger.query_telemetry(device_id=1, signal="POSITION",
                                   t_start=0.0, t_end=10.0)
# → [(0.0, 45.2), (0.1, 46.1), (0.2, 47.3), ...]
```

**`query_frames(**filters)`** — flexible query returning full `sqlite3.Row` objects. Supports filtering by any combination of device_id, action, spec_name, sender, and time range. Returns dict-like rows accessible by column name.

**`get_latest(device_id, signal)`** — returns the single most recent `(timestamp_s, decoded_float)` for a signal. Uses `ORDER BY timestamp_s DESC LIMIT 1` so it hits the index and returns instantly regardless of table size.

**`get_session_summary()`** — returns aggregate statistics: total frames, duration, average frame rate, number of unique ESCs, and number of unique signals.

---

## 6. Dashboard web UI

### 6.1 Architecture

The dashboard is a single Python file (`can_dashboard.py`) that uses Flask to serve both the HTML/CSS/JS frontend and the JSON API endpoints. There is no build step, no npm, no bundler — the entire UI is embedded as a string in the Python file and served directly. The only dependency beyond the Python standard library is Flask (`pip install flask`).

The dashboard connects to the SQLite database in **read-only mode** (`?mode=ro` URI parameter), so it cannot accidentally corrupt the logger's data. It opens a fresh connection for each API request, which is lightweight with SQLite and avoids any issues with connection reuse across threads.

### 6.2 API endpoints

The Flask server exposes four endpoints:

**`GET /`** — serves the dashboard HTML page. This is a self-contained document with inline CSS and JavaScript — no external assets need to be loaded except two Google Fonts.

**`GET /api/latest`** — returns a JSON object with the latest decoded value for every `(device_id, signal)` combination in the database. The response is keyed by device ID and signal name:

```json
{
  "1": {
    "POSITION":    {"value": 47.3, "timestamp_s": 12.45},
    "TEMPERATURE": {"value": 42.1, "timestamp_s": 12.00},
    "VOLTAGE":     {"value": 11.9, "timestamp_s": 12.20}
  }
}
```

The query uses `GROUP BY device_id, spec_name` with `MAX(frame_id)` to efficiently find the latest frame for each combination without scanning the full table.

**`GET /api/history/<device_id>/<signal>`** — returns the last 200 data points for a specific signal, ordered chronologically. Used to populate the sparkline charts when the page first loads.

**`GET /api/stats`** — returns aggregate bus statistics (total frames, duration, frame rate, ESC count).

**`GET /api/stream`** — the Server-Sent Events (SSE) endpoint. This is the mechanism that makes the dashboard update live.

### 6.3 Server-Sent Events (SSE)

SSE is a simple HTTP-based protocol for server-to-client streaming. The browser opens a long-lived `GET` request to `/api/stream`, and the server sends data events as they become available. Unlike WebSockets, SSE is unidirectional (server to client only), uses standard HTTP, works through proxies, and reconnects automatically if the connection drops.

The SSE generator function runs in a loop:

```
every 200 ms:
    1. Query raw_frames for any rows with frame_id > last_seen_frame_id
       grouped by (device_id, spec_name)
    2. If new data exists, format as JSON and yield as an SSE event
    3. Update last_seen_frame_id to the highest frame_id seen
    4. Sleep 200 ms
```

The `frame_id > last_seen_frame_id` filter is what makes this efficient — it only queries for frames that have been inserted since the last poll, rather than re-scanning the entire table. The frame_id column is the auto-incrementing primary key, so this comparison is a simple integer range check against the index.

On the browser side, the JavaScript creates an `EventSource` and processes incoming events:

```javascript
const es = new EventSource('/api/stream');
es.onmessage = (ev) => {
    const data = JSON.parse(ev.data);
    for (const [devId, signals] of Object.entries(data)) {
        for (const [sig, info] of Object.entries(signals)) {
            updateCard(devId, sig, info.value, info.t);
        }
    }
};
```

If the SSE connection drops (server restart, network issue), the browser automatically reconnects after 2 seconds. The status dot in the top bar turns red during disconnection and green when live.

### 6.4 Frontend rendering

The dashboard creates one section per ESC, each containing a grid of signal cards. Cards are created dynamically when the first data for a new device ID arrives — if a new ESC appears on the bus, its box appears automatically without any configuration.

Each signal card displays the current value with units (e.g. "47.32 deg"), a timestamp showing when the value was last updated, and a sparkline canvas showing the last 60 data points. The sparkline is drawn on a `<canvas>` element using direct Canvas 2D API calls — no charting library is needed for this simple line-plus-fill visualisation.

When a value updates, the card plays a brief flash animation (background color pulse) to draw attention to the change. Each signal type has its own accent color (blue for speed, cyan for position, orange for voltage, red for temperature, etc.) applied via CSS custom properties.

### 6.5 Running the dashboard

The logger and dashboard run as two separate processes. Both must point at the same database file:

```bash
# Terminal 1 — capture from bus
python scripts/can_logger.py --port COM4 --db can_log.db

# Terminal 2 — serve dashboard
python scripts/can_dashboard.py --db can_log.db
# → open http://localhost:5000
```

The dashboard can also be started before the logger — it will show a "Waiting for CAN data" empty state and automatically populate once frames start arriving. It can also be pointed at a previously captured database file to review historical data (although the SSE stream will show no new updates in that case).

---

## 7. Data flow summary

Putting the entire pipeline together, here is the path a single CAN frame takes from the ESC to the browser:

```
1.  ESC firmware: Telemetry_Tick_1ms() fires, reads from ESC_SensorData cache,
    packs float into bytes 0–3, builds 11-bit CAN ID via CAN_BuildID(),
    queues frame with HAL_FDCAN_AddMessageToTxFifoQ()

2.  CAN FD bus: frame transmits at 500 kbps arbitration / 2 Mbps data phase
    with bit rate switching

3.  CANable adapter: receives frame, forwards to PC via USB-serial (SLCAN)

4.  python-can: bus.recv() returns a can.Message with arbitration_id, data,
    dlc, is_fd, bitrate_switch

5.  can_logger.py: calls logger.log_frame(msg)

6.  datalogger.py log_frame():
    a. decode_can_id() extracts sender, action, config, type, spec, device_id
    b. _resolve_spec_name() converts spec integer to string ("TEMPERATURE")
    c. unpack_single_float() decodes bytes 0–3 into a Python float
    d. Appends all fields as a tuple to the in-memory buffer

7.  datalogger.py flush() (triggered by batch_size=64 or timer at 250 ms):
    a. Swaps buffer under lock
    b. executemany() inserts all rows in one transaction
    c. COMMIT writes to WAL file on disk

8.  can_dashboard.py /api/stream SSE generator (polls every 200 ms):
    a. Queries raw_frames WHERE frame_id > last_seen, grouped by
       (device_id, spec_name)
    b. Yields JSON event to the HTTP response stream

9.  Browser EventSource receives the SSE event:
    a. Parses JSON
    b. Updates the card value, timestamp, and unit text
    c. Appends to sparkline history array, redraws canvas
    d. Triggers card flash animation
```

The total latency from ESC transmission to browser display is typically 250–500 ms, dominated by the logger's flush interval (250 ms) and the dashboard's SSE poll interval (200 ms). These can both be reduced for lower latency at the cost of slightly higher CPU usage.

---

## 8. Storage estimates

At 38 frames/sec per ESC, each row in `raw_frames` occupies roughly 200–250 bytes on disk (including index overhead). Approximate storage growth:

| ESC count | Frames/sec | Per minute | Per hour  | Per 8 hours |
|-----------|-----------|------------|-----------|-------------|
| 1         | 38        | ~0.5 MB    | ~30 MB    | ~240 MB     |
| 4         | 152       | ~2 MB      | ~120 MB   | ~960 MB     |
| 8         | 304       | ~4 MB      | ~240 MB   | ~1.9 GB     |

SQLite handles databases up to 281 TB, so storage capacity is not a practical concern. For very long captures (days), the database file can be vacuumed or old sessions can be deleted to reclaim space.

---

## 9. Extending the system

**Adding new telemetry signals** — when a new `ReadSpec` value is added to the firmware telemetry table, the logger captures it automatically because it decodes all frames regardless of spec value. The `spec_name` column will contain the new enum name from `protocol.py`'s `ReadSpec` class. To display it in the dashboard, add an entry to the `SIGNAL_META` dictionary in `can_dashboard.py` with the label, unit, icon, and decimal precision.

**Multi-motor frame decoding** — the current logger stores the raw `data_bytes` blob for multi-motor frames but only populates `decoded_float` for single-motor float payloads. To decode multi-motor payloads (e.g. `pack_multi_speeds`), a post-processing query or a new column could be added.

**Historical analysis** — the database file can be opened in any SQLite tool (DB Browser for SQLite, DBeaver, Python's `sqlite3` module, or a Jupyter notebook with pandas). Example query to export a time series as a pandas DataFrame:

```python
import sqlite3, pandas as pd

conn = sqlite3.connect("can_log.db")
df = pd.read_sql_query("""
    SELECT timestamp_s, decoded_float AS position_deg
    FROM raw_frames
    WHERE device_id = 1 AND spec_name = 'POSITION'
      AND decoded_float IS NOT NULL
    ORDER BY timestamp_s
""", conn)
```
