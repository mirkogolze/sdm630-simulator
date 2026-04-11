# RS485 Echo-Problem: Analyse und Lösungsweg

## Kontext

Hardwareaufbau: Raspberry Pi mit Waveshare 4-Port USB-RS485-Konverter (CH348L-Chip),
später ersetzt durch Waveshare Industrial USB-RS485 (FT232RNL + SP485EEN).

- `ttyACM0` / `ttyUSB0` — SDM630-Simulator (Modbus-Server, unit=2) + THOR K11 Wallbox (Master)
- `ttyACM0` — Growatt SPH10000 Wechselrichter (Modbus-Client, unit=1, separater Bus)

## Problem: "Frame check failed" Log-Flut

### Symptom

Nach jedem gesendeten Modbus-Frame erscheinen Tausende von Einträgen:

```text
Frame check failed, possible garbage after frame, testing..
```

Der interne `recv_buffer` wächst mit jeder Iteration und enthält immer dasselbe Muster:

```text
0x2 0x50 0x56 0x4 0x64   ← Fragment der eigenen FC04-Antwort (unit=2)
0x2 0x10 0x0 0x1e 0xc5   ← Fragment der eigenen FC16-Antwort (unit=2)
```

### Ursache: Hardware-TX-Echo

Der Waveshare USB-RS485-Adapter loopt alle gesendeten TX-Bytes auf RX zurück (fehlende
oder zu langsame DE/RE-Pin-Steuerung). Da der Simulator als **Server** dauerhaft lauscht,
landen diese Echo-Bytes im pymodbus-Frame-Buffer und werden als eingehende Frames geparst.
Die CRC-Prüfung schlägt fehl → "Frame check failed".

### Warum ttyACM0 (Client) davon nicht betroffen ist

Der pymodbus-**Client** ruft vor jeder Antwort intern `reset_input_buffer()` auf. Echo-Bytes,
die im falschen Zeitfenster ankommen, werden dadurch verworfen, bevor der Parser sie sieht.
Der **Server** führt diesen Reset nie durch — er liest alles ungefiltert in den Buffer.

## Analyse des pymodbus-Quellcodes

Datei: `pymodbus/pymodbus/transport/transport.py`

### `handle_local_echo=True` — Implementierung

```python
def datagram_received(self, data: bytes, addr: tuple | None) -> None:
    if self.comm_params.handle_local_echo and self.sent_buffer:
        if data.startswith(self.sent_buffer):
            # Echo vollständig am Anfang → entfernen
            data = data[len(self.sent_buffer):]
            self.sent_buffer = b""
        elif self.sent_buffer.startswith(data):
            # Echo nur partiell angekommen → Rest abwarten, return
            self.sent_buffer = self.sent_buffer[len(data):]
            return
        else:
            # Echo kam nicht oder in falscher Reihenfolge → ignorieren
            self.sent_buffer = b""
```

**Einschränkung:** Funktioniert nur wenn der Echo als **vollständiger Block** am Anfang des
empfangenen Datums ankommt. Bei 9600 Baud und USB-Latenz kann der Echo fragmentiert eintreffen
— dann greift der `startswith`-Zweig nicht und der Echo landet trotzdem im Buffer.

### Automatischer Buffer-Reset bei 1024 Bytes

```python
if len(self.recv_buffer) > 1024:
    self.recv_buffer = b''
```

Der Buffer wächst maximal auf 1024 Bytes, wird dann automatisch geleert. Das verhindert
unbegrenzte Akkumulation, führt aber zum periodischen Verwerfen legitimer Frame-Daten.

### `sent_buffer` wird beim Senden befüllt

```python
# in callback_new_connection / write():
if self.comm_params.handle_local_echo:
    self.sent_buffer += data
```

Der `sent_buffer` wird nur befüllt wenn `handle_local_echo=True` gesetzt ist.

## Korrektur: `data_received` leitet zu `datagram_received` weiter

Verifikation im Quellcode (`pymodbus/transport/transport.py`):

```python
def data_received(self, data: bytes) -> None:
    self.datagram_received(data, None)          # ← Serial NUTZT denselben Pfad!

def datagram_received(self, data: bytes, addr: tuple | None) -> None:
    if self.comm_params.handle_local_echo and self.sent_buffer:
        if data.startswith(self.sent_buffer):   # ← startswith-only-Match
            ...
```

`handle_local_echo=True` **läuft also für Serial** — die ursprüngliche Aussage "nur UDP" war
falsch. Das Problem ist dennoch reell: Der `startswith`-Match schlägt bei fragmentiertem Echo
fehl, und `sent_buffer` wird dann auf `b""` zurückgesetzt → Echo landet im Buffer.

## Vergleich der Optionen

| Option | Aufwand | Wirkung | Status |
| --- | --- | --- | --- |
| `handle_local_echo=True` | 0 | Partiell (nur bei Block-Echo) | ❌ unzureichend |
| `delay_before_rx=0.015` | minimal | Mittel (CH348L-abhängig) | ❌ unzureichend |
| PTY Echo-Filter Proxy (sent_buffer) | mittel | – | ❌ asyncio-Race, nicht deployed |
| PTY Echo-Filter Proxy (reader-pause) | hoch | – | ❌ gescheitert, Ursache unklar |
| Log-Filter `_SimulatorOnlyFilter` | gering | Analyse-Hilfe | ❌ Pattern unvollständig |
| ModbusProtocol Monkey-Patch (datagram_received) | gering | – | ❌ `sent_buffer` im async-Pfad leer |
| Hardware-Austausch CH348L → FT232RNL | €15 | Kein Echo erwartet | ❌ RC-Delay identisch |
| asyncio Transport-Level Filter (`_RS485EchoFilter`) | mittel | – | ❌ `connection_made` timing-Race, Filter nie aktiv |
| SerialTransport Klassen-Patch (`_st_write`/`_st_read_ready`) | mittel | Vollständig (byte-count, pyserial-Ebene) | ❌ asyncio bound-method capture, `_st_read_ready` nie aufgerufen |
| `SerialTransport.setup()`-Patch (`remove_reader`/`add_reader`) | mittel | Vollständig (asyncio loop-Ebene) | 🔄 deployed, Test ausstehend |

## Lösungsplan mit Fallback

### Phase 1 — Quick-Test (deployed, gescheitert)

`delay_before_rx=0.015` in `RS485Settings` brachte keine ausreichende Verbesserung.
Das Echo vom CH348L trifft noch innerhalb des RX-Fensters ein.

### Phase 3 — PTY Echo-Filter Proxy (gescheitert)

Zwei Varianten wurden entwickelt und getestet, beide scheiterten.

**Variante 1 — `sent_buffer` Pattern-Matching:**

```python
for byte in data:
    if i < len(sent_buffer) and byte == sent_buffer[i]:
        i += 1  # Echo-Byte matched → verwerfen
    else:
        result.append(byte)
```

Fehler: asyncio-Race-Condition — Echo kann eintreffen, bevor `_on_pty_data`
den `sent_buffer` befüllt hat. Ergebnis: Echo wurde nicht gefiltert.

**Variante 2 — Reader-Pause (analog zum pymodbus-Client):**

```python
def _on_pty_data(self) -> None:
    data = os.read(self._master_fd, 4096)
    self._ser.write(data)
    self._loop.remove_reader(self._ser.fileno())      # RX stumm schalten
    self._loop.call_later(0.025, self._reenable_serial_reader)

def _reenable_serial_reader(self) -> None:
    self._ser.reset_input_buffer()                    # Echo verwerfen
    self._loop.add_reader(self._ser.fileno(), self._on_serial_data)
```

Ergebnis aus HA-Log: Echo-Bytes `0x2 0x10 0x0 0x1e 0xc5` und
`0x2 0x50 0x56 0x4 0x64` erscheinen weiterhin in pymodbus `extra data`.
Mögliche Ursachen noch nicht diagnostiziert.

**Logging-Filter `_SimulatorOnlyFilter` (unzureichend):**

Pattern-Matching auf `"send: 0x1 "` / `"recv: 0x1 "` deckt nicht alle
pymodbus-Meldungen ab. Weitere Nachrichten des Growatt-Verkehrs ohne dieses
Muster passieren den Filter — für die Analyse nicht brauchbar genug.

### Offene Testergebnisse

- [x] Phase 1 (`delay_before_rx=0.015`) → kein ausreichender Effekt
- [x] Phase 3 Proxy (sent_buffer) → asyncio-Race, nicht funktional
- [x] Phase 3 Proxy (reader-pause) → Echo trotzdem in pymodbus sichtbar
- [x] Logging-Filter → Pattern unvollständig, für Analyse ungeeignet
- [x] pymodbus Monkey-Patch (`_apply_modbus_echo_patch`) → gescheitert

**Monkey-Patch-Ansatz (gescheitert):**

Deadline-basiertes Verwerfen in `datagram_received` via `ModbusProtocol`-Monkey-Patch
mit `is_server`-Guard (Growatt-Client unberührt). Code in `sensor.py` implementiert
und deployed. Echo-Spam bleibt trotzdem sichtbar — Ursache unbekannt.

```python
def _patched_send(self, data, addr=None):
    if self.is_server:
        self._echo_deadline = time.monotonic() + 0.030
    _orig_send(self, data, addr)

def _patched_datagram_received(self, data, addr):
    if self.is_server and time.monotonic() < getattr(self, "_echo_deadline", 0.0):
        return  # verwerfen
    _orig_datagram_received(self, data, addr)
```

**Hypothese warum es nicht funktioniert:** pymodbus auf HAOS nutzt möglicherweise
nicht die Klasse aus dem importierten Modul-Objekt direkt, sondern instanziiert
über einen internen Factory-Mechanismus, der den Patch umgeht. Oder `datagram_received`
wird auf einem anderen Codepfad (z. B. über asyncio Protocol-Dispatch) aufgerufen, der
vom Patch nicht erfasst wird.

**Bewiesene Ursache (2026-04-04):** Quellcode-Analyse von `pymodbus.transaction.transaction`
zeigt:

```python
# In TransactionManager.pdu_send() — der async Server-Sendepfad:
packet = self.framer.buildFrame(...)
if self.is_sync and self.comm_params.handle_local_echo:  # ← is_sync=False!
    self.sent_buffer = packet                            # ← WIRD NIE ERREICHT
self.low_level_send(self.trace_packet(True, packet), addr=addr)
```

`ModbusSerialServer` läuft immer asynchron (`is_sync=False`). Die Bedingung
`self.is_sync` schließt den async Server-Pfad vom `sent_buffer`-Setzen aus.
Das ist ein pymodbus-Designfehler: `handle_local_echo` funktioniert im async
Server-Betrieb grundsätzlich nicht. Damit war der Monkey-Patch auf `datagram_received`
von Anfang an zwecklos — `self.sent_buffer` war immer `b""`, sodass der Echo-Zweig
nie ausgeführt wurde.

### Fazit: pymodbus-interne Ansätze erschöpft

Alle Ansätze innerhalb der pymodbus-API (`handle_local_echo`, Monkey-Patching auf
pymodbus-Klassen) sind strukturell zum Scheitern verurteilt, weil `sent_buffer` im
async Server-Pfad nie gesetzt wird. Die Lösung muss **unterhalb** von pymodbus
ansetzen.

## Hardware-Austausch CH348L → FT232RNL + SP485EEN (gescheitert)

Am 2026-04-04 wurde der Waveshare 4-Port CH348L-Adapter durch den
**Waveshare Industrial USB TO RS485** (FT232RNL + SP485EEN) ersetzt.

**Ergebnis:** Identisches Echo-Verhalten. Kein Unterschied.

**Ursache:** Beide Adapter-Familien verwenden eine **RC-Delay-Schaltung** für
auto-direction, nicht die FTDI TXDEN-Funktion (CBUS2). Der FT232RNL-Chip hat
zwar eine TXDEN-Funktion, aber das Waveshare-Board verbindet CBUS2 nicht mit
dem DE/RE-Pin des SP485EEN — stattdessen sitzt ein RC-Kondensator dazwischen,
der auf ~115200 Baud optimiert ist. Bei 9600 Baud kippt die Timing-Schaltung
Bits im Echo.

**Zusatzbeobachtung:** Nach Herausziehen des CH348L-Adapters verschwanden die
`0x1`-Einträge im pymodbus-Log vollständig. Sie waren **kein** RS485-Cross-Talk,
sondern Growatt-Client-Traffic, der im gleichen `[pymodbus.logging]`-Logger
erfasst wurde.

## Phase 5 — asyncio Transport-Level Filter (2026-04-04, gescheitert)

Der `_RS485EchoFilter` wurde als `asyncio.Protocol`-Wrapper um den `ServerRequestHandler`
injiziert (`ModbusSerialServer.callback_new_connection`-Patch). Er sollte `transport.write()`
hooking um `_pending`-Zähler zu führen und in `data_received()` die entsprechenden Bytes
zu verwerfen.

**Ergebnis (Log 2026-04-04T18-08-31):** `RS485 echo filter: ACTIVE` erscheint im Log,
aber keinerlei `sent X bytes`-Meldungen — der write-Hook wurde nie aufgerufen.
`Frame check failed` weiterhin unverändert.

**Root-Cause:** `SerialTransport.setup()` ruft als Erstes `add_reader(fd, intern_read_ready)`
auf und erst danach (via `loop.call_soon`) `intern_protocol.connection_made`. Das bedeutet:
der Reader-Callback ist bereits aktiv bevor unser `connection_made`-Wrapper den write-Hook
installieren kann. Der erste eingehende Request (und damit die erste Response + Echo) passiert
vor der Hook-Installation. Danach bleibt `_pending` dauerhaft 0, weil unser
`connection_made` auf der `_RS485EchoFilter`-Instanz aufgerufen wird, aber das
`transport`-Objekt das `SerialTransport` direkt an `intern_protocol` übergibt, nicht an
den Wrapper.

```python
# SerialTransport.setup() (vereinfacht):
def setup(self) -> None:
    self.async_loop.add_reader(fd, self.intern_read_ready)  # ← ZUERST: Reader aktiv
    self.async_loop.call_soon(self.intern_protocol.connection_made, self)  # ← DANACH
    #                          ^
    #                          intern_protocol = _RS485EchoFilter, aber
    #                          self = SerialTransport — write-Hook kann
    #                          nicht auf SerialTransport.write gesetzt werden
    #                          weil __slots__ = () keine Instanz-Attribute
    #                          erlaubt
```

## Phase 6 — SerialTransport Klassen-Patch (2026-04-04, deployed)

### Grundprinzip

Der Patch operiert auf `pymodbus.transport.serialtransport.SerialTransport` —
der pyserial I/O-Schicht, eine Ebene unterhalb von asyncio-Protokollen:

```text
pyserial (sync_serial.read/write)
     ↓
 SerialTransport._st_write / _st_read_ready  ← HIER greift der Patch
     ↓
asyncio (intern_protocol.data_received)
     ↓
pymodbus ServerRequestHandler
     ↓
RTU Framer → gültige PDUs
```

### Implementierung

Direkte Methodenersetzung auf `SerialTransport`-Klassenebene. Keine Instanz-Wrapper,
kein Timing-Race:

```python
_echo_pending: dict[int, int] = {}  # id(instance) → ausstehende Echo-Bytes

def _st_write(self, data):
    _echo_pending[id(self)] = _echo_pending.get(id(self), 0) + len(data)
    _orig_st_write(self, data)          # pyserial.write()

def _st_read_ready(self):
    data = self.sync_serial.read(1024)  # pyserial.read()
    if not data:
        return
    pending = _echo_pending.get(id(self), 0)
    if pending:
        skip = min(len(data), pending)
        _echo_pending[id(self)] = pending - skip
        data = data[skip:]              # Echo-Bytes vor Protokoll-Übergabe verwerfen
        if not data:
            return
    self.intern_protocol.data_received(data)  # saubere Bytes an pymodbus

SerialTransport.write = _st_write
SerialTransport.intern_read_ready = _st_read_ready
```

### Root-Cause des Scheiterns (2026-04-11 bewiesen)

`SerialTransport.setup()` (Quellcode `serialtransport.py:45`) registriert den Callback so:

```python
def setup(self) -> None:
    self.async_loop.add_reader(self.sync_serial.fileno(), self.intern_read_ready)
    self.async_loop.call_soon(self.intern_protocol.connection_made, self)
```

`self.intern_read_ready` wird **zum Zeitpunkt des `add_reader()`-Aufrufs** ausgewertet —
das Ergebnis ist ein gebundenes Methodenobjekt (`bound method`) das direkt auf die
Original-Implementierung zeigt. Der asyncio Loop speichert dieses Objekt als Callback.

Unser Klassen-Patch `SerialTransport.intern_read_ready = _st_read_ready` kommt beim
Modul-Import vor der Instanziierung — aber das spielt keine Rolle: `setup()` wird aufgerufen
**nachdem** die Instanz existiert, und `self.intern_read_ready` schlägt dabei zuerst im
Instanz-`__dict__` nach, findet dort nichts, und findet die Klassen-Methode — aber
das zurückgegebene bound-method-Objekt ist eine **Kopie** zum Zeitpunkt des Lookups.
Nach dem `add_reader()`-Aufruf hat der Loop einen direkten Pointer auf diese Kopie —
spätere Änderungen an der Klasse ändern diesen Pointer nicht mehr.

**Beweis aus dem Log (2026-04-04T19-53-16):** Null `RS485 echo filter: skipped X bytes`
Meldungen — `_st_read_ready` wurde **nie aufgerufen**.

**Konsequenz:** Der `_st_write`-Patch (write-Seite) funktioniert, weil Python bei jedem
`instance.write(data)`-Aufruf den MRO neu traversiert. `_echo_pending` wurde korrekt
befüllt — aber niemals geleert, da die Read-Seite nie gefiltert hat.

## Phase 7 — `SerialTransport.setup()`-Patch (2026-04-11, deployed)

### Grundprinzip

Statt `intern_read_ready` auf Klassenebene zu ersetzen (was den bereits registrierten
asyncio-Callback nicht ändert), patchen wir `setup()` selbst. Nach dem originalen
`setup()`-Aufruf (der `add_reader` mit der Original-Methode aufgerufen hat) ersetzen
wir den registrierten Callback durch unseren Filter:

```text
_patched_setup() aufgerufen
    → _orig_st_setup() aufrufen  (registriert Original-Callback via add_reader)
    → loop.remove_reader(fd)     (entfernt Original-Callback)
    → loop.add_reader(fd, lambda: _st_read_ready_impl(self))  (unser Filter)
```

Kein Binding-Problem — wir operieren direkt auf dem asyncio Event-Loop nach dem
Originalpfad. Der Loop hält danach unsere Lambda-Funktion, nicht die Original-Methode.

### Implementierung

```python
_echo_pending: dict[int, int] = {}  # id(instance) → ausstehende Echo-Bytes

def _st_write(self, data):
    _echo_pending[id(self)] = _echo_pending.get(id(self), 0) + len(data)
    _orig_st_write(self, data)

def _st_read_ready_impl(transport):   # kein self — kein Binding-Problem
    data = transport.sync_serial.read(1024)
    if not data:
        return
    pending = _echo_pending.get(id(transport), 0)
    if pending:
        skip = min(len(data), pending)
        _echo_pending[id(transport)] = pending - skip
        data = data[skip:]
        if not data:
            return
    transport.intern_protocol.data_received(data)

_orig_st_setup = SerialTransport.setup

def _patched_setup(self, *args, **kwargs):
    _orig_st_setup(self, *args, **kwargs)         # registriert Original-Callback
    if not self.force_poll:                        # force_poll=True → polling_task, kein add_reader
        fd = self.sync_serial.fileno()
        self.async_loop.remove_reader(fd)          # entfernt Original-Callback
        self.async_loop.add_reader(fd, lambda: _st_read_ready_impl(self))  # unser Filter

SerialTransport.setup = _patched_setup
SerialTransport.write = _st_write
```

### Warum der `setup()`-Patch das Binding-Problem löst

| Eigenschaft | Klassen-Patch `intern_read_ready` | `setup()`-Patch |
| --- | --- | --- |
| asyncio Callback nach Patch | **Original-Bound-Method** (patch wirkungslos) | **Unsere Lambda** (ersetzt via `remove_reader`) |
| Zeitpunkt der Registrierung | vor `add_reader` — zu früh | **nach** `add_reader` — korrekt |
| Binding-Problem | **ja** | **nein** |
| `force_poll`-Guard (Windows) | nein | **ja** |

### Erwartete Log-Ausgabe nach Deploy

```log
WARNING ... RS485 echo filter: SerialTransport.setup() patch INSTALLED
WARNING ... RS485 echo filter: asyncio reader replaced on fd=XX — ACTIVE
DEBUG   ... RS485 echo filter: skipped 9/9 echo bytes, pending now 0
```

Wenn nur die erste Zeile erscheint, aber nicht die zweite: `setup()` wurde nicht
aufgerufen — Transport-Initialisierungssequenz weicht ab.

### Testergebnis

- [ ] Auf HAOS deployed, Test ausstehend

## Logging

Der `_SimulatorOnlyFilter` in `sensor.py` ist installiert, aber unzureichend: pymodbus
erzeugt beim DEBUG-Level Meldungen ohne das `"send: 0x1 "` / `"recv: 0x1 "`-Präfix
(z. B. interne Decoder- und Framer-Meldungen), die den Filter passieren.

Empfohlene Minimaleinstellung für normale Betrieb:

```yaml
logger:
  default: warning
  logs:
    custom_components.sdm630_simulator: debug
    pymodbus: warning
```

## Offene Fragen / Testergebnisse

- [x] Phase 1 (`delay_before_rx=0.015`) → unzureichend
- [x] Phase 3 PTY Proxy (sent_buffer) → asyncio-Race, nicht funktional
- [x] Phase 3 PTY Proxy (reader-pause) → Echo trotzdem in pymodbus sichtbar
- [x] Logging-Filter → Pattern unvollständig
- [x] ModbusProtocol Monkey-Patch → funktionslos (`sent_buffer` im async-Pfad nie gesetzt)
- [x] Hardware-Austausch CH348L → FT232RNL (Waveshare Industrial) → kein Unterschied, RC-Delay-Architektur identisch
- [x] asyncio Transport-Level Filter (`_RS485EchoFilter`) → `connection_made` Timing-Race, write-Hook nie aktiv
- [x] SerialTransport Klassen-Patch (`_st_write`/`_st_read_ready`) → gescheitert: asyncio
  bound-method capture — `add_reader` speichert Bound-Method zum Zeitpunkt des Aufrufs,
  class-level Patch ändert registrierten Callback nicht
- [ ] `SerialTransport.setup()`-Patch (`remove_reader`/`add_reader`) → deployed, Test ausstehend

## Bus-Topologie

```text
THOR Wallbox (Master)
    │
    ├─── RS485 ──── ttyACM2 ──── SDM630-Simulator (unit=2)
    │                               ↑ TX-Echo zurück auf RX (Hardware-Problem)
    │
Growatt SPH10000 (unit=1)
    │
    ├─── RS485 ──── ttyACM0 ──── Growatt-Integration (Client)
```

Die Growatt-Integration und der SDM630-Simulator liegen auf **getrennten RS485-Segmenten**
(separate Kabelpaare am 4-Port-Konverter). Cross-Bus-Störungen sind daher ausgeschlossen.
