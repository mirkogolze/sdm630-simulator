import logging
from datetime import datetime, timedelta
from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
)
from homeassistant.components.sensor import (
    RestoreSensor,
    SensorDeviceClass,
    SensorEntity,
)
from homeassistant.const import CONF_NAME, STATE_UNAVAILABLE, STATE_UNKNOWN
from homeassistant.core import callback
from homeassistant.helpers.event import (
    async_track_state_change_event,
    async_track_time_interval,
)
from homeassistant.util import dt as dt_util
from pymodbus.server import StartAsyncSerialServer
from pymodbus.framer import FramerType
try:
    import pymodbus.transport.transport as _pymodbus_transport
except (ImportError, TypeError):
    _pymodbus_transport = None  # type: ignore[assignment]
from .modbus_server import (
    context,
    identity,
    input_data_block,
)
from .sdm630_input_registers import TOTAL_POWER
from . import sdm630_input_registers as _input_regs
from . import CONF_ENTITIES, CONF_REGISTER_MAPPINGS, DEFAULTS, DOMAIN

# ── RS485 Echo Suppression Monkey-Patch ───────────────────────────────────────
# The Waveshare USB-RS485 adapter (FT232RNL + SP485EEN) uses an RC delay circuit
# for automatic direction control.  This causes the server to receive a corrupted
# echo of every response it sends.  pymodbus's built-in handle_local_echo uses a
# byte-level content comparison (startswith) which fails on corrupted echoes.
#
# This patch replaces the comparison with length-based suppression: after sending
# N bytes, the next N received bytes are silently discarded regardless of content.
# This is safe because at 9600 baud the echo completes in ~100 ms while the THOR
# wallbox polls only every ~60 s, so echo and real requests never overlap.
if _pymodbus_transport is not None:
    from pymodbus.logging import Log as _PymbLog

    _original_datagram_received = _pymodbus_transport.ModbusProtocol.datagram_received

    def _echo_aware_datagram_received(
        self: _pymodbus_transport.ModbusProtocol,
        data: bytes,
        addr: tuple | None,
    ) -> None:
        """Drop TX echo by byte count instead of content comparison."""
        if self.comm_params.handle_local_echo and self.sent_buffer:
            skip = min(len(data), len(self.sent_buffer))
            _PymbLog.debug(
                "recv skipping {} echo bytes (length-based), remaining sent_buffer={}",
                skip,
                len(self.sent_buffer) - skip,
            )
            self.sent_buffer = self.sent_buffer[skip:]
            data = data[skip:]
            if not data:
                return
        # ---- normal receive path (copied from original, minus echo branch) ----
        _PymbLog.transport_dump(_PymbLog.RECV_DATA, data, self.recv_buffer)
        if len(self.recv_buffer) > 1024:
            self.recv_buffer = b""
        self.recv_buffer += data
        cut = self.callback_data(self.recv_buffer, addr=addr)
        self.recv_buffer = self.recv_buffer[cut:]
        if self.recv_buffer:
            _PymbLog.transport_dump(_PymbLog.EXTRA_DATA, None, self.recv_buffer)

    _pymodbus_transport.ModbusProtocol.datagram_received = _echo_aware_datagram_received  # type: ignore[assignment]

# ── End Monkey-Patch ──────────────────────────────────────────────────────────

# Maps register constant names (e.g. "PHASE_1_VOLTAGE") to their PDU addresses.
# Built dynamically from all uppercase int attributes in sdm630_input_registers.
REGISTER_NAME_TO_ADDRESS: dict[str, int] = {
    k: v for k, v in vars(_input_regs).items()
    if k == k.upper() and isinstance(v, int)
}
from .surplus_engine import (
    SurplusEngine,
    SensorSnapshot,
    EvaluationResult,
    CACHE_KEY_SOC,
    CACHE_KEY_POWER_TO_GRID,
    CACHE_KEY_PV_PRODUCTION,
    CACHE_KEY_POWER_TO_USER,
    CACHE_KEY_POWER_FROM_GRID,
    SOC_HARD_FLOOR,
)

_LOGGER = logging.getLogger(__name__)

DEFAULT_NAME = "SDM630 Simulated Meter"
SCAN_INTERVAL = timedelta(seconds=10)

# Maps entity role keys (from config['entities']) to sensor cache keys.
# Only numeric state subscriptions are listed here (sun/weather are excluded).
ENTITY_ROLE_TO_CACHE_KEY = {
    "soc":             CACHE_KEY_SOC,
    "power_to_grid":   CACHE_KEY_POWER_TO_GRID,
    "pv_production":   CACHE_KEY_PV_PRODUCTION,
    "power_to_user":   CACHE_KEY_POWER_TO_USER,
    "power_from_grid": CACHE_KEY_POWER_FROM_GRID,
}

WALLBOX_POLL_WARNING_THRESHOLD: int = 300  # seconds

# ── RS485 Serial Port ─────────────────────────────────────────────────────────
# Waveshare Industrial USB-RS485 (FT232RNL + SP485EEN, automatic transceiving).
# After plugging in, check the stable by-id path with:
#   ls /dev/serial/by-id/
# and replace the placeholder below with the actual symlink target.
SDM630_PORT: str = "/dev/ttyUSB0"

async def start_modbus_server() -> None:
    """Start the Modbus RTU serial server."""
    try:
        _LOGGER.info("Starting SDM630 Modbus Serial Simulator on %s...", SDM630_PORT)
        await StartAsyncSerialServer(
            context=context,
            identity=identity,
            port=SDM630_PORT,
            framer=FramerType.RTU,
            stopbits=1,
            bytesize=8,
            parity="E",
            baudrate=9600,
            handle_local_echo=True,
            ignore_missing_slaves=True,
        )
    except Exception as e:
        _LOGGER.error("Failed to start Modbus server: %s", str(e))


class _SimulatorOnlyFilter(logging.Filter):
    """Suppress pymodbus DEBUG/INFO messages that belong to the Growatt (unit=0x1) connection.

    When pymodbus.logging is set to DEBUG both Growatt and SDM630 frame traffic
    appears.  This filter passes all WARNING+ records unchanged and drops
    frame-level DEBUG/INFO records that contain the Growatt unit-address byte
    (0x1) so that only SDM630 simulator traffic (unit 0x2) remains visible.

    Install once at startup::

        logging.getLogger("pymodbus.logging").addFilter(_SimulatorOnlyFilter())

    Then enable live via HA Developer Tools → Services::

        logger.set_level  {"pymodbus.logging": "debug"}
    """

    _GROWATT_PREFIXES = ("send: 0x1 ", "recv: 0x1 ", "Processing: 0x1 ")

    def filter(self, record: logging.LogRecord) -> bool:  # noqa: A003
        if record.levelno >= logging.WARNING:
            return True
        msg = record.getMessage()
        for prefix in self._GROWATT_PREFIXES:
            if prefix in msg:
                return False
        return True


async def async_setup_platform(hass, config, async_add_entities, discovery_info=None):
    """Set up the SDM630 simulated sensor."""
    # The component config (with entities, thresholds etc.) is stored in
    # hass.data[DOMAIN] by __init__.py. The `config` argument here is the
    # platform config which is empty when loaded via async_load_platform.
    component_cfg = hass.data.get(DOMAIN, {}).get("config", config)

    # Install pymodbus log filter once — allows enabling pymodbus DEBUG without
    # flooding logs with Growatt (unit=0x1) frame traffic.
    logging.getLogger("pymodbus.logging").addFilter(_SimulatorOnlyFilter())

    name = component_cfg.get(CONF_NAME, DEFAULT_NAME)
    hass.loop.create_task(start_modbus_server())

    raw_surplus_sensor = SDM630RawSurplusSensor()
    reported_surplus_sensor = SDM630ReportedSurplusSensor()
    poll_warning_sensor = SDM630WallboxPollWarningSensor()
    wallbox_last_poll_sensor = SDM630WallboxLastPollSensor()
    wallbox_last_poll_sensor.set_warning_sensor(poll_warning_sensor)
    input_data_block.set_poll_callback(wallbox_last_poll_sensor.on_poll)

    sensor = SDM630SimSensor(name, hass, component_cfg)
    sensor.set_surplus_sensors(raw_surplus_sensor, reported_surplus_sensor)

    async_add_entities([
        sensor,
        raw_surplus_sensor,
        reported_surplus_sensor,
        wallbox_last_poll_sensor,
        poll_warning_sensor,
    ])


class SDM630RawSurplusSensor(RestoreSensor):
    """Sensor exposing raw (unfiltered) surplus power in W."""

    _attr_should_poll = False
    _attr_native_unit_of_measurement = "W"
    _attr_device_class = SensorDeviceClass.POWER

    def __init__(self) -> None:
        self._attr_name = "SDM Raw Surplus"
        self._attr_unique_id = "sdm_raw_surplus"
        self._attr_native_value = None

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        last_data = await self.async_get_last_sensor_data()
        if last_data and last_data.native_value is not None:
            try:
                self._attr_native_value = float(last_data.native_value)  # type: ignore[arg-type]
            except (ValueError, TypeError):
                _LOGGER.warning("Could not restore raw surplus value: %r",
                                last_data.native_value)

    @callback
    def update_value(self, value_w: float) -> None:
        """Push a new surplus value (in W) — non-blocking, called from evaluation tick."""
        self._attr_native_value = round(value_w, 1)
        self.async_write_ha_state()


class SDM630ReportedSurplusSensor(RestoreSensor):
    """Sensor exposing reported (hysteresis-filtered) surplus power in W."""

    _attr_should_poll = False
    _attr_native_unit_of_measurement = "W"
    _attr_device_class = SensorDeviceClass.POWER

    def __init__(self) -> None:
        self._attr_name = "SDM Reported Surplus"
        self._attr_unique_id = "sdm_reported_surplus"
        self._attr_native_value = None

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        last_data = await self.async_get_last_sensor_data()
        if last_data and last_data.native_value is not None:
            try:
                self._attr_native_value = float(last_data.native_value)  # type: ignore[arg-type]
            except (ValueError, TypeError):
                _LOGGER.warning("Could not restore reported surplus value: %r",
                                last_data.native_value)

    @callback
    def update_value(self, value_w: float) -> None:
        """Push a new surplus value (in W) — non-blocking, called from evaluation tick."""
        self._attr_native_value = round(value_w, 1)
        self.async_write_ha_state()


class SDM630WallboxLastPollSensor(RestoreSensor):
    """Sensor recording the UTC datetime of the last Modbus FC04 poll from the wallbox."""

    _attr_should_poll = False
    _attr_device_class = SensorDeviceClass.TIMESTAMP

    def __init__(self) -> None:
        self._attr_name = "SDM Wallbox Last Poll"
        self._attr_unique_id = "sdm_wallbox_last_poll"
        self._attr_native_value = None  # datetime | None
        self._poll_warning_sensor: "SDM630WallboxPollWarningSensor | None" = None

    def set_warning_sensor(self, sensor: "SDM630WallboxPollWarningSensor") -> None:
        """Wire the warning sensor so on_poll can notify it."""
        self._poll_warning_sensor = sensor

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        last_data = await self.async_get_last_sensor_data()
        if last_data and last_data.native_value is not None:
            val = last_data.native_value
            if isinstance(val, str):
                val = dt_util.parse_datetime(val)
            self._attr_native_value = val

    @callback
    def on_poll(self) -> None:
        """Called by Modbus poll hook — runs in HA event loop, must be non-blocking."""
        try:
            now = dt_util.utcnow()
            self._attr_native_value = now
            self.async_write_ha_state()
            if self._poll_warning_sensor is not None:
                self._poll_warning_sensor.set_last_poll_dt(now)
            _LOGGER.debug("SDM Wallbox last poll updated: %s", now)
        except Exception:  # noqa: BLE001
            _LOGGER.warning("Failed to update sdm_wallbox_last_poll sensor", exc_info=True)


class SDM630WallboxPollWarningSensor(BinarySensorEntity):
    """Binary sensor that turns on when no wallbox poll has been received for >= 5 min."""

    _attr_should_poll = False
    _attr_device_class = BinarySensorDeviceClass.PROBLEM

    def __init__(self) -> None:
        self._attr_name = "SDM Wallbox Poll Warning"
        self._attr_unique_id = "sdm_wallbox_poll_warning"
        self._attr_is_on = False
        self._last_poll_dt: datetime | None = None

    def set_last_poll_dt(self, dt: datetime) -> None:
        """Called by SDM630WallboxLastPollSensor.on_poll — non-blocking, no I/O."""
        self._last_poll_dt = dt

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        self.async_on_remove(
            async_track_time_interval(
                self.hass,
                self._evaluate_warning,
                timedelta(seconds=60),
            )
        )

    @callback
    def _evaluate_warning(self, now) -> None:
        if self._last_poll_dt is None:
            is_on = False
        else:
            elapsed = (now - self._last_poll_dt).total_seconds()
            is_on = elapsed >= WALLBOX_POLL_WARNING_THRESHOLD
        if is_on != self._attr_is_on:
            self._attr_is_on = is_on
            self.async_write_ha_state()
            _LOGGER.debug(
                "SDM Wallbox poll warning → %s (elapsed=%s s)",
                "ON" if is_on else "OFF",
                None if self._last_poll_dt is None
                else int((now - self._last_poll_dt).total_seconds()),
            )


class SDM630SimSensor(SensorEntity):
    def __init__(self, name, hass, config):
        """Initialize the sensor."""
        self._attr_name = name
        self._attr_native_value = None
        self._attr_native_unit_of_measurement = "Watt"
        self._attr_unique_id = "sdm630_simulator_power"
        self._attr_should_poll = False
        self.hass = hass
        self._config = config
        self._sensor_cache: dict[str, tuple[float, datetime | None, bool]] = {}
        self._engine: SurplusEngine | None = None
        self._first_tick: bool = True
        self._entity_to_cache_key: dict[str, str] = {}
        self._cache_key_to_entity: dict[str, str] = {}
        self._failsafe_reason_logged: str | None = None
        self._invalidation_reasons: dict[str, str] = {}
        self._raw_surplus_sensor: SDM630RawSurplusSensor | None = None
        self._reported_surplus_sensor: SDM630ReportedSurplusSensor | None = None
        self._entity_to_register: dict[str, int] = {}

    def set_surplus_sensors(
        self,
        raw_sensor: "SDM630RawSurplusSensor",
        reported_sensor: "SDM630ReportedSurplusSensor",
    ) -> None:
        """Store references to the surplus dashboard sensors."""
        self._raw_surplus_sensor = raw_sensor
        self._reported_surplus_sensor = reported_sensor

    async def async_added_to_hass(self) -> None:
        """Run when entity about to be added to hass."""
        await super().async_added_to_hass()

        self._engine = SurplusEngine(self._config)

        entities_cfg = self._config.get(CONF_ENTITIES, {})
        self._entity_to_cache_key: dict[str, str] = {
            entity_id: ENTITY_ROLE_TO_CACHE_KEY[role]
            for role, entity_id in entities_cfg.items()
            if role in ENTITY_ROLE_TO_CACHE_KEY
        }
        self._cache_key_to_entity: dict[str, str] = {
            v: k for k, v in self._entity_to_cache_key.items()
        }
        numeric_entity_ids = list(self._entity_to_cache_key.keys())

        if numeric_entity_ids:
            self.async_on_remove(
                async_track_state_change_event(
                    self.hass, numeric_entity_ids, self._handle_state_change
                )
            )

        # Seed cache with current state of all tracked entities so that
        # sensors which already have a value at startup don't stay empty
        # until their next state_changed event.
        for entity_id, cache_key in self._entity_to_cache_key.items():
            state = self.hass.states.get(entity_id)
            if state is None or state.state in (STATE_UNAVAILABLE, STATE_UNKNOWN):
                continue
            try:
                self._sensor_cache[cache_key] = (
                    float(state.state), state.last_updated, True
                )
            except (ValueError, TypeError):
                pass

        # Register mappings: subscribe entities and seed initial values.
        register_mappings: dict = self._config.get(CONF_REGISTER_MAPPINGS, {})
        for entity_id, reg_name in register_mappings.items():
            address = REGISTER_NAME_TO_ADDRESS.get(reg_name)
            if address is None:
                _LOGGER.warning(
                    "sdm630_simulator: unknown register name %r in register_mappings — skipped",
                    reg_name,
                )
                continue
            self._entity_to_register[entity_id] = address

        if self._entity_to_register:
            self.async_on_remove(
                async_track_state_change_event(
                    self.hass,
                    list(self._entity_to_register.keys()),
                    self._handle_register_mapping_change,
                )
            )
            # Seed with current HA state so registers are populated at startup.
            for entity_id, address in self._entity_to_register.items():
                state = self.hass.states.get(entity_id)
                if state is None or state.state in (STATE_UNAVAILABLE, STATE_UNKNOWN):
                    continue
                try:
                    input_data_block.set_float(address, float(state.state))
                except (ValueError, TypeError):
                    pass

        interval = timedelta(seconds=self._config.get("evaluation_interval", 15))
        self.async_on_remove(
            async_track_time_interval(self.hass, self._evaluation_tick, interval)
        )

    @callback
    def _handle_state_change(self, event) -> None:
        """Update sensor cache on state change — no Modbus write here."""
        new_state = event.data.get("new_state")
        if new_state is None:
            return
        entity_id = new_state.entity_id
        cache_key = self._entity_to_cache_key.get(entity_id)
        if cache_key is None:
            return
        if new_state.state in (STATE_UNAVAILABLE, STATE_UNKNOWN):
            last_val = self._sensor_cache.get(cache_key, (0.0, None, True))[0]
            self._sensor_cache[cache_key] = (last_val, dt_util.utcnow(), False)
            self._invalidation_reasons[cache_key] = f" = {new_state.state}"
            return
        try:
            self._sensor_cache[cache_key] = (
                float(new_state.state), new_state.last_updated, True
            )
            self._invalidation_reasons.pop(cache_key, None)
        except (ValueError, TypeError):
            last_val = self._sensor_cache.get(cache_key, (0.0, None, True))[0]
            self._sensor_cache[cache_key] = (last_val, dt_util.utcnow(), False)
            self._invalidation_reasons[cache_key] = ": non-numeric value"
            _LOGGER.debug(
                "Cache invalidated for %s: non-numeric value '%s'",
                entity_id, new_state.state,
            )

    @callback
    def _handle_register_mapping_change(self, event) -> None:
        """Write a mapped entity's new value directly to its Modbus register."""
        new_state = event.data.get("new_state")
        if new_state is None or new_state.state in (STATE_UNAVAILABLE, STATE_UNKNOWN):
            return
        address = self._entity_to_register.get(new_state.entity_id)
        if address is None:
            return
        try:
            input_data_block.set_float(address, float(new_state.state))
        except (ValueError, TypeError):
            _LOGGER.debug(
                "register_mappings: non-numeric value %r from %s — skipped",
                new_state.state, new_state.entity_id,
            )

    def _refresh_cache_timestamps(self) -> None:
        """Refresh cache timestamps from HA state registry.

        Home Assistant may not fire state_changed events when an entity's
        value stays unchanged between integration polls.  This causes the
        cached timestamp to go stale even though the sensor is still alive.
        If HA still reports the entity as available (not unavailable/unknown),
        the sensor is considered alive and the staleness timer is reset to now.
        """
        now = dt_util.utcnow()
        for entity_id, cache_key in self._entity_to_cache_key.items():
            entry = self._sensor_cache.get(cache_key)
            if entry is None:
                continue
            state = self.hass.states.get(entity_id)
            if state is None or state.state in (STATE_UNAVAILABLE, STATE_UNKNOWN):
                continue
            value, _cached_ts, is_valid = entry
            if is_valid:
                self._sensor_cache[cache_key] = (value, now, is_valid)

    def _check_staleness(self) -> str:
        """Check critical sensor cache entries for staleness (Story 4.2).

        Returns a non-empty reason string and triggers FAILSAFE if any critical
        sensor's last_updated timestamp exceeds stale_threshold_seconds.
        Returns empty string if all critical sensors are fresh.
        Sensors absent from _cache_key_to_entity or with None timestamps are
        silently skipped (startup grace — AC4, AC5).
        """
        engine = self._engine
        assert engine is not None
        threshold: int = self._config.get("stale_threshold_seconds", 60)
        now = dt_util.utcnow()
        critical_keys = (CACHE_KEY_SOC, CACHE_KEY_PV_PRODUCTION, CACHE_KEY_POWER_TO_USER)

        for cache_key in critical_keys:
            entity_id = self._cache_key_to_entity.get(cache_key)
            if entity_id is None:
                continue  # not configured — AC5

            entry = self._sensor_cache.get(cache_key)
            if entry is None:
                continue  # not in cache yet — startup grace

            _value, last_changed, _is_valid = entry
            if last_changed is None:
                continue  # explicit startup grace sentinel — AC4

            elapsed = (now - last_changed).total_seconds()
            if elapsed > threshold:  # strict > — AC3
                reason = f"{entity_id} stale for {int(elapsed)}s"
                _LOGGER.warning(
                    "SDM630 FAIL-SAFE: %s stale for %ds. Reporting 0 kW.",
                    entity_id,
                    int(elapsed),
                )
                engine.hysteresis_filter.force_failsafe(reason)
                return reason

        return ""

    async def _evaluation_tick(self, now) -> None:
        """Called by async_track_time_interval at each evaluation cycle."""
        engine = self._engine
        assert engine is not None
        if self._first_tick:
            entities = self._config.get(CONF_ENTITIES, {})
            _LOGGER.info(
                "SDM630 SurplusEngine started. Interval=%ds, entities=%d configured",
                self._config.get("evaluation_interval", 15),
                len(entities),
            )
            self._first_tick = False

        # Refresh cache timestamps from HA state registry so that sensors
        # whose value is unchanged (no state_changed event) are not falsely
        # reported as stale.
        self._refresh_cache_timestamps()

        # Story 4.2: staleness detection (force_failsafe called internally if stale)
        stale_reason = self._check_staleness()

        # Story 4.1: sensor-unavailability fail-safe guard (skip if already stale)
        if stale_reason:
            failsafe_reason = stale_reason
        else:
            cache_valid, validity_reason = self._check_cache_validity()
            if not cache_valid:
                engine.hysteresis_filter.force_failsafe(validity_reason)
                failsafe_reason = validity_reason
            else:
                failsafe_reason = ""

        if failsafe_reason:
            if self._failsafe_reason_logged != failsafe_reason:
                _LOGGER.warning("SDM630 FAIL-SAFE: %s. Reporting 0 kW.", failsafe_reason)
                self._failsafe_reason_logged = failsafe_reason
            else:
                _LOGGER.debug("SDM630 FAIL-SAFE (ongoing): %s", failsafe_reason)
            result = EvaluationResult(
                reported_kw=0.0,
                real_surplus_kw=0.0,
                buffer_used_kw=0.0,
                soc_percent=self._sensor_cache.get(CACHE_KEY_SOC, (0.0,))[0],
                soc_floor_active=SOC_HARD_FLOOR,
                charging_state="FAILSAFE",
                reason=failsafe_reason,
                forecast_available=False,
            )
            _LOGGER.debug(
                "SDM630 Eval: surplus=%.2fkW buffer=%.2fkW SOC=%d%% floor=%d%% "
                "state=%s reported=%.2fkW reason=%s forecast=%s",
                result.real_surplus_kw, result.buffer_used_kw, result.soc_percent,
                result.soc_floor_active, result.charging_state, result.reported_kw,
                result.reason, result.forecast_available,
            )
            self._write_result(result)
            return

        # Story 4.4: value range validation (runs only when unavailability/staleness checks pass)
        range_fail = self._validate_cache()
        if range_fail:
            engine.hysteresis_filter.force_failsafe(range_fail)
            if self._failsafe_reason_logged != range_fail:
                _LOGGER.warning("SDM630 FAIL-SAFE: %s. Reporting 0 kW.", range_fail)
                self._failsafe_reason_logged = range_fail
            else:
                _LOGGER.debug("SDM630 FAIL-SAFE (ongoing): %s", range_fail)
            result = EvaluationResult(
                reported_kw=0.0,
                real_surplus_kw=0.0,
                buffer_used_kw=0.0,
                soc_percent=self._sensor_cache.get(CACHE_KEY_SOC, (0.0,))[0],
                soc_floor_active=SOC_HARD_FLOOR,
                charging_state="FAILSAFE",
                reason=range_fail,
                forecast_available=False,
            )
            self._write_result(result)
            return

        # Recovery: staleness, validity, and range checks all passed
        if self._failsafe_reason_logged is not None:
            engine.hysteresis_filter.resume()
            _LOGGER.info("SDM630 recovered from FAILSAFE. Resuming normal evaluation.")
            self._failsafe_reason_logged = None

        sunset_time = None
        sunrise_time = None
        # sun.sun provides both times; always read it for sunrise + sunset fallback
        sun_state = self.hass.states.get("sun.sun")
        if sun_state and sun_state.state not in (STATE_UNAVAILABLE, STATE_UNKNOWN):
            raw_setting = sun_state.attributes.get("next_setting")
            raw_rising  = sun_state.attributes.get("next_rising")
            if raw_setting:
                sunset_time  = dt_util.parse_datetime(raw_setting)
            if raw_rising:
                sunrise_time = dt_util.parse_datetime(raw_rising)
        else:
            _LOGGER.debug("sun.sun unavailable — solar boundary times set to None")

        # If a dedicated sunset entity is configured (e.g. sensor.sun_next_setting
        # from the sun2 integration), use its state as primary sunset time.
        sunset_entity = self._config.get(CONF_ENTITIES, {}).get("sunset")
        if sunset_entity:
            ss = self.hass.states.get(sunset_entity)
            if ss and ss.state not in (STATE_UNAVAILABLE, STATE_UNKNOWN):
                parsed = dt_util.parse_datetime(ss.state)
                if parsed is not None:
                    sunset_time = parsed
            else:
                _LOGGER.debug("sunset entity %s unavailable — keeping sun.sun value", sunset_entity)

        snapshot = SensorSnapshot(
            soc_percent       = self._sensor_cache.get(CACHE_KEY_SOC, (0.0, None, False))[0],
            power_to_grid_w   = self._sensor_cache.get(CACHE_KEY_POWER_TO_GRID, (0.0, None, False))[0],
            pv_production_w   = self._sensor_cache.get(CACHE_KEY_PV_PRODUCTION, (0.0, None, False))[0],
            power_to_user_w   = self._sensor_cache.get(CACHE_KEY_POWER_TO_USER, (0.0, None, False))[0],
            power_from_grid_w = self._sensor_cache.get(CACHE_KEY_POWER_FROM_GRID, (0.0, None, False))[0],
            timestamp         = now,
            sunset_time       = sunset_time,
            sunrise_time      = sunrise_time,
        )

        result = await engine.evaluate_cycle(snapshot, hass=self.hass)

        # Story 1.4: structured decision log
        _LOGGER.debug(
            "SDM630 Eval: surplus=%.2fkW buffer=%.2fkW SOC=%d%% floor=%d%% "
            "state=%s reported=%.2fkW reason=%s forecast=%s",
            result.real_surplus_kw, result.buffer_used_kw, result.soc_percent,
            result.soc_floor_active, result.charging_state, result.reported_kw,
            result.reason, result.forecast_available,
        )
        if result.charging_state == "FAILSAFE":
            _LOGGER.warning("SDM630 FAIL-SAFE: %s. Reporting 0 kW.", result.reason)

        self._write_result(result)

    def _validate_cache(self) -> "str | None":
        """Validate cache values are within plausible ranges (Story 4.4).

        Returns a reason string if any valid cache entry is out of range,
        or None if all checks pass. Skips entries already marked invalid
        (valid=False) — Story 4.1 handles those independently (AC7).
        """
        ranges = self._config.get("sensor_ranges", DEFAULTS["sensor_ranges"])
        checks = [
            (CACHE_KEY_SOC,           "soc"),
            (CACHE_KEY_POWER_TO_GRID, "power_w"),
            (CACHE_KEY_PV_PRODUCTION, "power_w"),
            (CACHE_KEY_POWER_TO_USER, "power_w"),
        ]
        for cache_key, range_key in checks:
            entry = self._sensor_cache.get(cache_key)
            if entry is None or not entry[2]:  # missing or valid=False: defer to Story 4.1
                continue
            rng = ranges.get(range_key)
            if rng is None:
                continue
            min_val, max_val = rng
            value = entry[0]
            if not (min_val <= value <= max_val):
                entity_id = self._cache_key_to_entity.get(cache_key, cache_key)
                return (
                    f"{entity_id}: value {value} out of range [{min_val}, {max_val}]"
                )
        return None

    def _check_cache_validity(self) -> tuple[bool, str]:
        """Return (is_valid, reason). FAILSAFE reason set if any required entry is missing or invalid."""
        required_keys = [
            CACHE_KEY_SOC,
            CACHE_KEY_POWER_TO_GRID,
            CACHE_KEY_PV_PRODUCTION,
            CACHE_KEY_POWER_TO_USER,
        ]
        for cache_key in required_keys:
            entry = self._sensor_cache.get(cache_key)
            entity_id = self._cache_key_to_entity.get(cache_key, cache_key)
            if entry is None:
                return False, f"{entity_id}: no data received"
            if not entry[2]:
                reason_detail = self._invalidation_reasons.get(cache_key, " = unavailable")
                return False, f"{entity_id}{reason_detail}"
        return True, ""

    def _write_result(self, result: EvaluationResult) -> None:
        """Write evaluation result to Modbus register and HA state."""
        input_data_block.set_float(TOTAL_POWER, result.reported_kw)
        self._attr_native_value = result.reported_kw
        self.async_write_ha_state()
        self._update_surplus_sensors(result)

    def _update_surplus_sensors(self, result: EvaluationResult) -> None:
        """Push surplus values to dashboard sensors — non-blocking, fail-silent."""
        try:
            if self._raw_surplus_sensor is not None:
                self._raw_surplus_sensor.update_value(result.real_surplus_kw * 1000)
            if self._reported_surplus_sensor is not None:
                self._reported_surplus_sensor.update_value(result.reported_kw * 1000)
        except Exception:
            _LOGGER.warning("Failed to update surplus sensors", exc_info=True)
            return
        _LOGGER.debug(
            "SDM630 surplus sensors updated: raw=%.1fW reported=%.1fW",
            result.real_surplus_kw * 1000,
            result.reported_kw * 1000,
        )


