"""
ibf_can.py

Python CAN communication library for IBF GEN2450/1.2 microwave generator.

Protocol summary for generator 1:

Generator -> Control PC
    0x001 : actual values and status
    0x002 : actual values

Control PC -> Generator
    0x004 : control message

Bitrate:
    250 kbit/s

Hardware:
    PEAK PCAN-USB

Required:
    pip install python-can
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Dict, Callable

import can


# ============================================================
# Data containers
# ============================================================

@dataclass
class IBFStatus:
    # From 0x001 byte 0: B_STAT
    mains: bool = False
    preheat: bool = False
    ready: bool = False
    microwave: bool = False
    cooling_after_stop: bool = False
    interlock_ok: bool = False
    reflection_warning: bool = False

    # From 0x001 byte 1: F_STAT1
    failure: bool = False
    power_failure: bool = False
    cooling1_failure: bool = False
    cooling2_failure: bool = False
    interlock_failure: bool = False
    overload_failure: bool = False
    arc_failure: bool = False
    reflection_failure: bool = False

    # From 0x001 byte 2: F_STAT2
    error_code: int = 0
    heating_failure: bool = False
    can_bus_failure: bool = False

    # From 0x001 byte 3: EXT_STAT
    echo_reset: bool = False
    echo_standby_on: bool = False
    echo_microwave_on: bool = False
    echo_pulse: bool = False
    echo_pulse_ext: bool = False
    echo_can_remote: bool = False

    # From 0x001 byte 4-7
    power_w: int = 0
    reflection_w: int = 0

    # From 0x002
    u_anode_v: int = 0
    i_anode_ma: int = 0
    u_amplifier_v: int = 0
    time_magn_h: int = 0

    last_update_time: float = field(default_factory=time.time)


@dataclass
class IBFCommand:
    """
    Control command sent to CAN ID 0x004.
    """
    reset: bool = False
    standby_on: bool = False
    microwave_on: bool = False
    pulse_on: bool = False
    pulse_ext: bool = False
    can_remote: bool = True
    control_unit_ready: bool = True

    power_setpoint_w: int = 0
    on_time: int = 0
    off_time: int = 0


# ============================================================
# Command builder
# ============================================================

class IBFCommandBuilder:
    """
    Builds CAN frames for the IBF generator control message.
    """

    CONTROL_ID_GEN1 = 0x004
    CONTROL_ID_GEN2 = 0x00C

    @staticmethod
    def build_control_message(
        command: IBFCommand,
        generator: int = 1
    ) -> can.Message:
        """
        Build the CAN control message.

        Byte 0: ST_STAT
            bit 0 RESET
            bit 1 STANDBY_ON
            bit 2 MICROWAVE_ON
            bit 3 PULSE_ON
            bit 4 PULSE_EXT
            bit 6 CAN_REMOTE

        Byte 1: H_STAT1
            bit 5 MAINS_CONTROL_UNIT

        Byte 2-3:
            POWER_SETVALUE high byte, low byte

        Byte 4-5:
            ON_TIME high byte, low byte

        Byte 6-7:
            OFF_TIME high byte, low byte
        """

        b0 = 0x00

        if command.reset:
            b0 |= 0x01
        if command.standby_on:
            b0 |= 0x02
        if command.microwave_on:
            b0 |= 0x04
        if command.pulse_on:
            b0 |= 0x08
        if command.pulse_ext:
            b0 |= 0x10
        if command.can_remote:
            b0 |= 0x40

        b1 = 0x20 if command.control_unit_ready else 0x00

        power = max(0, min(int(command.power_setpoint_w), 65535))
        on_time = max(0, min(int(command.on_time), 65535))
        off_time = max(0, min(int(command.off_time), 65535))

        data = [
            b0,
            b1,
            (power >> 8) & 0xFF,
            power & 0xFF,
            (on_time >> 8) & 0xFF,
            on_time & 0xFF,
            (off_time >> 8) & 0xFF,
            off_time & 0xFF,
        ]

        if generator == 1:
            arbitration_id = IBFCommandBuilder.CONTROL_ID_GEN1
        elif generator == 2:
            arbitration_id = IBFCommandBuilder.CONTROL_ID_GEN2
        else:
            raise ValueError("generator must be 1 or 2")

        return can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )


# ============================================================
# Status decoder
# ============================================================

class IBFStatusDecoder:
    """
    Decodes CAN messages from the IBF generator.
    """

    STATUS1_GEN1 = 0x001
    STATUS2_GEN1 = 0x002

    STATUS1_GEN2 = 0x009
    STATUS2_GEN2 = 0x00A

    @staticmethod
    def decode_status_1(data: bytes, status: IBFStatus) -> IBFStatus:
        """
        Decode CAN ID 0x001 or 0x009.

        Byte 0: B_STAT
        Byte 1: F_STAT1
        Byte 2: F_STAT2
        Byte 3: EXT_STAT
        Byte 4-5: POWER
        Byte 6-7: REFLECTION
        """

        if len(data) != 8:
            return status

        b_stat = data[0]
        f_stat1 = data[1]
        f_stat2 = data[2]
        ext_stat = data[3]

        status.mains = bool(b_stat & 0x01)
        status.preheat = bool(b_stat & 0x02)
        status.ready = bool(b_stat & 0x04)
        status.microwave = bool(b_stat & 0x08)
        status.cooling_after_stop = bool(b_stat & 0x10)
        status.interlock_ok = bool(b_stat & 0x20)
        status.reflection_warning = bool(b_stat & 0x80)

        status.failure = bool(f_stat1 & 0x01)
        status.power_failure = bool(f_stat1 & 0x02)
        status.cooling1_failure = bool(f_stat1 & 0x04)
        status.cooling2_failure = bool(f_stat1 & 0x08)
        status.interlock_failure = bool(f_stat1 & 0x10)
        status.overload_failure = bool(f_stat1 & 0x20)
        status.arc_failure = bool(f_stat1 & 0x40)
        status.reflection_failure = bool(f_stat1 & 0x80)

        status.error_code = f_stat2 & 0x7F
        status.can_bus_failure = bool(f_stat2 & 0x80)

        # The manual also lists heating failure in this byte.
        # Depending on the manual formatting, this may overlap with error-code bits.
        status.heating_failure = bool(f_stat2 & 0x40)

        status.echo_reset = bool(ext_stat & 0x01)
        status.echo_standby_on = bool(ext_stat & 0x02)
        status.echo_microwave_on = bool(ext_stat & 0x04)
        status.echo_pulse = bool(ext_stat & 0x08)
        status.echo_pulse_ext = bool(ext_stat & 0x10)
        status.echo_can_remote = bool(ext_stat & 0x40)

        status.power_w = (data[4] << 8) | data[5]
        status.reflection_w = (data[6] << 8) | data[7]

        status.last_update_time = time.time()

        return status

    @staticmethod
    def decode_status_2(data: bytes, status: IBFStatus) -> IBFStatus:
        """
        Decode CAN ID 0x002 or 0x00A.

        Byte 0-1: U_ANODE, unit V
        Byte 2-3: I_ANODE, unit mA
        Byte 4-5: U_AMPLIFIER, unit V
        Byte 6-7: TIME_MAGN, unit h
        """

        if len(data) != 8:
            return status

        status.u_anode_v = (data[0] << 8) | data[1]
        status.i_anode_ma = (data[2] << 8) | data[3]
        status.u_amplifier_v = (data[4] << 8) | data[5]
        status.time_magn_h = (data[6] << 8) | data[7]

        status.last_update_time = time.time()

        return status


# ============================================================
# Main CAN interface class
# ============================================================

class IBFGeneratorCAN:
    """
    Main class for communicating with the IBF generator over CAN.

    This class:
        - opens the PCAN-USB device
        - sends control messages cyclically
        - receives status frames
        - decodes status into an IBFStatus object
    """

    def __init__(
        self,
        channel: str = "PCAN_USBBUS1",
        bitrate: int = 250000,
        interface: str = "pcan",
        generator: int = 1,
        command_period_s: float = 0.2
    ):
        self.channel = channel
        self.bitrate = bitrate
        self.interface = interface
        self.generator = generator
        self.command_period_s = command_period_s

        self.bus: Optional[can.BusABC] = None

        self.status = IBFStatus()
        self.command = IBFCommand()

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self.on_status_update: Optional[Callable[[IBFStatus], None]] = None
        self.on_raw_message: Optional[Callable[[can.Message], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None

    def connect(self):
        self.bus = can.interface.Bus(
            interface=self.interface,
            channel=self.channel,
            bitrate=self.bitrate
        )

    def disconnect(self):
        self.stop()

        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass

        self.bus = None

    def start(self):
        if self.bus is None:
            self.connect()

        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)

        self._thread = None

    def _loop(self):
        last_send = 0.0

        while self._running:
            try:
                now = time.time()

                if now - last_send >= self.command_period_s:
                    self._send_current_command()
                    last_send = now

                if self.bus is None:
                    time.sleep(0.05)
                    continue

                msg = self.bus.recv(timeout=0.02)

                if msg is None:
                    continue

                if self.on_raw_message is not None:
                    self.on_raw_message(msg)

                self._process_message(msg)

            except Exception as exc:
                if self.on_error is not None:
                    self.on_error(str(exc))
                time.sleep(0.2)

    def _send_current_command(self):
        if self.bus is None:
            return

        with self._lock:
            command_copy = IBFCommand(
                reset=self.command.reset,
                standby_on=self.command.standby_on,
                microwave_on=self.command.microwave_on,
                pulse_on=self.command.pulse_on,
                pulse_ext=self.command.pulse_ext,
                can_remote=self.command.can_remote,
                control_unit_ready=self.command.control_unit_ready,
                power_setpoint_w=self.command.power_setpoint_w,
                on_time=self.command.on_time,
                off_time=self.command.off_time
            )

        msg = IBFCommandBuilder.build_control_message(
            command_copy,
            generator=self.generator
        )

        self.bus.send(msg)

    def _process_message(self, msg: can.Message):
        if msg.is_remote_frame:
            return

        if self.generator == 1:
            status1_id = IBFStatusDecoder.STATUS1_GEN1
            status2_id = IBFStatusDecoder.STATUS2_GEN1
        else:
            status1_id = IBFStatusDecoder.STATUS1_GEN2
            status2_id = IBFStatusDecoder.STATUS2_GEN2

        updated = False

        with self._lock:
            if msg.arbitration_id == status1_id:
                self.status = IBFStatusDecoder.decode_status_1(
                    msg.data,
                    self.status
                )
                updated = True

            elif msg.arbitration_id == status2_id:
                self.status = IBFStatusDecoder.decode_status_2(
                    msg.data,
                    self.status
                )
                updated = True

            status_copy = self.get_status_copy_locked()

        if updated and self.on_status_update is not None:
            self.on_status_update(status_copy)

    def get_status_copy_locked(self) -> IBFStatus:
        return IBFStatus(**self.status.__dict__)

    def get_status(self) -> IBFStatus:
        with self._lock:
            return self.get_status_copy_locked()

    # ========================================================
    # High-level commands
    # ========================================================

    def set_remote(self, enabled: bool = True):
        with self._lock:
            self.command.can_remote = enabled

    def startup(self):
        """
        Put the generator into CAN remote and standby/preheat.
        Microwave remains OFF.
        """
        with self._lock:
            self.command.reset = False
            self.command.can_remote = True
            self.command.control_unit_ready = True
            self.command.standby_on = True
            self.command.microwave_on = False
            self.command.power_setpoint_w = 0

    def microwave_on(self):
        """
        Enable microwave output.

        The generator should already be ready, interlock OK,
        and connected to a proper load/applicator.
        """
        with self._lock:
            self.command.reset = False
            self.command.can_remote = True
            self.command.control_unit_ready = True
            self.command.standby_on = True
            self.command.microwave_on = True

    def microwave_off(self):
        """
        Turn microwave output OFF but keep standby/preheat active.
        """
        with self._lock:
            self.command.microwave_on = False
            self.command.power_setpoint_w = 0
            self.command.standby_on = True
            self.command.can_remote = True
            self.command.control_unit_ready = True

    def full_stop(self):
        """
        Turn microwave and standby OFF while keeping CAN remote active.
        """
        with self._lock:
            self.command.reset = False
            self.command.microwave_on = False
            self.command.standby_on = False
            self.command.power_setpoint_w = 0
            self.command.can_remote = True
            self.command.control_unit_ready = True

    def reset_fault(self, pulse_time_s: float = 0.5):
        """
        Pulse the reset bit briefly, then return to remote idle.
        """

        with self._lock:
            self.command.reset = True
            self.command.can_remote = True
            self.command.control_unit_ready = True
            self.command.microwave_on = False
            self.command.standby_on = False
            self.command.power_setpoint_w = 0

        time.sleep(pulse_time_s)

        with self._lock:
            self.command.reset = False

    def set_power(self, power_w: int):
        """
        Set microwave power setpoint in watts.
        """
        power_w = int(power_w)

        if power_w < 0:
            power_w = 0

        with self._lock:
            self.command.power_setpoint_w = power_w

    def set_pulse_mode(
        self,
        enabled: bool,
        on_time: int = 0,
        off_time: int = 0,
        external_pulse: bool = False
    ):
        with self._lock:
            self.command.pulse_on = enabled
            self.command.pulse_ext = external_pulse
            self.command.on_time = int(on_time)
            self.command.off_time = int(off_time)


# ============================================================
# Simple direct test
# ============================================================

if __name__ == "__main__":
    gen = IBFGeneratorCAN(
        channel="PCAN_USBBUS1",
        bitrate=250000
    )

    def print_status(st: IBFStatus):
        print(
            f"READY={st.ready} "
            f"INTERLOCK={st.interlock_ok} "
            f"MW={st.microwave} "
            f"P={st.power_w} W "
            f"R={st.reflection_w} W "
            f"FAULT={st.failure} "
            f"ERR={st.error_code}"
        )

    gen.on_status_update = print_status

    gen.start()
    gen.startup()

    try:
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        gen.full_stop()
        time.sleep(1.0)
        gen.disconnect()
