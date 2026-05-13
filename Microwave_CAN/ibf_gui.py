import sys
import time
import can
from dataclasses import dataclass

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QDoubleSpinBox,
    QSpinBox,
    QGroupBox,
    QMessageBox,
)

import matplotlib
matplotlib.use("Qt5Agg")

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# ============================================================
# CAN CONFIGURATION
# ============================================================

PCAN_CHANNEL = "PCAN_USBBUS1"
CAN_BITRATE = 250000
GENERATOR_NUMBER = 1
MAX_POWER_W = 1200


# ============================================================
# IBF STATUS DATA STRUCTURE
# ============================================================

@dataclass
class IBFStatus:
    mains: bool = False
    preheat: bool = False
    ready: bool = False
    microwave_on: bool = False
    cooling_after_stop: bool = False
    interlock_ok: bool = False
    reflection_warning: bool = False

    failure: bool = False
    power_failure: bool = False
    cooling1_failure: bool = False
    cooling2_failure: bool = False
    interlock_failure: bool = False
    overload_failure: bool = False
    arc_failure: bool = False
    reflection_failure: bool = False

    error_code: int = 0
    heating_failure: bool = False
    can_bus_failure: bool = False

    reset_echo: bool = False
    standby_echo: bool = False
    microwave_echo: bool = False
    pulse_echo: bool = False
    pulse_ext_echo: bool = False
    can_remote: bool = False

    power: int = 0
    reflection: int = 0

    u_anode: int = 0
    i_anode: int = 0
    u_amplifier: int = 0
    time_magnetron: int = 0


# ============================================================
# LOW-LEVEL PCAN INTERFACE
# ============================================================

class PCANInterface:
    def __init__(self, channel=PCAN_CHANNEL, bitrate=CAN_BITRATE):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None

    def connect(self):
        self.bus = can.interface.Bus(
            interface="pcan",
            channel=self.channel,
            bitrate=self.bitrate
        )

    def disconnect(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def send(self, can_id, data, is_extended_id=False):
        if self.bus is None:
            raise RuntimeError("CAN bus is not connected")

        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=is_extended_id
        )
        self.bus.send(msg)

    def receive(self, timeout=0.1):
        if self.bus is None:
            return None
        return self.bus.recv(timeout=timeout)


# ============================================================
# IBF GENERATOR COMMAND CLASS
# ============================================================

class IBFGenerator:
    """
    IBF GEN2450 CAN command class.

    Generator 1:
        RX status ID 1 = 0x001
        RX status ID 2 = 0x002
        TX control ID  = 0x004

    Generator 2:
        RX status ID 1 = 0x009
        RX status ID 2 = 0x00A
        TX control ID  = 0x00C
    """

    def __init__(self, can_interface: PCANInterface, generator_number=1):
        self.can = can_interface
        self.generator_number = generator_number

        if generator_number == 1:
            self.status_id_1 = 0x001
            self.status_id_2 = 0x002
            self.control_id = 0x004
        elif generator_number == 2:
            self.status_id_1 = 0x009
            self.status_id_2 = 0x00A
            self.control_id = 0x00C
        else:
            raise ValueError("generator_number must be 1 or 2")

        self.control_frame = bytearray([0x00] * 8)

        # Byte 0 bit definitions from IBF manual
        self.BIT_RESET = 0
        self.BIT_STANDBY_ON = 1
        self.BIT_MICROWAVE_ON = 2
        self.BIT_PULSE_ON = 3
        self.BIT_PULSE_EXT = 4
        self.BIT_CAN_REMOTE = 6

    def _set_bit(self, byte_index, bit_index, state: bool):
        if state:
            self.control_frame[byte_index] |= (1 << bit_index)
        else:
            self.control_frame[byte_index] &= ~(1 << bit_index)

    def _write_u16_be(self, byte_index, value):
        value = int(max(0, min(65535, value)))
        self.control_frame[byte_index] = (value >> 8) & 0xFF
        self.control_frame[byte_index + 1] = value & 0xFF

    def send_control_frame(self):
        self.can.send(
            can_id=self.control_id,
            data=list(self.control_frame),
            is_extended_id=False
        )

    def set_can_remote(self, enable=True):
        self._set_bit(0, self.BIT_CAN_REMOTE, enable)
        self.send_control_frame()

    def reset_fault(self):
        self._set_bit(0, self.BIT_RESET, True)
        self.send_control_frame()

        time.sleep(0.05)

        self._set_bit(0, self.BIT_RESET, False)
        self.send_control_frame()

    def set_standby(self, enable=True):
        self._set_bit(0, self.BIT_STANDBY_ON, enable)
        self._set_bit(0, self.BIT_CAN_REMOTE, True)
        self.send_control_frame()

    def set_microwave(self, enable=True):
        self._set_bit(0, self.BIT_MICROWAVE_ON, enable)
        self._set_bit(0, self.BIT_CAN_REMOTE, True)
        self.send_control_frame()

    def set_power(self, power_watts):
        power_watts = int(max(0, min(MAX_POWER_W, power_watts)))
        self._write_u16_be(2, power_watts)
        self._set_bit(0, self.BIT_CAN_REMOTE, True)
        self.send_control_frame()

    def set_pulse_enable(self, enable=True):
        self._set_bit(0, self.BIT_PULSE_ON, enable)
        self._set_bit(0, self.BIT_CAN_REMOTE, True)
        self.send_control_frame()

    def set_external_pulse(self, enable=True):
        self._set_bit(0, self.BIT_PULSE_EXT, enable)
        self._set_bit(0, self.BIT_CAN_REMOTE, True)
        self.send_control_frame()

    def set_pulse_timing(self, on_time, off_time):
        self._write_u16_be(4, on_time)
        self._write_u16_be(6, off_time)
        self._set_bit(0, self.BIT_CAN_REMOTE, True)
        self.send_control_frame()

    def parse_status_frame_1(self, data, status: IBFStatus):
        """
        CAN ID 0x001 or 0x009:
        Generator actual values and status.
        """
        b_stat = data[0]
        f_stat1 = data[1]
        f_stat2 = data[2]
        ext_stat = data[3]

        status.mains = bool(b_stat & (1 << 0))
        status.preheat = bool(b_stat & (1 << 1))
        status.ready = bool(b_stat & (1 << 2))
        status.microwave_on = bool(b_stat & (1 << 3))
        status.cooling_after_stop = bool(b_stat & (1 << 4))
        status.interlock_ok = bool(b_stat & (1 << 5))
        status.reflection_warning = bool(b_stat & (1 << 7))

        status.failure = bool(f_stat1 & (1 << 0))
        status.power_failure = bool(f_stat1 & (1 << 1))
        status.cooling1_failure = bool(f_stat1 & (1 << 2))
        status.cooling2_failure = bool(f_stat1 & (1 << 3))
        status.interlock_failure = bool(f_stat1 & (1 << 4))
        status.overload_failure = bool(f_stat1 & (1 << 5))
        status.arc_failure = bool(f_stat1 & (1 << 6))
        status.reflection_failure = bool(f_stat1 & (1 << 7))

        status.error_code = f_stat2 & 0x7F
        status.heating_failure = bool(f_stat2 & (1 << 6))
        status.can_bus_failure = bool(f_stat2 & (1 << 7))

        status.reset_echo = bool(ext_stat & (1 << 0))
        status.standby_echo = bool(ext_stat & (1 << 1))
        status.microwave_echo = bool(ext_stat & (1 << 2))
        status.pulse_echo = bool(ext_stat & (1 << 3))
        status.pulse_ext_echo = bool(ext_stat & (1 << 4))
        status.can_remote = bool(ext_stat & (1 << 6))

        status.power = int.from_bytes(data[4:6], byteorder="big", signed=False)
        status.reflection = int.from_bytes(data[6:8], byteorder="big", signed=False)

    def parse_status_frame_2(self, data, status: IBFStatus):
        """
        CAN ID 0x002 or 0x00A:
        Generator actual values.
        """
        status.u_anode = int.from_bytes(data[0:2], byteorder="big", signed=False)
        status.i_anode = int.from_bytes(data[2:4], byteorder="big", signed=False)
        status.u_amplifier = int.from_bytes(data[4:6], byteorder="big", signed=False)
        status.time_magnetron = int.from_bytes(data[6:8], byteorder="big", signed=False)


# ============================================================
# RECEIVE THREAD
# ============================================================

class CANReceiveThread(QThread):
    message_received = pyqtSignal(object)
    error_signal = pyqtSignal(str)

    def __init__(self, can_interface: PCANInterface):
        super().__init__()
        self.can = can_interface
        self.running = True

    def run(self):
        while self.running:
            try:
                msg = self.can.receive(timeout=0.1)
                if msg is not None:
                    self.message_received.emit(msg)
            except Exception as e:
                self.error_signal.emit(str(e))
                time.sleep(0.5)

    def stop(self):
        self.running = False
        self.wait()


# ============================================================
# REFLECTED POWER PLOT
# ============================================================

class ReflectionPowerPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.figure = Figure(figsize=(5, 3))
        self.ax = self.figure.add_subplot(111)
        super().__init__(self.figure)

        self.time_data = []
        self.reflection_data = []
        self.start_time = time.time()

        self.ax.set_title("Reflected Power")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Reflection")
        self.ax.grid(True)

        self.line, = self.ax.plot([], [], linewidth=1.5)

    def add_point(self, reflection):
        t = time.time() - self.start_time

        self.time_data.append(t)
        self.reflection_data.append(reflection)

        if len(self.time_data) > 300:
            self.time_data = self.time_data[-300:]
            self.reflection_data = self.reflection_data[-300:]

        self.line.set_data(self.time_data, self.reflection_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.draw_idle()


# ============================================================
# MAIN GUI
# ============================================================

class IBFControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("IBF GEN2450/1.2 CAN Control")
        self.resize(1200, 650)

        self.can_interface = PCANInterface()
        self.ibf = IBFGenerator(self.can_interface, generator_number=GENERATOR_NUMBER)

        self.rx_thread = None

        self.status = IBFStatus()
        self.status_labels = {}
        self.status_state_cache = {}

        self.latest_reflection = None

        self.init_ui()
        self.connect_can()

    # --------------------------------------------------------
    # UI
    # --------------------------------------------------------

    def init_ui(self):
        main_layout = QHBoxLayout(self)

        left_panel = self.create_left_panel()
        right_panel = self.create_right_panel()

        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(right_panel, 2)

        

    def create_left_panel(self):
        group = QGroupBox("Control")
        layout = QVBoxLayout(group)

        self.remote_button = QPushButton("Enable CAN Remote")
        self.remote_button.setCheckable(True)
        self.remote_button.clicked.connect(self.toggle_can_remote)
        layout.addWidget(self.remote_button)

        self.standby_button = QPushButton("Start Preheating / Standby ON")
        self.standby_button.setCheckable(True)
        self.standby_button.clicked.connect(self.toggle_standby)
        layout.addWidget(self.standby_button)

        self.microwave_button = QPushButton("Microwave ON")
        self.microwave_button.setCheckable(True)
        self.microwave_button.clicked.connect(self.toggle_microwave)
        self.microwave_button.setMinimumHeight(45)

        layout.addWidget(self.microwave_button)

        layout.addSpacing(10)

        layout.addWidget(QLabel("Power Setpoint"))

        self.power_input = QDoubleSpinBox()
        self.power_input.setRange(0, MAX_POWER_W)
        self.power_input.setValue(0)
        self.power_input.setSingleStep(10)
        self.power_input.setDecimals(0)
        self.power_input.setSuffix(" W")
        layout.addWidget(self.power_input)

        self.set_power_button = QPushButton("Set Power")
        self.set_power_button.clicked.connect(self.set_power)
        layout.addWidget(self.set_power_button)

        layout.addSpacing(15)

        pulse_group = QGroupBox("Pulse Mode")
        pulse_layout = QVBoxLayout(pulse_group)

        self.pulse_status_label = QLabel("Pulse mode: OFF")
        self.pulse_status_label.setAlignment(Qt.AlignCenter)
        self.pulse_status_label.setStyleSheet(
            "background-color: white; color: black; padding: 5px; border: 1px solid gray;"
        )
        pulse_layout.addWidget(self.pulse_status_label)

        self.pulse_enable_button = QPushButton("Enable Pulse Mode")
        self.pulse_enable_button.setCheckable(True)
        self.pulse_enable_button.clicked.connect(self.toggle_pulse_mode)
        pulse_layout.addWidget(self.pulse_enable_button)

        self.pulse_ext_button = QPushButton("Use External Pulse Input")
        self.pulse_ext_button.setCheckable(True)
        self.pulse_ext_button.clicked.connect(self.toggle_external_pulse)
        pulse_layout.addWidget(self.pulse_ext_button)

        pulse_layout.addWidget(QLabel("Pulse ON Time"))

        self.pulse_on_input = QSpinBox()
        self.pulse_on_input.setRange(0, 65535)
        self.pulse_on_input.setValue(100)
        self.pulse_on_input.setSingleStep(1)
        self.pulse_on_input.setSuffix(" raw")
        pulse_layout.addWidget(self.pulse_on_input)

        pulse_layout.addWidget(QLabel("Pulse OFF Time"))

        self.pulse_off_input = QSpinBox()
        self.pulse_off_input.setRange(0, 65535)
        self.pulse_off_input.setValue(100)
        self.pulse_off_input.setSingleStep(1)
        self.pulse_off_input.setSuffix(" raw")
        pulse_layout.addWidget(self.pulse_off_input)

        self.set_pulse_timing_button = QPushButton("Set Pulse ON/OFF Time")
        self.set_pulse_timing_button.clicked.connect(self.set_pulse_timing)
        pulse_layout.addWidget(self.set_pulse_timing_button)

        layout.addWidget(pulse_group)

        layout.addSpacing(15)

        self.reset_fault_button = QPushButton("Reset Fault")
        self.reset_fault_button.clicked.connect(self.reset_fault)
        layout.addWidget(self.reset_fault_button)

        layout.addStretch()

        self.connection_label = QLabel("CAN: Disconnected")
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setStyleSheet(
            "background-color: red; color: white; padding: 6px; border-radius: 4px;"
        )
        layout.addWidget(self.connection_label)

        return group

    def create_right_panel(self):
        group = QGroupBox("Status and Reflected Power")
        layout = QVBoxLayout(group)

        status_group = QGroupBox("Status")
        status_layout = QGridLayout(status_group)

        status_names = [
            "MAINS",
            "PREHEAT",
            "READY",
            "MICROWAVE",
            "COOLING_AFTER_STOP",
            "INTERLOCK_OK",
            "REFLECTION_WARNING",
            "FAILURE",
            "POWER_FAILURE",
            "COOLING1_FAILURE",
            "COOLING2_FAILURE",
            "INTERLOCK_FAILURE",
            "OVERLOAD_FAILURE",
            "ARC_FAILURE",
            "REFLECTION_FAILURE",
            "HEATING_FAILURE",
            "CAN_BUS_FAILURE",
            "RESET_ECHO",
            "STANDBY_ECHO",
            "MICROWAVE_ECHO",
            "PULSE_ECHO",
            "PULSE_EXT_ECHO",
            "CAN_REMOTE",
        ]

        for i, name in enumerate(status_names):
            label = QLabel(name)
            label.setAlignment(Qt.AlignCenter)

            font = label.font()
            font.setPointSize(max(1, font.pointSize() - 2))
            label.setFont(font)

            label.setStyleSheet(self.status_style(False))

            row = i // 4
            col = i % 4

            status_layout.addWidget(label, row, col)
            self.status_labels[name] = label
            self.status_state_cache[name] = None

        layout.addWidget(status_group)

        values_group = QGroupBox("Readback Values")
        values_layout = QGridLayout(values_group)

        self.power_readback_label = QLabel("Power: 0")
        self.reflection_readback_label = QLabel("Reflection: 0")
        self.error_code_label = QLabel("Error Code: 0")
        self.u_anode_label = QLabel("U Anode: 0 V")
        self.i_anode_label = QLabel("I Anode: 0 mA")
        self.u_amp_label = QLabel("U Amplifier: 0 V")
        self.time_mag_label = QLabel("Magnetron Time: 0 h")

        values_layout.addWidget(self.power_readback_label, 0, 0)
        values_layout.addWidget(self.reflection_readback_label, 0, 1)
        values_layout.addWidget(self.error_code_label, 0, 2)
        values_layout.addWidget(self.u_anode_label, 1, 0)
        values_layout.addWidget(self.i_anode_label, 1, 1)
        values_layout.addWidget(self.u_amp_label, 1, 2)
        values_layout.addWidget(self.time_mag_label, 1, 3)

        layout.addWidget(values_group)

        self.reflection_plot = ReflectionPowerPlot()
        layout.addWidget(self.reflection_plot)

        return group

    # --------------------------------------------------------
    # Style
    # --------------------------------------------------------

    def status_style(self, active=False):
        if active:
            return """
                QLabel {
                    background-color: red;
                    color: white;
                    border: 1px solid black;
                    border-radius: 4px;
                    padding: 4px;
                }
            """
        return """
            QLabel {
                background-color: white;
                color: black;
                border: 1px solid gray;
                border-radius: 4px;
                padding: 4px;
            }
        """

    def set_status_indicator(self, name, active):
        """
        Update label only when state changes.
        This prevents visible blinking caused by repeated stylesheet updates.
        """
        if name not in self.status_labels:
            return

        if self.status_state_cache.get(name) == active:
            return

        self.status_state_cache[name] = active
        self.status_labels[name].setStyleSheet(self.status_style(active))

    # --------------------------------------------------------
    # CAN
    # --------------------------------------------------------

    def connect_can(self):
        try:
            self.can_interface.connect()

            self.connection_label.setText("CAN: Connected at 250 kbit/s")
            self.connection_label.setStyleSheet(
                "background-color: green; color: white; padding: 6px; border-radius: 4px;"
            )

            self.rx_thread = CANReceiveThread(self.can_interface)
            self.rx_thread.message_received.connect(self.handle_can_message)
            self.rx_thread.error_signal.connect(self.show_can_error)
            self.rx_thread.start()

        except Exception as e:
            self.connection_label.setText("CAN: Connection Failed")
            self.connection_label.setStyleSheet(
                "background-color: red; color: white; padding: 6px; border-radius: 4px;"
            )

            QMessageBox.critical(
                self,
                "CAN Error",
                f"Could not connect to PCAN-USB.\n\n{e}"
            )

    def show_can_error(self, error_message):
        self.connection_label.setText("CAN: Error")
        self.connection_label.setStyleSheet(
            "background-color: red; color: white; padding: 6px; border-radius: 4px;"
        )

    def handle_can_message(self, msg):
        if msg.arbitration_id == self.ibf.status_id_1 and len(msg.data) >= 8:
            self.ibf.parse_status_frame_1(msg.data, self.status)
            self.update_status_from_cache()

        elif msg.arbitration_id == self.ibf.status_id_2 and len(msg.data) >= 8:
            self.ibf.parse_status_frame_2(msg.data, self.status)
            self.update_values_from_cache()

    # --------------------------------------------------------
    # Status update
    # --------------------------------------------------------

    def update_status_from_cache(self):
        s = self.status

        self.set_status_indicator("MAINS", s.mains)
        self.set_status_indicator("PREHEAT", s.preheat)
        self.set_status_indicator("READY", s.ready)
        self.set_status_indicator("MICROWAVE", s.microwave_on)
        self.set_status_indicator("COOLING_AFTER_STOP", s.cooling_after_stop)
        self.set_status_indicator("INTERLOCK_OK", s.interlock_ok)
        self.set_status_indicator("REFLECTION_WARNING", s.reflection_warning)

        self.set_status_indicator("FAILURE", s.failure)
        self.set_status_indicator("POWER_FAILURE", s.power_failure)
        self.set_status_indicator("COOLING1_FAILURE", s.cooling1_failure)
        self.set_status_indicator("COOLING2_FAILURE", s.cooling2_failure)
        self.set_status_indicator("INTERLOCK_FAILURE", s.interlock_failure)
        self.set_status_indicator("OVERLOAD_FAILURE", s.overload_failure)
        self.set_status_indicator("ARC_FAILURE", s.arc_failure)
        self.set_status_indicator("REFLECTION_FAILURE", s.reflection_failure)

        self.set_status_indicator("HEATING_FAILURE", s.heating_failure)
        self.set_status_indicator("CAN_BUS_FAILURE", s.can_bus_failure)

        self.set_status_indicator("RESET_ECHO", s.reset_echo)
        self.set_status_indicator("STANDBY_ECHO", s.standby_echo)
        self.set_status_indicator("MICROWAVE_ECHO", s.microwave_echo)
        self.set_status_indicator("PULSE_ECHO", s.pulse_echo)
        self.set_status_indicator("PULSE_EXT_ECHO", s.pulse_ext_echo)
        self.set_status_indicator("CAN_REMOTE", s.can_remote)

        self.power_readback_label.setText(f"Power: {s.power}")
        self.reflection_readback_label.setText(f"Reflection: {s.reflection}")
        self.error_code_label.setText(f"Error Code: {s.error_code}")

        if self.latest_reflection != s.reflection:
            self.latest_reflection = s.reflection
            self.reflection_plot.add_point(s.reflection)

    def update_values_from_cache(self):
        s = self.status

        self.u_anode_label.setText(f"U Anode: {s.u_anode} V")
        self.i_anode_label.setText(f"I Anode: {s.i_anode} mA")
        self.u_amp_label.setText(f"U Amplifier: {s.u_amplifier} V")
        self.time_mag_label.setText(f"Magnetron Time: {s.time_magnetron} h")

    # --------------------------------------------------------
    # Button callbacks
    # --------------------------------------------------------

    def toggle_can_remote(self):
        enable = self.remote_button.isChecked()

        try:
            self.ibf.set_can_remote(enable)

            if enable:
                self.remote_button.setText("Disable CAN Remote")
            else:
                self.remote_button.setText("Enable CAN Remote")

        except Exception as e:
            self.remote_button.setChecked(not enable)
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def toggle_standby(self):
        enable = self.standby_button.isChecked()

        try:
            self.ibf.set_standby(enable)

            if enable:
                self.standby_button.setText("Stop Preheating / Standby OFF")
            else:
                self.standby_button.setText("Start Preheating / Standby ON")

        except Exception as e:
            self.standby_button.setChecked(not enable)
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def toggle_microwave(self):
        enable = self.microwave_button.isChecked()

        try:
            self.ibf.set_microwave(enable)

            if enable:
                self.microwave_button.setText("Microwave OFF")
            else:
                self.microwave_button.setText("Microwave ON")

        except Exception as e:
            self.microwave_button.setChecked(not enable)
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def set_power(self):
        power = self.power_input.value()

        try:
            self.ibf.set_power(power)
        except Exception as e:
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def toggle_pulse_mode(self):
        enable = self.pulse_enable_button.isChecked()

        try:
            self.ibf.set_pulse_enable(enable)

            if enable:
                self.pulse_enable_button.setText("Disable Pulse Mode")
                self.pulse_status_label.setText("Pulse mode: ON")
                self.pulse_status_label.setStyleSheet(
                    "background-color: red; color: white; padding: 5px; border: 1px solid black;"
                )
            else:
                self.pulse_enable_button.setText("Enable Pulse Mode")
                self.pulse_status_label.setText("Pulse mode: OFF")
                self.pulse_status_label.setStyleSheet(
                    "background-color: white; color: black; padding: 5px; border: 1px solid gray;"
                )

        except Exception as e:
            self.pulse_enable_button.setChecked(not enable)
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def toggle_external_pulse(self):
        enable = self.pulse_ext_button.isChecked()

        try:
            self.ibf.set_external_pulse(enable)

            if enable:
                self.pulse_ext_button.setText("Use Internal Pulse Timing")
            else:
                self.pulse_ext_button.setText("Use External Pulse Input")

        except Exception as e:
            self.pulse_ext_button.setChecked(not enable)
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def set_pulse_timing(self):
        on_time = self.pulse_on_input.value()
        off_time = self.pulse_off_input.value()

        try:
            self.ibf.set_pulse_timing(on_time, off_time)
        except Exception as e:
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    def reset_fault(self):
        try:
            self.ibf.reset_fault()
        except Exception as e:
            QMessageBox.warning(self, "CAN Command Failed", str(e))

    # --------------------------------------------------------
    # Close
    # --------------------------------------------------------

    def closeEvent(self, event):
        try:
            if self.rx_thread is not None:
                self.rx_thread.stop()

            self.can_interface.disconnect()

        except Exception:
            pass

        event.accept()


# ============================================================
# MAIN
# ============================================================

def main():
    app = QApplication(sys.argv)

    window = IBFControlGUI()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
