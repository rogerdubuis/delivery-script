import sys
import os
import time
import serial
import glob
import subprocess
import re
import json
from datetime import datetime
import csv
import math
import threading

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLineEdit, QLabel, QGridLayout, QDoubleSpinBox, QFrame, QSpinBox, QComboBox,
                             QDialog, QListWidget, QListWidgetItem, QTextEdit, QMessageBox, QInputDialog,
                             QFileDialog, QTableWidget, QTableWidgetItem, QHeaderView, QGroupBox)
from PyQt6.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt6.QtGui import QColor, QIcon
import pyqtgraph as pg
import numpy as np
from collections import deque
from scipy.signal import detrend
from scipy import signal
from scipy.fft import fft, ifft

IS_WINDOWS = sys.platform == 'win32'
DEFAULT_PORT = 'COM3' if IS_WINDOWS else '/dev/ttyS4'

# Constants
TIME_WINDOW = 30
MAX_DATA_POINTS = 1000
WINDOW_SIZE = 20
ENABLE_SIGNAL_PROCESSING = False
LEGACY_FIRMWARE_MODE = True
DEBUG_SERIAL = os.environ.get("DELIVERY_DEBUG_SERIAL", "1") != "0"
CONTROLLER_ADDRESS = os.environ.get("DELIVERY_CONTROLLER_ADDRESS", "253")

# Custom channel names - modify these to change display names throughout the application
# The keys must remain as Ch1-Ch6 for internal functionality
CHANNEL_NAMES = {
    'Ch1': 'Mixture',
    'Ch2': 'Ch2',
    'Ch3': 'Dilution2',
    'Ch4': 'Vapor2',
    'Ch5': 'Vapor1',
    'Ch6': 'Dilution1'
}

CHANNEL_MAX_SCCM = {
    'Ch1': 200, 'Ch2': 200, 'Ch3': 200,
    'Ch4': 200, 'Ch5': 200, 'Ch6': 200
}

# Map from display name back to channel key
DISPLAY_TO_CHANNEL = {v: k for k, v in CHANNEL_NAMES.items()}

# List of internal channel keys (used for iterations)
CHANNEL_KEYS = ['Ch1', 'Ch2', 'Ch3', 'Ch4', 'Ch5', 'Ch6']

def setup_permissions():
    if IS_WINDOWS:
        return True
    try:
        groups_output = subprocess.check_output(['groups'], text=True)
        if 'dialout' not in groups_output:
            print("Adding user to dialout group...")
            subprocess.run(['pkexec', 'usermod', '-a', '-G', 'dialout', subprocess.check_output(['whoami'], text=True).strip()], check=True)
            print("Added to dialout group. Please log out and log back in for changes to take effect.")
            return False
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error setting up permissions: {e}")
        return False

class MFCController:
    _instance = None
    
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            port = cls.find_serial_port()
            cls._instance = cls(port=port)
        return cls._instance
    
    @staticmethod
    def find_serial_port():
        if IS_WINDOWS:
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    print(f"Found {len(ports)} serial port(s): {', '.join(p.device for p in ports)}")
                    return ports[0].device
            except Exception as e:
                print(f"Error enumerating COM ports: {e}")
            return DEFAULT_PORT

        try:
            lspci_output = subprocess.check_output(['lspci'], text=True)
            for line in lspci_output.split('\n'):
                if 'Serial controller: Asix' in line:
                    pci_id = line.split()[0]
                    print(f"Found Asix serial controller at PCI ID: {pci_id}")
                    
                    try:
                        dmesg_output = subprocess.check_output(['pkexec', 'dmesg'], text=True)
                    except subprocess.CalledProcessError:
                        try:
                            dmesg_output = subprocess.check_output(['dmesg'], text=True)
                        except subprocess.CalledProcessError:
                            print("Could not access dmesg output")
                            return DEFAULT_PORT
                    
                    for dmesg_line in dmesg_output.split('\n'):
                        if pci_id in dmesg_line and 'ttyS' in dmesg_line:
                            match = re.search(r'ttyS\d+', dmesg_line)
                            if match:
                                port = f"/dev/{match.group(0)}"
                                print(f"Found serial port: {port}")
                                return port
                    
                    return DEFAULT_PORT
            
            usb_ports = glob.glob('/dev/ttyUSB*')
            if usb_ports:
                return usb_ports[0]
                
            serial_ports = glob.glob('/dev/ttyS*')
            if serial_ports:
                return serial_ports[0]
                
            if sys.platform == 'darwin':
                mac_ports = glob.glob('/dev/cu.*')
                if mac_ports:
                    return mac_ports[0]
                    
            return DEFAULT_PORT
        except Exception as e:
            print(f"Error finding serial port: {e}")
            return DEFAULT_PORT
    
    @staticmethod
    def list_available_ports():
        if IS_WINDOWS:
            try:
                import serial.tools.list_ports
                return sorted(p.device for p in serial.tools.list_ports.comports())
            except Exception:
                return [DEFAULT_PORT]

        usb_ports = glob.glob('/dev/ttyUSB*')
        serial_ports = glob.glob('/dev/ttyS*')
        acm_ports = glob.glob('/dev/ttyACM*')
        
        if sys.platform == 'darwin':
            mac_ports = glob.glob('/dev/cu.*')
            all_ports = usb_ports + serial_ports + acm_ports + mac_ports
        else:
            all_ports = usb_ports + serial_ports + acm_ports
            
        return sorted(all_ports)
    
    def __init__(self, port=None, baudrate=9600, timeout=1):
        if port is None:
            port = DEFAULT_PORT
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            print(f"Connected to controller on {port}")
            if DEBUG_SERIAL:
                print(
                    "Serial settings: "
                    f"baud={baudrate}, timeout={timeout}, write_timeout={timeout}, "
                    f"bytesize=8, parity=N, stopbits=1"
                )
            self.port = port
            
            # Update to use channel names from CHANNEL_NAMES dictionary
            self.channels = {
                channel_key: 0.0 for channel_key in CHANNEL_KEYS
            }
            
        except serial.SerialException as e:
            print(f"Failed to open serial port {port}: {e}")
            raise
        except Exception as e:
            print(f"Error during initialization: {e}")
            raise
    
    def _log_serial_payload(self, direction, command, payload, elapsed_ms=None, note=""):
        if not DEBUG_SERIAL:
            return

        if isinstance(payload, str):
            payload_bytes = payload.encode('ascii', errors='replace')
        else:
            payload_bytes = bytes(payload)

        hex_payload = payload_bytes.hex(' ') if payload_bytes else "<empty>"
        ascii_payload = payload_bytes.decode('ascii', errors='replace') if payload_bytes else ""
        suffix = []
        if elapsed_ms is not None:
            suffix.append(f"{elapsed_ms:.1f} ms")
        if note:
            suffix.append(note)
        suffix_text = f" [{' | '.join(suffix)}]" if suffix else ""
        print(f"{direction} {command.strip()}{suffix_text}")
        print(f"  ascii: {ascii_payload!r}")
        print(f"  hex:   {hex_payload}")

    def send_command(self, command):
        try:
            started = time.monotonic()
            self.ser.reset_input_buffer()
            encoded_command = command.encode()
            self._log_serial_payload("TX", command, encoded_command)
            self.ser.write(encoded_command)
            self.ser.flush()

            deadline = time.monotonic() + max(float(self.ser.timeout or 0), 0.05) + 0.1
            response = bytearray()
            while time.monotonic() < deadline:
                waiting = self.ser.in_waiting
                if waiting:
                    response.extend(self.ser.read(waiting))
                    if b';FF' in response or b'OK' in response or b'NAK' in response:
                        break
                else:
                    time.sleep(0.01)

            if response:
                elapsed_ms = (time.monotonic() - started) * 1000
                self._log_serial_payload("RX", command, response, elapsed_ms=elapsed_ms)
                try:
                    return response.decode('ascii', errors='ignore').strip()
                except Exception:
                    return None
            elapsed_ms = (time.monotonic() - started) * 1000
            self._log_serial_payload("RX", command, b"", elapsed_ms=elapsed_ms, note="timeout/no response")
            return None
        except Exception as e:
            print(f"Error sending command {command.strip()}: {e}")
            return None
    
    def set_flow_rate(self, channel, flow_rate):
        try:
            command = f"SET {channel} {flow_rate:.2f}\r\n"
            response = self.send_command(command)
            
            if channel in self.channels:
                self.channels[channel] = float(flow_rate)
            
            return response
        except Exception as e:
            print(f"Error setting flow rate: {e}")
            return None
    
    def get_flow_rate(self, channel):
        try:
            command = f"GET {channel}\r\n"
            response = self.send_command(command)
            
            if channel in self.channels:
                return self.channels[channel]
            
            return 0.0
        except Exception as e:
            print(f"Error getting flow rate: {e}")
            return 0.0
    
    def read_all_channels(self):
        data = {}
        current_time = time.time()
        
        data['timestamp'] = current_time
        
        for channel in self.channels:
            data[channel] = self.get_flow_rate(channel)
        
        return data
    
    def close(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print(f"Closed connection to {self.port}")
    
    def read_pressure(self, channel):
        """Read flow from specified MFC channel using the 946 FR command."""
        if not 1 <= channel <= 6:
            return None
        cmd = f"@{CONTROLLER_ADDRESS}FR{channel}?;FF\r"
        return self.send_command(cmd)
    
    def parse_pressure_response(self, response):
        """Parse ACK response payload between @<addr>ACK and ;FF."""
        if not response:
            return None
        
        try:
            match = re.search(r'@\d+ACK(.*?);FF', response)
            if not match:
                return None
            
            value = match.group(1)
            if value == "MISCONN":
                return None
            
            try:
                return float(value)
            except ValueError:
                return None
                
        except Exception:
            return None
    
    def read_all_pressures(self, verbose=True):
        """Read all pressure channels with minimal delay"""
        readings = []
        for i in range(1, 7):
            response = self.read_pressure(i)
            value = self.parse_pressure_response(response)
            if isinstance(value, float):
                readings.append(f"{value:.1f}")
            else:
                readings.append("---")
        
        # Display in terminal with custom channel names when explicitly requested.
        if verbose:
            print(f"{CHANNEL_NAMES['Ch1']}: {readings[0]} | {CHANNEL_NAMES['Ch2']}: {readings[1]} | {CHANNEL_NAMES['Ch3']}: {readings[2]} | {CHANNEL_NAMES['Ch4']}: {readings[3]} | {CHANNEL_NAMES['Ch5']}: {readings[4]} | {CHANNEL_NAMES['Ch6']}: {readings[5]}")
        
        return readings
    
    def zero_channel(self, channel):
        """Zero the specified channel"""
        if not 1 <= channel <= 6:
            return False, "Invalid channel number"
        
        cmd = f"@{CONTROLLER_ADDRESS}QZ{channel}?;FF\r"
        response = self.send_command(cmd)
        
        if response:
            if "OK" in response:
                return True, "Zero successful"
            elif "NAK" in response:
                return False, "Zero failed (NAK response)"
            else:
                return False, f"Unexpected response: {response}"
        else:
            return False, "No response received"

    def extract_ack_payload(self, response):
        """Extract payload from @<addr>ACK...;FF responses."""
        if not response:
            return None
        match = re.search(r'@\d+ACK(.*?);FF', response)
        if not match:
            return None
        return match.group(1).strip()

    def probe_mfc_channel(self, channel):
        """Probe a channel using native 946 MFC commands."""
        slot_mapping = {
            1: "A1", 2: "A2",
            3: "B1", 4: "B2",
            5: "C1", 6: "C2"
        }
        probe_commands = {
            'flow': f"@{CONTROLLER_ADDRESS}FR{channel}?;FF\r",
            'setpoint': f"@{CONTROLLER_ADDRESS}QSP{channel}?;FF\r",
            'mode': f"@{CONTROLLER_ADDRESS}QMD{channel}?;FF\r",
        }

        probe_results = {}
        for name, cmd in probe_commands.items():
            response = self.send_command(cmd)
            probe_results[name] = {
                'command': cmd.strip(),
                'response': response,
                'ack_payload': self.extract_ack_payload(response),
                'responded': bool(response),
                'nak': bool(response and "NAK" in response),
            }

        responded_commands = [
            name for name, result in probe_results.items()
            if result['responded'] and not result['nak']
        ]

        return {
            'slot': slot_mapping.get(channel, "Unknown"),
            'channel': channel,
            'responses': probe_results,
            'is_mfc': bool(responded_commands),
            'device_type': "MFC probe response" if responded_commands else "No MFC response",
            'working_commands': responded_commands,
        }

    def probe_mfc_channels(self):
        """Probe all channels using documented 946 MFC commands."""
        mfc_channels = {}

        print("\nMFC Channel Probe:")
        print("------------------")
        print("Slot A = channels 1 and 2 (A1 and A2)")
        print("Slot B = channels 3 and 4 (B1 and B2)")
        print("Slot C = channels 5 and 6 (C1 and C2)")
        print("------------------")

        for channel in range(1, 7):
            info = self.probe_mfc_channel(channel)
            mfc_channels[channel] = info
            if info['is_mfc']:
                print(
                    f"Channel {channel} (Slot {info['slot']}): RESPONSE on "
                    f"{', '.join(info['working_commands'])}"
                )
            else:
                print(f"Channel {channel} (Slot {info['slot']}): NO RESPONSE")

        return mfc_channels
    
    def identify_mfc_channels(self):
        """Identify which channels have MFC hardware installed
        
        Returns:
            dict: Dictionary with channel numbers as keys and MFC status as values
        """
        return self.probe_mfc_channels()
    
    def set_flow_point(self, channel, flow_value):
        """Set flow point for specified channel using QSPn command
        
        Args:
            channel (int): Channel number (1-6)
            flow_value (float): Flow value in SCCM
            
        Returns:
            tuple: (success, message)
        """
        if not 1 <= channel <= 6:
            return False, "Invalid channel number"
        
        # First check if the channel supports flow control
        if not self.supports_flow_control(channel):
            return False, "This channel is monitor-only and does not support flow control"
        
        if not self.ensure_setpoint_mode(channel):
            return False, "Failed to set device to setpoint mode"
        
        # Format flow value in scientific notation with 2 decimal places
        # Example: 100.0 becomes 1.00E+02
        scientific_value = f"{flow_value:.2E}"
        # Keep the E+/- notation exactly as required by the 946
        
        # Create command string - ensure we're using the numeric channel
        cmd = f"@{CONTROLLER_ADDRESS}QSP{channel}!{scientific_value};FF\r"
        response = self.send_command(cmd)
        if response:
            if "ACK" in response:
                # Store the set value in channels
                if f"Ch{channel}" in self.channels:
                    self.channels[f"Ch{channel}"] = flow_value
                return True, f"Flow point set to {flow_value} SCCM"
            elif "NAK" in response:
                return False, "Flow point setting failed (NAK response)"
            else:
                return False, f"Unexpected response: {response}"
        else:
            return False, "No response received"
    
    def ensure_setpoint_mode(self, channel):
        """Make sure the MFC is in set point mode for the specified channel
        
        Args:
            channel (int): Channel number (1-6)
            
        Returns:
            bool: True if successful, False if failed
        """
        if not 1 <= channel <= 6:
            return False
            
        # The 946 uses QMD to query and set the MFC operating mode.
        cmd = f"@{CONTROLLER_ADDRESS}QMD{channel}?;FF\r"
        response = self.send_command(cmd)
        
        if not response:
            if LEGACY_FIRMWARE_MODE:
                print(f"No QMD response for channel {channel}; trying direct QSP due to legacy mode")
                return True
            return False

        mode = (self.extract_ack_payload(response) or "").upper()
        if mode == "SETPOINT":
            return True

        cmd = f"@{CONTROLLER_ADDRESS}QMD{channel}!SETPOINT;FF\r"
        response = self.send_command(cmd)
        if response:
            return "ACK" in response
        if LEGACY_FIRMWARE_MODE:
            print(f"No QMD set-mode response for channel {channel}; trying direct QSP due to legacy mode")
            return True
        return False

    def get_device_type(self, channel):
        """Get the type of device connected to the specified channel
        
        Args:
            channel (int): Channel number (1-6)
            
        Returns:
            tuple: (success, device_type_or_error_message)
        """
        if not 1 <= channel <= 6:
            return False, "Invalid channel number"
        
        # Special case for all channels with older firmware
        # that doesn't support QIT but are known to be MFC devices
        print(f"Bypassing QIT check for channel {channel} (Slot {['A','A','B','B','C','C'][channel-1]}) with older firmware")
        return True, "MFC (Legacy FC 1.23)"
        
        # The following code is disabled to treat all channels as legacy
        '''
        # Query device type
        cmd = f"@{CONTROLLER_ADDRESS}QIT{channel}?;FF\r"  # Query Instrument Type
        response = self.send_command(cmd)
        
        if response:
            if "ACK" in response:
                # Extract device type from response
                device_info = re.sub(r'^@\\d+ACK', '', response).replace(";FF", "").strip()
                return True, device_info
            else:
                return False, f"Unexpected response: {response}"
        else:
            return False, "No response received"
        '''

    def get_setpoint(self, channel):
        """Get the current setpoint for the specified channel
        
        Args:
            channel (int): Channel number (1-6)
            
        Returns:
            tuple: (success, setpoint_or_error_message)
        """
        if not 1 <= channel <= 6:
            return False, "Invalid channel number"
        
        # Query current setpoint
        cmd = f"@{CONTROLLER_ADDRESS}QSP{channel}?;FF\r"  # Query Setpoint
        response = self.send_command(cmd)
        
        if response:
            if "ACK" in response:
                # Extract setpoint from response
                setpoint_str = re.sub(r'^@\d+ACK', '', response).replace(";FF", "").strip()
                try:
                    # Convert scientific notation to float
                    setpoint = float(setpoint_str)
                    return True, setpoint
                except ValueError:
                    return False, f"Could not parse setpoint: {setpoint_str}"
            else:
                return False, f"Unexpected response: {response}"
        else:
            return False, "No response received"

    def supports_flow_control(self, channel):
        """Check if the channel supports flow control
        
        Args:
            channel (int): Channel number (1-6)
            
        Returns:
            bool: True if flow control is supported, False if not
        """
        if not 1 <= channel <= 6:
            return False
            
        # Special case for all channels with older firmware
        probe = self.probe_mfc_channel(channel)
        return probe['is_mfc']
        
        # The following code is disabled to treat all channels as legacy
        '''
        # Check if flow control commands are supported
        cmd = f"@{CONTROLLER_ADDRESS}QFE{channel}?;FF\r"
        response = self.send_command(cmd)
        
        if not response:
            return False
            
        # NAK response means flow control not supported
        if "NAK" in response:
            return False
            
        # ACK response means flow control is supported
        if "ACK" in response:
            return True
            
        return False
        '''


class SerialWorker(QObject):
    data = pyqtSignal(dict)
    connection_status = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.is_running = True
        self.controller = None
        self.sampling_rate = 0.5
        self.pressure_readings = ["---"] * 6
        self.poll_in_progress = False
        self._stop_event = threading.Event()
        self._serial_lock = threading.Lock()
        self._poll_thread = None
        self.polling_enabled = False
    
    def connect_device(self, port=None):
        try:
            self.connection_status.emit("connecting")
            if not setup_permissions():
                self.connection_status.emit("permission_error")
                return False

            selected_port = port or DEFAULT_PORT
            self.controller = MFCController(port=selected_port, timeout=0.2)
            MFCController._instance = self.controller
            self._stop_event.clear()
            self.connection_status.emit("connected")
            return True
        except Exception as e:
            print(f"Error connecting to device: {e}")
            self.connection_status.emit("disconnected")
            return False

    def _poll_loop(self):
        while not self._stop_event.is_set():
            if self.polling_enabled:
                self.read_data()
            self._stop_event.wait(self.sampling_rate)

    def start_polling(self):
        if not self.controller:
            return
        self.polling_enabled = True
        if not self._poll_thread or not self._poll_thread.is_alive():
            self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._poll_thread.start()

    def stop_polling(self):
        self.polling_enabled = False
    
    def read_data(self):
        if not self.controller or self.poll_in_progress:
            return
            
        self.poll_in_progress = True
        try:
            # Poll only pressure readings here to keep the UI responsive.
            with self._serial_lock:
                self.pressure_readings = self.controller.read_all_pressures(verbose=False)
            flow_data = {'timestamp': time.time()}
            self.data.emit(flow_data)
        except Exception as e:
            print(f"Error reading data: {e}")
        finally:
            self.poll_in_progress = False
    
    def send_command(self, channel, flow_rate):
        if not self.controller:
            return False
            
        try:
            self.controller.set_flow_rate(channel, flow_rate)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def stop(self):
        self.is_running = False
        self.polling_enabled = False
        self.poll_in_progress = False
        self._stop_event.set()
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        if self.controller:
            if self._serial_lock.acquire(timeout=0.2):
                try:
                    self.controller.close()
                finally:
                    self._serial_lock.release()
            else:
                print("Warning: Timed out waiting for serial lock during disconnect")
            self.controller = None
    
    def zero_channel(self, channel):
        """Zero the specified channel"""
        if not self.controller:
            return False, "Not connected to controller"
            
        try:
            was_polling = self.polling_enabled
            self.stop_polling()
            if self.poll_in_progress:
                deadline = time.monotonic() + 0.5
                while self.poll_in_progress and time.monotonic() < deadline:
                    time.sleep(0.01)
            if not self._serial_lock.acquire(timeout=0.5):
                return False, "Timed out waiting for serial device"
            try:
                return self.controller.zero_channel(channel)
            finally:
                self._serial_lock.release()
        except Exception as e:
            print(f"Error zeroing channel: {e}")
            return False, f"Error: {e}"
        finally:
            if was_polling and self.controller:
                self.start_polling()
    
    def set_flow_point(self, channel, flow_value):
        """Set flow point for the specified channel"""
        if not self.controller:
            return False, "Not connected to controller"
            
        try:
            was_polling = self.polling_enabled
            self.stop_polling()
            if self.poll_in_progress:
                deadline = time.monotonic() + 0.5
                while self.poll_in_progress and time.monotonic() < deadline:
                    time.sleep(0.01)
            if not self._serial_lock.acquire(timeout=0.5):
                return False, "Timed out waiting for serial device"
            try:
                return self.controller.set_flow_point(channel, flow_value)
            finally:
                self._serial_lock.release()
        except Exception as e:
            print(f"Error setting flow point: {e}")
            return False, f"Error: {e}"
        finally:
            if was_polling and self.controller:
                self.start_polling()

    def get_device_type(self, channel):
        """Get the type of device connected to the specified channel"""
        if not self.controller:
            return False, "Not connected to controller"
            
        try:
            with self._serial_lock:
                return self.controller.get_device_type(channel)
        except Exception as e:
            print(f"Error getting device type: {e}")
            return False, f"Error: {e}"

    def get_setpoint(self, channel):
        """Get the current setpoint for the specified channel"""
        if not self.controller:
            return False, "Not connected to controller"
            
        try:
            with self._serial_lock:
                return self.controller.get_setpoint(channel)
        except Exception as e:
            print(f"Error getting setpoint: {e}")
            return False, f"Error: {e}"

    def supports_flow_control(self, channel):
        """Check if the channel supports flow control"""
        if not self.controller:
            return False
            
        try:
            with self._serial_lock:
                return self.controller.supports_flow_control(channel)
        except Exception as e:
            print(f"Error checking flow control support: {e}")
            return False


class ColorSquare(QFrame):
    def __init__(self, color):
        super().__init__()
        self.setFixedSize(15, 15)
        self.setStyleSheet(f"background-color: {color}; border: 1px solid black;")


class RecipeDialog(QDialog):
    """Dialog for creating and editing vapor mixture recipes.

    Users specify a total flow rate and vapor percentages.  The system
    auto-calculates dilution channels so that each vapor+dilution pair
    sums to the total flow.
    """

    VAPOR_CHANNELS = {'Ch5': 'Vapor1', 'Ch4': 'Vapor2'}
    DILUTION_MAP = {'Ch5': 'Ch6', 'Ch4': 'Ch3'}

    def __init__(self, parent=None, initial_values=None):
        super().__init__(parent)
        self.setWindowTitle("Create Mixture Recipe")
        self.setMinimumSize(500, 480)
        self._main_window = parent

        layout = QVBoxLayout(self)

        # --- Input group ---
        input_group = QGroupBox("Mixture Parameters")
        input_layout = QGridLayout(input_group)

        input_layout.addWidget(QLabel("Total Flow (SCCM):"), 0, 0)
        self.total_flow_spin = QDoubleSpinBox()
        self.total_flow_spin.setRange(0, 200)
        self.total_flow_spin.setDecimals(1)
        self.total_flow_spin.setSingleStep(1.0)
        self.total_flow_spin.setValue(100.0)
        self.total_flow_spin.valueChanged.connect(self._recalculate)
        input_layout.addWidget(self.total_flow_spin, 0, 1)

        input_layout.addWidget(QLabel("Vapor1 Ratio (%):"), 1, 0)
        self.vapor1_spin = QDoubleSpinBox()
        self.vapor1_spin.setRange(0, 100)
        self.vapor1_spin.setDecimals(1)
        self.vapor1_spin.setSingleStep(1.0)
        self.vapor1_spin.setValue(50.0)
        self.vapor1_spin.valueChanged.connect(self._recalculate)
        input_layout.addWidget(self.vapor1_spin, 1, 1)

        input_layout.addWidget(QLabel("Vapor2 Ratio (%):"), 2, 0)
        self.vapor2_spin = QDoubleSpinBox()
        self.vapor2_spin.setRange(0, 100)
        self.vapor2_spin.setDecimals(1)
        self.vapor2_spin.setSingleStep(1.0)
        self.vapor2_spin.setValue(50.0)
        self.vapor2_spin.valueChanged.connect(self._recalculate)
        input_layout.addWidget(self.vapor2_spin, 2, 1)

        # Optional overrides for Mixture (Ch1) and Ch2
        input_layout.addWidget(QLabel(f"{CHANNEL_NAMES['Ch1']} (Ch1) Override:"), 3, 0)
        self.ch1_spin = QDoubleSpinBox()
        self.ch1_spin.setRange(0, CHANNEL_MAX_SCCM['Ch1'])
        self.ch1_spin.setDecimals(1)
        self.ch1_spin.setSuffix(" SCCM")
        self.ch1_spin.setValue(0.0)
        self.ch1_spin.valueChanged.connect(self._recalculate)
        input_layout.addWidget(self.ch1_spin, 3, 1)

        input_layout.addWidget(QLabel(f"{CHANNEL_NAMES['Ch2']} (Ch2) Override:"), 4, 0)
        self.ch2_spin = QDoubleSpinBox()
        self.ch2_spin.setRange(0, CHANNEL_MAX_SCCM['Ch2'])
        self.ch2_spin.setDecimals(1)
        self.ch2_spin.setSuffix(" SCCM")
        self.ch2_spin.setValue(0.0)
        self.ch2_spin.valueChanged.connect(self._recalculate)
        input_layout.addWidget(self.ch2_spin, 4, 1)

        layout.addWidget(input_group)

        # --- Preview table ---
        preview_group = QGroupBox("Computed Channel Values")
        preview_layout = QVBoxLayout(preview_group)

        self.preview_table = QTableWidget(6, 3)
        self.preview_table.setHorizontalHeaderLabels(["Channel", "Name", "SCCM"])
        self.preview_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.preview_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.preview_table.verticalHeader().setVisible(False)
        preview_layout.addWidget(self.preview_table)

        self.warning_label = QLabel("")
        self.warning_label.setStyleSheet("color: red; font-weight: bold;")
        self.warning_label.setWordWrap(True)
        preview_layout.addWidget(self.warning_label)

        layout.addWidget(preview_group)

        # --- Buttons ---
        button_layout = QHBoxLayout()

        apply_button = QPushButton("Apply to Hardware")
        apply_button.clicked.connect(self._apply)
        button_layout.addWidget(apply_button)

        save_button = QPushButton("Save Recipe")
        save_button.clicked.connect(self._save_csv)
        button_layout.addWidget(save_button)

        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)

        if initial_values:
            self._load_initial(initial_values)

        self._recalculate()

    # ------------------------------------------------------------------
    def _load_initial(self, values):
        """Populate the dialog from a previously saved recipe dict."""
        self.total_flow_spin.setValue(values.get('total_flow', 100.0))
        self.vapor1_spin.setValue(values.get('vapor1_pct', 50.0))
        self.vapor2_spin.setValue(values.get('vapor2_pct', 50.0))
        self.ch1_spin.setValue(values.get('Ch1', 0.0))
        self.ch2_spin.setValue(values.get('Ch2', 0.0))

    # ------------------------------------------------------------------
    def compute_channels(self):
        """Return a dict of channel_key -> SCCM based on current inputs."""
        total = self.total_flow_spin.value()
        v1_pct = self.vapor1_spin.value()
        v2_pct = self.vapor2_spin.value()

        vapor1 = total * (v1_pct / 100.0)
        vapor2 = total * (v2_pct / 100.0)
        dilution1 = total - vapor1
        dilution2 = total - vapor2

        return {
            'Ch1': self.ch1_spin.value(),
            'Ch2': self.ch2_spin.value(),
            'Ch3': round(dilution2, 1),
            'Ch4': round(vapor2, 1),
            'Ch5': round(vapor1, 1),
            'Ch6': round(dilution1, 1),
        }

    # ------------------------------------------------------------------
    def _recalculate(self):
        """Recompute channel values, refresh the preview table, validate."""
        values = self.compute_channels()
        warnings = []

        for row, ch_key in enumerate(CHANNEL_KEYS):
            sccm = values[ch_key]
            self.preview_table.setItem(row, 0, QTableWidgetItem(ch_key))
            self.preview_table.setItem(row, 1, QTableWidgetItem(CHANNEL_NAMES[ch_key]))
            item = QTableWidgetItem(f"{sccm:.1f}")
            item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

            ch_max = CHANNEL_MAX_SCCM[ch_key]
            if sccm < 0:
                item.setBackground(QColor(255, 180, 180))
                warnings.append(f"{CHANNEL_NAMES[ch_key]} is negative ({sccm:.1f})")
            elif sccm > ch_max:
                item.setBackground(QColor(255, 220, 150))
                warnings.append(f"{CHANNEL_NAMES[ch_key]} exceeds max {ch_max} SCCM ({sccm:.1f})")
            self.preview_table.setItem(row, 2, item)

        self.warning_label.setText("\n".join(warnings) if warnings else "All values within limits.")
        if not warnings:
            self.warning_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.warning_label.setStyleSheet("color: red; font-weight: bold;")

    # ------------------------------------------------------------------
    def _validate(self):
        """Return True if all channel values are within limits."""
        values = self.compute_channels()
        for ch_key, sccm in values.items():
            ch_max = CHANNEL_MAX_SCCM[ch_key]
            if sccm < 0 or sccm > ch_max:
                return False
        return True

    # ------------------------------------------------------------------
    def _apply(self):
        """Send computed values to the hardware via the parent MainWindow."""
        if not self._validate():
            QMessageBox.warning(self, "Validation Error",
                                "One or more channels are out of range. Fix the values before applying.")
            return

        if not self._main_window or not self._main_window.serial_worker:
            QMessageBox.warning(self, "Not Connected",
                                "Please connect to the device before applying a recipe.")
            return

        values = self.compute_channels()
        errors = []
        for ch_key in CHANNEL_KEYS:
            sccm = values[ch_key]
            self._main_window.flow_controls[ch_key]['spinbox'].setValue(sccm)
            channel_num = int(ch_key[2:])
            success, message = self._main_window.serial_worker.set_flow_point(channel_num, sccm)
            if not success:
                errors.append(f"{CHANNEL_NAMES[ch_key]}: {message}")

        if errors:
            QMessageBox.warning(self, "Partial Failure",
                                "Some channels failed to set:\n" + "\n".join(errors))
        else:
            QMessageBox.information(self, "Recipe Applied",
                                    "All channel flow rates have been set successfully.")
        self.accept()

    # ------------------------------------------------------------------
    def _save_csv(self):
        """Save the current recipe to a CSV file."""
        if not self._validate():
            QMessageBox.warning(self, "Validation Error",
                                "One or more channels are out of range. Fix the values before saving.")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Recipe", "recipe.csv", "CSV Files (*.csv)")
        if not path:
            return

        values = self.compute_channels()
        total = self.total_flow_spin.value()
        v1_pct = self.vapor1_spin.value()
        v2_pct = self.vapor2_spin.value()

        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["channel", "name", "sccm", "percentage"])
                writer.writerow(["_meta", "total_flow", total, ""])
                writer.writerow(["_meta", "vapor1_pct", v1_pct, ""])
                writer.writerow(["_meta", "vapor2_pct", v2_pct, ""])
                for ch_key in CHANNEL_KEYS:
                    pct = ""
                    if ch_key == 'Ch5':
                        pct = v1_pct
                    elif ch_key == 'Ch4':
                        pct = v2_pct
                    writer.writerow([ch_key, CHANNEL_NAMES[ch_key], values[ch_key], pct])
            QMessageBox.information(self, "Saved", f"Recipe saved to {path}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to save recipe: {e}")

    # ------------------------------------------------------------------
    @staticmethod
    def load_csv(path):
        """Parse a recipe CSV and return a dict suitable for initial_values."""
        result = {}
        try:
            with open(path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row['channel'] == '_meta':
                        if row['name'] == 'total_flow':
                            result['total_flow'] = float(row['sccm'])
                        elif row['name'] == 'vapor1_pct':
                            result['vapor1_pct'] = float(row['sccm'])
                        elif row['name'] == 'vapor2_pct':
                            result['vapor2_pct'] = float(row['sccm'])
                    else:
                        result[row['channel']] = float(row['sccm'])
        except Exception as e:
            print(f"Error loading recipe CSV: {e}")
            return None
        return result


class MainWindow(QMainWindow):
    flow_command_finished = pyqtSignal(str, bool, str, float)
    disconnect_finished = pyqtSignal()
    zero_command_finished = pyqtSignal(str, bool, str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Precision Olfactometer")
        self.setGeometry(100, 100, 1200, 600)
        
        self.setWindowIcon(QIcon('icon.png'))
        self.setMinimumSize(800, 500)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        graphs_widget = QWidget()
        graphs_layout = QVBoxLayout(graphs_widget)
        graphs_layout.setSpacing(5)
        graphs_layout.setContentsMargins(5, 5, 5, 5)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', 'Volumetric Flow Rate (SCCM)')
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.plot_widget.setYRange(-10, 100)
        graphs_layout.addWidget(self.plot_widget)
        
        legend = self.plot_widget.addLegend()
        
        main_layout.addWidget(graphs_widget, stretch=1)
        
        self.curves = {}
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        
        for i, (channel_key, color) in enumerate(zip(CHANNEL_KEYS, colors)):
            channel_name = CHANNEL_NAMES[channel_key]
            curve = self.plot_widget.plot(pen=color, name=channel_name)
            self.curves[channel_key] = curve
        
        right_column_widget = QWidget()
        right_column_widget.setFixedWidth(300)
        right_column_layout = QVBoxLayout(right_column_widget)
        right_column_layout.setSpacing(5)
        right_column_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.addWidget(right_column_widget, stretch=0)
        
        connection_widget = QWidget()
        connection_layout = QHBoxLayout(connection_widget)
        connection_layout.setSpacing(5)
        connection_layout.setContentsMargins(0, 0, 0, 0)
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_device)
        
        self.scan_button = QPushButton("Scan Ports")
        self.scan_button.clicked.connect(self.scan_ports)
        self.scan_button.setToolTip("Scan for ports (requires admin password)")
        
        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self.port_combo, 1)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.scan_button)
        
        right_column_layout.addWidget(connection_widget)
        
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 5px;")
        right_column_layout.addWidget(self.status_label)
        
        self.flow_controls = {}
        channel_colors = ['#FF0000', '#00FF00', '#0000FF', '#00FFFF', '#FF00FF', '#FFFF00']
        
        for i, (channel_key, color) in enumerate(zip(CHANNEL_KEYS, channel_colors)):
            flow_widget = QWidget()
            flow_layout = QHBoxLayout(flow_widget)
            flow_layout.setSpacing(6)
            flow_layout.setContentsMargins(0, 2, 0, 2)
            flow_widget.setMinimumHeight(36)
            
            color_square = ColorSquare(color)
            flow_layout.addWidget(color_square)
            
            # Use custom display name from CHANNEL_NAMES dictionary
            channel_label = QLabel(CHANNEL_NAMES[channel_key])
            channel_label.setMinimumWidth(80)
            channel_label.setMinimumHeight(28)
            flow_layout.addWidget(channel_label)
            
            flow_spinbox = QDoubleSpinBox()
            flow_spinbox.setRange(-1000, 1000)  # Changed range to accommodate pressure values
            flow_spinbox.setDecimals(1)
            flow_spinbox.setSuffix(" SCCM")
            flow_spinbox.setSingleStep(0.1)
            flow_spinbox.setMinimumHeight(28)
            flow_layout.addWidget(flow_spinbox, 1)

            readback_label = QLabel("PV: ---")
            readback_label.setFixedWidth(110)
            readback_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            flow_layout.addWidget(readback_label)
            
            set_button = QPushButton("Set")
            set_button.setFixedWidth(48)
            set_button.setMinimumHeight(28)
            set_button.clicked.connect(lambda _, ch=channel_key: self.set_flow_rate(ch))
            flow_layout.addWidget(set_button)
            
            right_column_layout.addWidget(flow_widget)
            
            self.flow_controls[channel_key] = {
                'spinbox': flow_spinbox,
                'readback_label': readback_label,
                'button': set_button,
                'layout': flow_layout  # Store layout reference for adding monitor label
            }
        
        recipe_label = QLabel("Recipe Controls")
        recipe_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_column_layout.addWidget(recipe_label)
        
        create_mixture_button = QPushButton("Create Mixture")
        create_mixture_button.clicked.connect(self.create_mixture)
        create_mixture_button.setToolTip("Open the mixture recipe dialog to set vapor ratios")
        right_column_layout.addWidget(create_mixture_button)
        
        load_recipe_button = QPushButton("Load Recipe")
        load_recipe_button.clicked.connect(self.load_recipe)
        load_recipe_button.setToolTip("Load a recipe from a CSV file")
        right_column_layout.addWidget(load_recipe_button)
        
        save_recipe_button = QPushButton("Save Recipe")
        save_recipe_button.clicked.connect(self.save_recipe)
        save_recipe_button.setToolTip("Save current channel values to a CSV file")
        right_column_layout.addWidget(save_recipe_button)
        
        right_column_layout.addWidget(QLabel("Units are in SCCM (Standard Cubic Centimeters per Minute)"))
        
        # Add Zero Channel UI
        zero_widget = QWidget()
        zero_layout = QHBoxLayout(zero_widget)
        zero_layout.setSpacing(5)
        zero_layout.setContentsMargins(0, 0, 0, 0)
        
        zero_label = QLabel("Zero Channel:")
        zero_layout.addWidget(zero_label)
        
        self.channel_combo = QComboBox()
        for i in range(1, 7):
            channel_key = f"Ch{i}"
            self.channel_combo.addItem(CHANNEL_NAMES[channel_key])
        zero_layout.addWidget(self.channel_combo)
        
        self.zero_button = QPushButton("Zero")
        self.zero_button.clicked.connect(self.zero_channel)
        self.zero_button.setEnabled(False)  # Initially disabled until connected
        zero_layout.addWidget(self.zero_button)
        
        right_column_layout.addWidget(zero_widget)
        
        # Add Identify MFC Channels button
        identify_mfc_button = QPushButton("Identify MFC Channels")
        identify_mfc_button.clicked.connect(self.show_all_mfc_channels)
        identify_mfc_button.setToolTip("Identify which channels have MFC hardware installed")
        identify_mfc_button.setEnabled(False)  # Initially disabled until connected
        self.identify_mfc_button = identify_mfc_button  # Store reference to enable/disable
        right_column_layout.addWidget(identify_mfc_button)
        
        # Add a diagnostic button
        diagnose_button = QPushButton("Diagnose Channel")
        diagnose_button.clicked.connect(self.diagnose_channel)
        diagnose_button.setToolTip("Run diagnostics to troubleshoot channel issues")
        right_column_layout.addWidget(diagnose_button)
        
        # Add a monitor flow response button
        monitor_button = QPushButton("Monitor Flow Response")
        monitor_button.clicked.connect(self.monitor_flow_response)
        monitor_button.setToolTip("Monitor how flow responds to setpoint changes")
        right_column_layout.addWidget(monitor_button)
        
        recording_widget = QWidget()
        recording_layout = QHBoxLayout(recording_widget)
        recording_layout.setSpacing(6)
        recording_layout.setContentsMargins(0, 2, 0, 2)
        recording_widget.setMinimumHeight(36)
        
        self.record_button = QPushButton("Start Recording")
        self.record_button.setMinimumHeight(28)
        self.record_button.clicked.connect(self.toggle_recording)
        recording_layout.addWidget(self.record_button)
        
        self.recording_duration = QSpinBox()
        self.recording_duration.setRange(10, 3600)
        self.recording_duration.setValue(60)
        self.recording_duration.setSuffix(" s")
        self.recording_duration.setMinimumHeight(28)
        recording_layout.addWidget(self.recording_duration)
        
        right_column_layout.addWidget(recording_widget)
        
        self.recording_status = QLabel("Not Recording")
        self.recording_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.recording_status.setStyleSheet("font-weight: bold;")
        right_column_layout.addWidget(self.recording_status)
        
        right_column_layout.addStretch(1)
        
        self.plot_data = {
            channel_key: {'time': deque(maxlen=MAX_DATA_POINTS), 'value': deque(maxlen=MAX_DATA_POINTS)} 
            for channel_key in CHANNEL_KEYS
        }
        
        self.serial_worker = SerialWorker()
        self.serial_worker.data.connect(self.update_plot)
        self.serial_worker.connection_status.connect(self.update_status)
        self.flow_command_finished.connect(self._handle_flow_command_finished)
        self.disconnect_finished.connect(self._handle_disconnect_finished)
        self.zero_command_finished.connect(self._handle_zero_command_finished)
        
        self.is_recording = False
        self.recording_start_time = None
        self.recording_timer = QTimer()
        self.recording_timer.timeout.connect(self.update_recording)
        
        self.start_time = None
        
        # Leave connection manual so startup stays responsive even if the controller stalls.
    
    def refresh_ports(self):
        self.port_combo.clear()
        available_ports = MFCController.list_available_ports()
        for port in available_ports:
            self.port_combo.addItem(port)
    
    def connect_device(self):
        selected_port = self.port_combo.currentText()
        
        MFCController._instance = None
        MFCController.find_serial_port = lambda: selected_port
        
        self.serial_worker.connect_device(port=selected_port)
        
    
    def update_status(self, status):
        if status == "connected":
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 5px;")
            self.connect_button.setText("Disconnect")
            self.connect_button.clicked.disconnect()
            self.connect_button.clicked.connect(self.disconnect_device)
            
            # Enable zero button when connected
            self.zero_button.setEnabled(True)
            
            # Enable identify button when connected
            self.identify_mfc_button.setEnabled(True)

            # In legacy firmware mode, treat all channels as controllable without probing them on connect.
            for channel in self.flow_controls:
                self.flow_controls[channel]['button'].setEnabled(True)
                self.flow_controls[channel]['spinbox'].setEnabled(True)
                self.flow_controls[channel]['button'].setText("Set")
                self.flow_controls[channel]['readback_label'].setText("PV: ---")
                if 'monitor_label' in self.flow_controls[channel]:
                    self.flow_controls[channel]['monitor_label'].hide()

            self.start_time = None
            self._load_current_setpoints()
            self.serial_worker.start_polling()
                
        elif status == "disconnected":
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 5px;")
            self.connect_button.setText("Connect")
            self.connect_button.clicked.disconnect()
            self.connect_button.clicked.connect(self.connect_device)
            
            for channel in self.flow_controls:
                self.flow_controls[channel]['button'].setEnabled(False)
                self.flow_controls[channel]['spinbox'].setEnabled(False)
                self.flow_controls[channel]['readback_label'].setText("PV: ---")
            
            # Disable buttons when disconnected
            self.zero_button.setEnabled(False)
            self.identify_mfc_button.setEnabled(False)
            self.start_time = None
                
        elif status == "connecting":
            self.status_label.setText("Status: Connecting...")
            self.status_label.setStyleSheet("background-color: orange; color: white; font-weight: bold; padding: 5px;")
            
        elif status == "permission_error":
            self.status_label.setText("Permission Error")
            self.status_label.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 5px;")
            QMessageBox.warning(self, "Permission Error", "Please restart the application after logging back in.")

    def _load_current_setpoints(self):
        """Sync the UI setpoint controls from the controller's current values."""
        if not self.serial_worker or not self.serial_worker.controller:
            return

        for channel_key in CHANNEL_KEYS:
            channel_num = int(channel_key[2:])
            success, setpoint = self.serial_worker.get_setpoint(channel_num)
            if success and isinstance(setpoint, (int, float)):
                spinbox = self.flow_controls[channel_key]['spinbox']
                spinbox.blockSignals(True)
                spinbox.setValue(setpoint)
                spinbox.blockSignals(False)
    
    def disconnect_device(self):
        self.connect_button.setEnabled(False)
        self.connect_button.setText("Disconnecting...")
        threading.Thread(target=self._disconnect_worker, daemon=True).start()
    
    def update_plot(self, data):
        if self.start_time is None:
            self.start_time = data.get('timestamp', time.time())
        
        current_time = data.get('timestamp', time.time()) - self.start_time
        
        # Get pressure readings from worker
        if hasattr(self.serial_worker, 'pressure_readings'):
            for i, channel_key in enumerate(CHANNEL_KEYS):
                if i < len(self.serial_worker.pressure_readings):
                    value_str = self.serial_worker.pressure_readings[i]
                    self.flow_controls[channel_key]['readback_label'].setText(f"PV: {value_str}")
                    if value_str != '---':
                        try:
                            value = float(value_str)
                            if channel_key in self.plot_data:
                                self.plot_data[channel_key]['time'].append(current_time)
                                self.plot_data[channel_key]['value'].append(value)
                        except ValueError:
                            # Skip invalid values
                            pass
        
        try:
            x_min = max(0, current_time - TIME_WINDOW)
            x_max = max(TIME_WINDOW, current_time)
            
            for channel_key in self.curves:
                if channel_key in self.plot_data and len(self.plot_data[channel_key]['time']) > 0:
                    x_data = list(self.plot_data[channel_key]['time'])
                    y_data = list(self.plot_data[channel_key]['value'])
                    
                    if len(x_data) == len(y_data):
                        self.curves[channel_key].setData(x_data, y_data)
            
            self.plot_widget.setXRange(x_min, x_max)
            visible_values = []
            for channel_key in CHANNEL_KEYS:
                times = self.plot_data[channel_key]['time']
                values = self.plot_data[channel_key]['value']
                for x, y in zip(times, values):
                    if x >= x_min:
                        visible_values.append(y)

            if visible_values:
                y_min = min(visible_values)
                y_max = max(visible_values)
                margin = max((y_max - y_min) * 0.1, 1.0)
                self.plot_widget.setYRange(y_min - margin, y_max + margin)
                        
        except Exception as e:
            print(f"Error updating plots: {e}")
    
    def set_flow_rate(self, channel):
        if not self.serial_worker:
            return
            
        flow_rate = self.flow_controls[channel]['spinbox'].value()
        button = self.flow_controls[channel]['button']
        button.setEnabled(False)
        button.setStyleSheet("background-color: orange;")
        
        channel_num = int(channel[2:])  # Extract number from "Ch1", "Ch2", etc.
        threading.Thread(
            target=self._set_flow_rate_worker,
            args=(channel, channel_num, flow_rate),
            daemon=True
        ).start()

    def _set_flow_rate_worker(self, channel, channel_num, flow_rate):
        success, message = self.serial_worker.set_flow_point(channel_num, flow_rate)
        self.flow_command_finished.emit(channel, success, message, flow_rate)

    def _handle_flow_command_finished(self, channel, success, message, flow_rate):
        button = self.flow_controls[channel]['button']
        button.setEnabled(True)
        if success:
            button.setStyleSheet("background-color: green;")
            QTimer.singleShot(500, lambda: button.setStyleSheet(""))
            print(f"Set {channel} to {flow_rate} SCCM")
        else:
            button.setStyleSheet("background-color: red;")
            QTimer.singleShot(500, lambda: button.setStyleSheet(""))
            print(f"Failed to set {channel}: {message}")
            QMessageBox.warning(self, "Setting Failed", f"Failed to set flow rate for {channel}: {message}")

    def _disconnect_worker(self):
        self.serial_worker.stop()
        self.disconnect_finished.emit()

    def _handle_disconnect_finished(self):
        self.connect_button.setEnabled(True)
        self.update_status("disconnected")
    
    def create_mixture(self):
        """Open the RecipeDialog to create a new mixture recipe."""
        dialog = RecipeDialog(parent=self)
        dialog.exec()

    def load_recipe(self):
        """Load a recipe CSV and open the RecipeDialog pre-filled with its values."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Recipe", "", "CSV Files (*.csv);;All Files (*)")
        if not path:
            return

        values = RecipeDialog.load_csv(path)
        if values is None:
            QMessageBox.warning(self, "Error", "Failed to parse the selected recipe file.")
            return

        dialog = RecipeDialog(parent=self, initial_values=values)
        dialog.exec()

    def save_recipe(self):
        """Save the current channel spinbox values to a CSV file."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Recipe", "recipe.csv", "CSV Files (*.csv)")
        if not path:
            return

        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["channel", "name", "sccm", "percentage"])
                for ch_key in CHANNEL_KEYS:
                    sccm = self.flow_controls[ch_key]['spinbox'].value()
                    writer.writerow([ch_key, CHANNEL_NAMES[ch_key], sccm, ""])
            QMessageBox.information(self, "Recipe Saved", f"Current settings saved to {path}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to save recipe: {e}")
    
    def toggle_recording(self):
        if not self.is_recording:
            self.start_recording()
        else:
            self.stop_recording()
    
    def start_recording(self):
        try:
            self.recording_start_time = datetime.now()
            timestamp_str = self.recording_start_time.strftime('%m-%d-%Y-%I%M%p').lower()
            
            os.makedirs('recordings', exist_ok=True)
            
            self.recording_file = f"recordings/flow_recording_{timestamp_str}.csv"
            
            with open(self.recording_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                headers = ['Time(s)'] + list(self.plot_data.keys())
                writer.writerow(headers)
            
            self.is_recording = True
            self.record_button.setText("Stop Recording")
            self.recording_status.setText("Recording...")
            self.recording_status.setStyleSheet("color: red; font-weight: bold;")
            
            self.recording_elapsed = 0
            self.recording_timer.start(1000)
            
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to start recording: {e}")
    
    def update_recording(self):
        if not self.is_recording:
            return
            
        self.recording_elapsed += 1
        self.recording_status.setText(f"Recording: {self.recording_elapsed}s")
        
        try:
            current_data = {}
            for channel in self.plot_data:
                if self.plot_data[channel]['value']:
                    current_data[channel] = self.plot_data[channel]['value'][-1]
                else:
                    current_data[channel] = 0
            
            with open(self.recording_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                row = [self.recording_elapsed] + [current_data.get(channel, 0) for channel in self.plot_data]
                writer.writerow(row)
            
            if self.recording_elapsed >= self.recording_duration.value():
                self.stop_recording()
                
        except Exception as e:
            print(f"Error updating recording: {e}")
    
    def stop_recording(self):
        if not self.is_recording:
            return
            
        self.recording_timer.stop()
        self.is_recording = False
        self.record_button.setText("Start Recording")
        self.recording_status.setText(f"Recorded {self.recording_elapsed}s")
        self.recording_status.setStyleSheet("color: green; font-weight: bold;")
        
        QMessageBox.information(self, "Recording Complete", 
                              f"Recording saved to {self.recording_file}")
    
    def closeEvent(self, event):
        if self.is_recording:
            self.stop_recording()
            
        if hasattr(self, 'serial_worker'):
            self.serial_worker.stop()
            
        event.accept()

    def auto_connect(self):
        """Auto-connect to the default serial port without scanning"""
        port = DEFAULT_PORT
        
        # Set the port in the combo box if it exists
        for i in range(self.port_combo.count()):
            if self.port_combo.itemText(i) == port:
                self.port_combo.setCurrentIndex(i)
                break
        
        # Force the controller to use ttyS4
        MFCController._instance = None
        
        # Connect to the device
        self.serial_worker.connect_device(port=port)
        
    def scan_ports(self):
        """Scan for all available ports using the find_serial_port method which requires admin access"""
        # Reset the controller instance to use the dynamic detection
        MFCController._instance = None
        
        # This will trigger the find_serial_port method which uses lspci and dmesg
        port = MFCController.find_serial_port()
        
        # Refresh the port list
        self.refresh_ports()
        
        # Select the found port in the combo
        for i in range(self.port_combo.count()):
            if self.port_combo.itemText(i) == port:
                self.port_combo.setCurrentIndex(i)
                break
                
        QMessageBox.information(self, "Port Scan Complete", f"Found device on {port}")
        
        # Connect to the new port
        self.connect_device()

    def zero_channel(self):
        """Zero the selected channel"""
        if not self.serial_worker or self.status_label.text() == "Status: Disconnected":
            QMessageBox.warning(self, "Not Connected", "Please connect to the device first")
            return
            
        # Get the selected channel display name and convert to internal channel key if needed
        channel_display_name = self.channel_combo.currentText()
        if channel_display_name in DISPLAY_TO_CHANNEL:
            channel_key = DISPLAY_TO_CHANNEL[channel_display_name]
            channel = int(channel_key[2:])  # Extract number from "Ch1", "Ch2", etc.
        else:
            # For backward compatibility with direct channel numbers
            try:
                channel = int(channel_display_name[2:])
            except (ValueError, IndexError):
                QMessageBox.warning(self, "Invalid Channel", f"Cannot parse channel from {channel_display_name}")
                return
        
        # Disable button and show it's working
        self.zero_button.setEnabled(False)
        self.zero_button.setText("Zeroing...")
        threading.Thread(
            target=self._zero_channel_worker,
            args=(channel, channel_display_name),
            daemon=True
        ).start()

    def _zero_channel_worker(self, channel, channel_display_name):
        success, message = self.serial_worker.zero_channel(channel)
        self.zero_command_finished.emit(channel_display_name, success, message)

    def _handle_zero_command_finished(self, channel_display_name, success, message):
        self.zero_button.setEnabled(True)
        self.zero_button.setText("Zero")
        if success:
            QMessageBox.information(self, "Success", f"Channel {channel_display_name} zeroed successfully")
            print(f"Channel {channel_display_name} zeroed successfully")
        else:
            QMessageBox.warning(self, "Zero Failed", f"Failed to zero channel {channel_display_name}: {message}")
            print(f"Failed to zero channel {channel_display_name}: {message}")

    def diagnose_channel(self):
        """Diagnose a channel to troubleshoot issues with zeroing or setting flow points"""
        if not self.serial_worker:
            QMessageBox.warning(self, "Not Connected", "Please connect to the device first")
            return
        
        # Get the channel number from user
        channel_options = [CHANNEL_NAMES[key] for key in CHANNEL_KEYS]
        channel_display_name, ok = QInputDialog.getItem(
            self, "Select Channel", "Choose a channel to diagnose:",
            channel_options, 4, False
        )
        
        if not ok:
            return
        
        # Convert display name back to internal channel key and extract channel number
        if channel_display_name in DISPLAY_TO_CHANNEL:
            channel_key = DISPLAY_TO_CHANNEL[channel_display_name]
            channel = int(channel_key[2:])
        else:
            # For backward compatibility if someone adds a name not in the map
            try:
                channel = int(channel_display_name[2:])
                channel_key = f"Ch{channel}"
            except (ValueError, IndexError):
                QMessageBox.warning(self, "Invalid Channel", f"Cannot parse channel from {channel_display_name}")
                return
        
        # Disable UI while diagnosing
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        
        # Create diagnostic report
        report = f"Diagnostic Report for {channel_display_name}:\n"
        report += "-" * 50 + "\n\n"
        
        # Add slot mapping information
        slot_mapping = {
            1: "A1", 2: "A2",
            3: "B1", 4: "B2",
            5: "C1", 6: "C2"
        }
        slot_name = slot_mapping.get(channel, "Unknown")
        report += f"MKS 946 Slot Mapping: Channel {channel} corresponds to Slot {slot_name}\n\n"
        report += "IMPORTANT: If your MFC is physically installed in a different slot,\n"
        report += "you must use the corresponding channel number for that slot:\n"
        report += "- Slot A = channels 1 and 2 (A1 and A2)\n"
        report += "- Slot B = channels 3 and 4 (B1 and B2)\n"
        report += "- Slot C = channels 5 and 6 (C1 and C2)\n\n"
        
        # Check if this channel supports flow control
        supports_control = self.serial_worker.supports_flow_control(channel)
        if supports_control:
            report += "Channel Type: CONTROLLABLE (Supports flow control)\n"
        else:
            report += "Channel Type: MONITOR ONLY (Does not support flow control)\n"
            report += "Note: This channel can only measure flow, not control it.\n"
            report += "      The UI has been updated to mark this as a monitor-only channel.\n\n"
            report += "POSSIBLE CAUSES:\n"
            report += "1. No MFC hardware is physically installed in this slot\n"
            report += "2. There is a gauge board installed in this slot instead of an MFC board\n"
            report += "3. The MFC is installed in a different slot (use the corresponding channel number)\n"
            report += "4. Incorrect wiring or connection issues with the MFC hardware\n\n"
        
        # Get device type
        success, device_type = self.serial_worker.get_device_type(channel)
        if success:
            report += f"Device Type: {device_type}\n"
            # Add interpretation of device type
            if "MFC" in str(device_type) or device_type in ["179", "1179"]:
                report += "This appears to be an MFC device type\n"
            else:
                report += "This does NOT appear to be an MFC device type\n"
        else:
            report += f"Failed to get device type: {device_type}\n"
        
        # 2. Check current setpoint
        success, setpoint = self.serial_worker.get_setpoint(channel)
        if success:
            report += f"Current Setpoint: {setpoint} SCCM\n"
        else:
            report += f"Failed to get setpoint: {setpoint}\n"
        
        # 3. Check the current MFC mode
        cmd = f"@{CONTROLLER_ADDRESS}QMD{channel}?;FF\r"
        response = self.serial_worker.controller.send_command(cmd)
        if response:
            report += f"MFC Mode Query Response: {response}\n"
            if "NAK" in response:
                report += "Status: MFC mode query not supported for this channel\n"
                report += "This indicates the 946 may not detect an MFC on this channel.\n"
                report += "Check the physical installation and slot mapping.\n"
            else:
                mode_match = re.search(r'@\d+ACK(.*?);FF', response)
                if mode_match:
                    report += f"Status: Current mode = {mode_match.group(1).strip()}\n"
        else:
            report += "No response to MFC mode query\n"
        
        # 4. Read current value
        response = self.serial_worker.controller.read_pressure(channel)
        if response:
            report += f"Current Reading: {response}\n"
            parsed = self.serial_worker.controller.parse_pressure_response(response)
            if parsed is not None:
                report += f"Parsed Value: {parsed} SCCM\n"
                
                # Compare with setpoint if available
                if success and isinstance(setpoint, (int, float)):
                    difference = parsed - setpoint
                    percentage = (parsed / setpoint * 100) if setpoint != 0 else float('inf')
                    report += f"Difference from Setpoint: {difference:.2f} SCCM ({percentage:.1f}%)\n"
                    
                    if abs(difference) > 5 and abs(percentage - 100) > 10:
                        report += "ISSUE: Flow is significantly different from setpoint.\n"
                        report += "This could indicate:\n"
                        report += "- Physical flow limitations\n"
                        report += "- Slow controller response\n"
                        report += "- External flow restrictions\n"
                        report += "- System needs calibration\n"
        else:
            report += "No reading available\n"
        
        # 5. Check valve control mode
        cmd = f"@{CONTROLLER_ADDRESS}QVM{channel}?;FF\r"  # Query Valve Mode
        response = self.serial_worker.controller.send_command(cmd)
        if response:
            report += f"Valve Mode Response: {response}\n"
            if "ACK" in response:
                mode_str = re.sub(r'^@\d+ACK', '', response).replace(";FF", "").strip()
                try:
                    mode = int(mode_str)
                    if mode == 0:
                        report += "Valve Mode: Normal (0)\n"
                    elif mode == 1:
                        report += "Valve Mode: Close (1)\n"
                    elif mode == 2:
                        report += "Valve Mode: Open (2)\n"
                    else:
                        report += f"Valve Mode: Unknown ({mode})\n"
                except ValueError:
                    report += f"Valve Mode: Unable to parse ({mode_str})\n"
        
        # Reset cursor
        QApplication.restoreOverrideCursor()
        
        # Show report
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Diagnostics for {channel_display_name}")
        dialog.resize(500, 400)
        
        layout = QVBoxLayout(dialog)
        
        text_edit = QTextEdit()
        text_edit.setReadOnly(True)
        text_edit.setPlainText(report)
        layout.addWidget(text_edit)
        
        button_box = QHBoxLayout()
        close_button = QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        button_box.addWidget(close_button)
        
        # Add a button to identify all MFC channels
        identify_button = QPushButton("Identify All MFC Channels")
        identify_button.clicked.connect(self.show_all_mfc_channels)
        button_box.addWidget(identify_button)
        
        layout.addLayout(button_box)
        
        dialog.exec()

    def show_all_mfc_channels(self):
        """Show a dialog with information about all MFC channels"""
        if not self.serial_worker or not self.serial_worker.controller:
            QMessageBox.warning(self, "Not Connected", "Please connect to the device first")
            return
            
        # Probe channels using documented 946 MFC commands
        mfc_channels = self.serial_worker.controller.probe_mfc_channels()
        
        # Create report
        report = "MFC CHANNEL PROBE\n"
        report += "=================\n\n"
        report += "Slot A = channels 1 and 2 (A1 and A2)\n"
        report += "Slot B = channels 3 and 4 (B1 and B2)\n"
        report += "Slot C = channels 5 and 6 (C1 and C2)\n\n"
        report += "A channel is considered responsive if the 946 replies to one or more native\n"
        report += "MFC commands: FR (flow read), QSP (setpoint query), or QMD (mode query).\n\n"
        report += "RESPONSIVE CHANNELS:\n"
        report += "--------------------\n"
        
        found_mfc = False
        for channel, info in mfc_channels.items():
            if info["is_mfc"]:
                found_mfc = True
                report += (
                    f"Channel {channel} (Slot {info['slot']}): RESPONDED on "
                    f"{', '.join(info['working_commands'])}\n"
                )
        
        if not found_mfc:
            report += "NO CHANNEL RESPONDED TO NATIVE 946 MFC COMMANDS.\n\n"
            report += "Possible causes:\n"
            report += "1. MFC is not properly connected to the 946 controller\n"
            report += "2. MFC module is not installed or configured in the 946 controller\n"
            report += "3. The selected serial port/settings are wrong\n"
            report += "4. The controller is in a different communication mode\n"
        
        report += "\n\nPER-CHANNEL DETAILS:\n"
        report += "--------------------\n"
        for channel, info in mfc_channels.items():
            report += f"Channel {channel} (Slot {info['slot']}):\n"
            for command_name in ['flow', 'setpoint', 'mode']:
                result = info['responses'][command_name]
                response_text = result['response'] if result['response'] else "NO RESPONSE"
                report += f"  {command_name.upper()} -> {response_text}\n"
        
        report += "\n\nIMPORTANT: When setting flow rates, you MUST use the channel\n"
        report += "number that corresponds to the slot where your MFC is installed.\n"
        report += "If your MFC is physically installed in slot B, you must use channels 3-4.\n"
        
        # Show dialog
        dialog = QDialog(self)
        dialog.setWindowTitle("MFC Channel Identification")
        dialog.resize(500, 400)
        
        layout = QVBoxLayout(dialog)
        
        text_edit = QTextEdit()
        text_edit.setReadOnly(True)
        text_edit.setPlainText(report)
        layout.addWidget(text_edit)
        
        button_layout = QHBoxLayout()
        
        close_button = QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        button_layout.addWidget(close_button)
        
        # Add a button to show troubleshooting guide
        troubleshoot_button = QPushButton("Troubleshooting Guide")
        troubleshoot_button.clicked.connect(self.show_troubleshooting_guide)
        button_layout.addWidget(troubleshoot_button)
        
        layout.addLayout(button_layout)
        
        dialog.exec()
        
    def monitor_flow_response(self):
        """Monitor how flow responds to setpoint changes over time"""
        if not self.serial_worker:
            QMessageBox.warning(self, "Not Connected", "Please connect to the device first")
            return
            
        # Get the channel number from user
        channel_options = [CHANNEL_NAMES[key] for key in CHANNEL_KEYS]
        channel_display_name, ok = QInputDialog.getItem(
            self, "Select Channel", "Choose a channel to monitor:",
            channel_options, 4, False
        )
        
        if not ok:
            return
            
        # Convert display name back to internal channel key and extract channel number
        if channel_display_name in DISPLAY_TO_CHANNEL:
            channel_key = DISPLAY_TO_CHANNEL[channel_display_name]
            channel = int(channel_key[2:])
        else:
            # For backward compatibility
            try:
                channel = int(channel_display_name[2:])
            except (ValueError, IndexError):
                QMessageBox.warning(self, "Invalid Channel", f"Cannot parse channel from {channel_display_name}")
                return
        
        # Get current values
        success, current_setpoint = self.serial_worker.get_setpoint(channel)
        if not success:
            QMessageBox.warning(self, "Error", f"Could not get current setpoint: {current_setpoint}")
            return
            
        # Get new setpoint from user
        new_setpoint, ok = QInputDialog.getDouble(
            self, "Enter New Setpoint", 
            f"Current setpoint: {current_setpoint} SCCM\nEnter new setpoint:",
            value=50.0, min=-1000.0, max=1000.0, decimals=1
        )
        
        if not ok:
            return
            
        # Set the new setpoint
        success, message = self.serial_worker.set_flow_point(channel, new_setpoint)
        if not success:
            QMessageBox.warning(self, "Error", f"Failed to set new setpoint: {message}")
            return
            
        # Create dialog to show response
        dialog = QDialog(self)
        dialog.setWindowTitle(f"Flow Response Monitor - {channel_display_name}")
        dialog.resize(600, 500)
        
        layout = QVBoxLayout(dialog)
        
        # Add plot
        plot_widget = pg.PlotWidget()
        plot_widget.showGrid(x=True, y=True)
        plot_widget.setLabel('left', 'Flow (SCCM)')
        plot_widget.setLabel('bottom', 'Time (s)')
        layout.addWidget(plot_widget)
        
        # Add status label
        status_label = QLabel(f"Monitoring response to setpoint change: {current_setpoint} → {new_setpoint} SCCM")
        layout.addWidget(status_label)
        
        # Add close button
        button_box = QHBoxLayout()
        close_button = QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        button_box.addWidget(close_button)
        layout.addLayout(button_box)
        
        # Setup data structures
        times = []
        flows = []
        setpoint_line = pg.InfiniteLine(pos=new_setpoint, angle=0, pen=pg.mkPen('r', width=2))
        plot_widget.addItem(setpoint_line)
        
        flow_curve = plot_widget.plot(pen='b')
        
        # Setup timer for reading flow
        start_time = time.time()
        
        def update_plot():
            elapsed = time.time() - start_time
            times.append(elapsed)
            
            # Get current flow
            response = self.serial_worker.controller.read_pressure(channel)
            if response:
                parsed = self.serial_worker.controller.parse_pressure_response(response)
                if parsed is not None:
                    flows.append(parsed)
                    status_label.setText(f"Time: {elapsed:.1f}s, Flow: {parsed:.1f} SCCM, Target: {new_setpoint} SCCM")
                else:
                    flows.append(float('nan'))
                    status_label.setText(f"Time: {elapsed:.1f}s, Flow: ERROR, Target: {new_setpoint} SCCM")
            else:
                flows.append(float('nan'))
                status_label.setText(f"Time: {elapsed:.1f}s, Flow: NO READING, Target: {new_setpoint} SCCM")
                
            # Update plot
            flow_curve.setData(times, flows)
            
            # Adjust plot range
            if elapsed > 10:
                plot_widget.setXRange(max(0, elapsed-30), elapsed)
                
            # Auto-scale Y axis occasionally
            if len(flows) % 10 == 0:
                valid_flows = [f for f in flows if not math.isnan(f)]
                if valid_flows:
                    min_flow = min(valid_flows)
                    max_flow = max(valid_flows)
                    margin = (max_flow - min_flow) * 0.2
                    plot_widget.setYRange(min_flow - margin, max_flow + margin)
        
        # Start timer
        timer = QTimer()
        timer.timeout.connect(update_plot)
        timer.start(200)  # Update every 200ms
        
        # Stop timer when dialog closes
        dialog.finished.connect(timer.stop)
        
        dialog.exec()

    def check_flow_control_support(self, show_dialog=True):
        """Check which channels support flow control and update UI accordingly"""
        if not self.serial_worker:
            return
            
        # First identify all MFC channels
        print("Checking channel responses with native 946 MFC commands...")
        if self.serial_worker.controller:
            mfc_channels = self.serial_worker.controller.probe_mfc_channels()
            
            # Display a notification about detected MFCs
            detected_mfcs = []
            for channel, info in mfc_channels.items():
                if info.get("is_mfc", False):
                    detected_mfcs.append(f"Channel {channel} (Slot {info['slot']})")
            
            if detected_mfcs and show_dialog:
                message = "Native 946 MFC commands responded on:\n" + "\n".join(detected_mfcs)
                message += "\n\nUse the 'Identify MFC Channels' button for per-command details."
                QMessageBox.information(self, "Responsive MFC Channels", message)
            elif not detected_mfcs and show_dialog:
                message = ("No channel responded to native 946 MFC commands.\n\n"
                           "If you have MFCs connected, please check:\n"
                           "1. The MFC is physically installed in the correct slot\n"
                           "2. The MFC is properly connected to the 946 controller\n"
                           "3. The 946 controller is properly configured for flow control\n\n"
                           "Use the 'Identify MFC Channels' button for more details.")
                QMessageBox.warning(self, "No Responsive MFC Channels", message)
            
        for channel in self.flow_controls:
            channel_num = int(channel[2:])
            if self.serial_worker.supports_flow_control(channel_num):
                # Channel supports flow control
                self.flow_controls[channel]['button'].setEnabled(True)
                self.flow_controls[channel]['spinbox'].setEnabled(True)
                self.flow_controls[channel]['button'].setText("Set")
                # Remove any "Monitor Only" label if it exists
                if 'monitor_label' in self.flow_controls[channel]:
                    self.flow_controls[channel]['monitor_label'].hide()
            else:
                # Channel is monitor-only
                self.flow_controls[channel]['button'].setEnabled(False)
                self.flow_controls[channel]['spinbox'].setEnabled(False)
                self.flow_controls[channel]['button'].setText("N/A")
                
                # Add a "Monitor Only" label if it doesn't exist
                if 'monitor_label' not in self.flow_controls[channel]:
                    label = QLabel("Monitor Only")
                    label.setStyleSheet("color: red; font-style: italic;")
                    self.flow_controls[channel]['layout'].addWidget(label)
                    self.flow_controls[channel]['monitor_label'] = label
                else:
                    self.flow_controls[channel]['monitor_label'].show()

    def show_troubleshooting_guide(self):
        """Show a dialog with troubleshooting information for MFC issues"""
        guide = """MFC TROUBLESHOOTING GUIDE
=======================

Channel/Slot Mapping on MKS 946:
-------------------------------
Slot A = channels 1 and 2 (A1 and A2)
Slot B = channels 3 and 4 (B1 and B2)
Slot C = channels 5 and 6 (C1 and C2)

If your MFC is physically installed in slot B but you're trying to control
channel 1 (slot A), it won't work. You must use channel 3 or 4 instead.

Older Firmware Support (FC 1.23):
------------------------------
This software has been configured to work with older MFC firmware (like FC 1.23)
that doesn't support the newer QIT and QFE commands. All channels are automatically 
enabled for flow control even if these commands would normally fail.

This means that even though the controller doesn't explicitly report MFCs on all
channels, the software allows you to control them as if they were detected.

Common Issues:
------------
1. "Monitor Only" error when trying to set flow rate:
   - Your code is trying to control a channel where no MFC is detected
   - Solution: Use the channel number matching the physical slot where your MFC is installed

2. No response or "NAK" when trying to set flow:
   - The MKS 946 doesn't recognize an MFC on that channel
   - Solutions:
     a. Verify the physical MFC module is installed in the correct slot
     b. Check wiring between the MFC and the 946 controller
     c. Make sure your code is sending commands to the correct channel number

3. MFC not detected on any channel:
   - Check that the MFC board is properly installed in the 946
   - Verify that the MFC is properly connected to the 946
   - Try a different serial port or connection method

Physical Setup Requirements:
--------------------------
1. The MKS 946 must have an MFC I/O board installed in at least one slot
2. The MFC must be properly connected to the 946 using the correct 15-pin connector
3. The MFC board must be installed in the slot you're trying to control

Using the Identify MFC Channels Tool:
----------------------------------
1. This tool queries each channel to check if an MFC is detected
2. It tells you which channels have MFC hardware installed 
3. For older firmware (FC 1.23), all channels are treated as MFC devices
   even if they don't respond to standard detection commands
"""
        
        dialog = QDialog(self)
        dialog.setWindowTitle("MFC Troubleshooting Guide")
        dialog.resize(600, 500)
        
        layout = QVBoxLayout(dialog)
        
        text_edit = QTextEdit()
        text_edit.setReadOnly(True)
        text_edit.setPlainText(guide)
        layout.addWidget(text_edit)
        
        close_button = QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        layout.addWidget(close_button)
        
        dialog.exec()


def terminal_mode():
    """Run in terminal-only mode displaying pressure readings"""
    try:
        # Check permissions
        if not setup_permissions():
            print("\nPlease restart the program after logging back in.")
            sys.exit(1)
            
        # Initialize controller directly with ttyS4
        controller = MFCController(port=DEFAULT_PORT)
        MFCController._instance = controller
        
        # Print custom channel names
        print("\nMonitoring flow with the following channel names:")
        for channel_key, display_name in CHANNEL_NAMES.items():
            print(f"  {channel_key} = {display_name}")
        
        print("\nStarting pressure monitoring, press Ctrl+C to exit")
        
        # Continuous reading and display
        while True:
            controller.read_all_pressures()  # This already prints to terminal with custom names
            time.sleep(0.1)  # Reduced from 0.5 for faster updates
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped")
        if controller:
            controller.close()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    # Check if terminal mode is requested
    if len(sys.argv) > 1 and sys.argv[1] in ['-t', '--terminal']:
        terminal_mode()
    else:
        app = QApplication(sys.argv)
        
        app.setApplicationName("Precision Olfactometer")
        app.setApplicationDisplayName("Precision Olfactometer")
        app.setOrganizationName("SENSE")
        app.setOrganizationDomain("sites.wustl.edu/sense")
        
        icon_path = os.path.join(os.path.dirname(__file__), 'icon.png')
        if os.path.exists(icon_path):
            app.setWindowIcon(QIcon(icon_path))
        else:
            print(f"Warning: Icon file not found at {icon_path}")
        
        window = MainWindow()
        window.show()
        
        sys.exit(app.exec()) 
