from threading import Thread
import XInput as xinput
from time import sleep
import winreg
import hid
import sys


# TODO: work https://www.reddit.com/r/PS3/comments/nwi796/how_to_dual_shock_3_on_windows_10_wirelessly_and/ and other tutorials into README.md
__all__ = ["check_hid_guardian", "list_connected_hid_devices", "Rotation3D", "BatteryData", "TrackpadTouch", "GenericHIDController",
           "GenericControllerResult", "PS5Controller", "PS5ControllerResult", "PS4Controller", "PS4ControllerResult", "XInputController"]


def check_hid_guardian() -> bool:
    """
    check if hidguardian is used and controller is hidden (copied from pydualsense)
    """
    if sys.platform.startswith('win32'):
        try:
            access_reg = winreg.ConnectRegistry(None, winreg.HKEY_LOCAL_MACHINE)
            access_key = winreg.OpenKey(access_reg, r'SYSTEM\CurrentControlSet\Services\HidGuardian\Parameters', 0, winreg.KEY_READ)
            affected_devices = winreg.QueryValueEx(access_key, 'AffectedDevices')[0]
            if "054C" in affected_devices and "0CE6" in affected_devices:
                return True
            return False
        except OSError as e:
            pass

    return False


def list_connected_hid_devices(print_duplicates=False):
    """
    Lists connected HID devices
    """
    printed = []
    for device in hid.enumerate():
        if not print_duplicates and device["product_string"] not in printed:
            print(f"{device['product_string']}: vendor_id: {device['vendor_id']}, product_id: {device['product_id']}")
            printed.append(device["product_string"])


def clamp(val, min_, max_):
    return min(max_, max(val, min_))


class Rotation3D:
    def __init__(self):
        self.pitch = 0
        self.yaw = 0
        self.roll = 0

    def __str__(self):
        builder = "Rotation3D("
        # item
        for item in [item for item in dir(self) if "__" not in item]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "
        return builder[:-2] + ")"


class BatteryData:
    def __init__(self):
        self.percent = 100
        self.charging = True

    def __str__(self):
        builder = "BatteryData("
        for item in [item for item in dir(self) if "__" not in item]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "
        return builder[:-2] + ")"


class TrackpadTouch:
    def __init__(self):
        self.id = 0
        self.is_active = False
        self.x = 0
        self.y = 0

    def __str__(self):
        builder = "TrackpadTouch("
        for item in [item for item in dir(self) if "__" not in item]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "
        return builder[:-2] + ")"


class GenericControllerResult:
    """
    Class for formatting data from GenericHIDController.read()
    """
    # old:
    # def __init__(self, lx, ly, rx, ry, lstick, rstick, dpad_up, dpad_down, dpad_left, dpad_right, select, start, y, x, b, a, lbumper, ltrigger, rbumper, rtrigger):
    def __init__(self):
        self.LX = 0
        self.LY = 0
        self.RX = 0
        self.RY = 0

        self.left_stick_pressed = False
        self.right_stick_pressed = False

        self.dpad_up = False
        self.dpad_down = False
        self.dpad_left = False
        self.dpad_right = False

        self.select = False
        self.start = False

        self.a = False
        self.b = False
        self.x = False
        self.y = False

        self.left_bumper = False
        self.left_trigger = False
        self.right_bumper = False
        self.right_trigger = False

    def __str__(self):
        builder = "GenericControllerResult("
        for item in [item for item in dir(self) if "__" not in item]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "
        return builder[:-2] + ")"


class GenericHIDController:
    """
    Class for reading from a generic HID controller using the hidapi library

    Supported controllers:
    - PS3 controllers (used with DSHIDMini)
    - Xbox controllers
    - Any other controller that sends its data over HID using Microsoft's mapping

    IMPORTANT: the trigger values may be off because of a thing that Microsoft does:

    https://learn.microsoft.com/en-us/windows/win32/xinput/directinput-and-xusb-devices#gamepad
    """
    def __init__(self, vendor_id, product_id):
        """
        Creates a Controller object
        :param vendor_id: vendor id of the controller
        :param product_id: product id of the controller
        """
        self._controller = hid.device()
        self._controller.open(vendor_id, product_id)
        self._controller.set_nonblocking(True)

        self.state = GenericControllerResult()

        self._running = True
        self._read_thread = Thread(target=self.read)
        self._read_thread.start()

    def read(self):
        """
        Reads from the controller
        """
        if not self._running:
            raise Exception("read() called after close()!")

        while self._running:
            report = self._controller.read(64, timeout_ms=10)
            if report and len(report) > 0:
                # print(report)
                self.state.LX = report[1] - 128
                self.state.LY = report[3] - 127
                self.state.RX = report[5] - 128
                self.state.RY = report[7] - 127
                # only one trigger is pressed
                if report[8] == 128:
                    if report[9] < 100:
                        self.state.right_trigger = True
                        self.state.left_trigger = False
                    else:
                        self.state.right_trigger = False
                        self.state.left_trigger = True
                elif clamp(report[9], 126, 130) != report:
                    self.state.right_trigger = False
                    self.state.left_trigger = False
                else:
                    self.state.right_trigger = True
                    self.state.left_trigger = True
                buttons = report[10]
                self.state.a = (buttons & (1 << 0)) != 0
                self.state.b = (buttons & (1 << 1)) != 0
                self.state.x = (buttons & (1 << 2)) != 0
                self.state.y = (buttons & (1 << 3)) != 0
                self.state.left_bumper = (buttons & (1 << 4)) != 0
                self.state.right_bumper = (buttons & (1 << 5)) != 0
                self.state.select = (buttons & (1 << 6)) != 0
                self.state.start = (buttons & (1 << 7)) != 0
                sticks_and_dpad = report[11]
                self.state.left_stick_pressed = (sticks_and_dpad & (1 << 0)) != 0
                self.state.right_stick_pressed = (sticks_and_dpad & (1 << 1)) != 0
                sticks_and_dpad = bin(sticks_and_dpad)[2:-2]
                if len(sticks_and_dpad) != 0:
                    sticks_and_dpad = int(sticks_and_dpad, 2)
                    self.state.dpad_up = sticks_and_dpad in [1, 2, 8]
                    self.state.dpad_right = sticks_and_dpad in [2, 3, 4]
                    self.state.dpad_down = sticks_and_dpad in [4, 5, 6]
                    self.state.dpad_left = sticks_and_dpad in [6, 7, 8]
                else:
                    self.state.dpad_up = False
                    self.state.dpad_right = False
                    self.state.dpad_down = False
                    self.state.dpad_left = False

    def close(self):
        self._controller.close()
        self._running = False
        self._read_thread.join(timeout=0)


class PS5ControllerResult:
    """
    Class for formatting data from PS5Controller.read()
    """
    # def __init__(self, LX, LY, RX, RY, left_joystick_pressed, right_joystick_pressed, dpad_up, dpad_down, dpad_left, dpad_right, share, options, triangle, square, circle, x_button, L2, left_trigger_btn, right_trigger_btn, R2, L1, R1, mic_btn, ps_btn, trackpad_touch1, trackpad_touch2, touchpad_pressed, gyro, accelerometer, battery):
    def __init__(self):
        self.LX = 0
        self.LY = 0
        self.RX = 0
        self.RY = 0
        self.left_joystick_pressed = False
        self.right_joystick_pressed = False

        self.dpad_up = False
        self.dpad_down = False
        self.dpad_left = False
        self.dpad_right = False

        self.share = False
        self.options = False

        self.triangle = False
        self.square = False
        self.circle = False
        self.x_button = False

        self.L1 = False
        self.R1 = False
        self.L2 = 0
        self.R2 = 0

        self.left_trigger_btn = False
        self.right_trigger_btn = False

        self.mic_btn = False
        self.ps_btn = False

        self.trackpad_touch1 = TrackpadTouch()
        self.trackpad_touch2 = TrackpadTouch()
        self.touchpad_pressed = False

        self.accelerometer = Rotation3D()
        self.gyro = Rotation3D()

        self.battery = BatteryData()

    def __str__(self):
        builder = "PS5ControllerResult("
        for item in [item for item in dir(self) if "__" not in item]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "
        return builder[:-2] + ")"


class PS5Controller:
    """
    Class for reading from a PS5 controller using the hidapi library

    Heavily based off of the pydualsense library and uses pydualsense's code for checking if the controller is hidden by HIDGuardian
    """
    def __init__(self, device_num=0):
        """
        Creates a Controller object
        :param device_num: index of the controller
        """
        if sys.platform.startswith('win32'):
            if check_hid_guardian():
                raise Exception('HIDGuardian detected. Delete the controller from HIDGuardian and restart PC to connect to controller')

        devices = hid.enumerate(vendor_id=0x054c)
        matches = []
        for device in devices:
            if device["vendor_id"] == 0x054c and device["product_id"] == 0x0CE6:
                matches.append(device)

        try:
            self._controller = hid.device()
            self._controller.open(vendor_id=matches[device_num]["vendor_id"], product_id=matches[device_num]["product_id"])
            self._controller.set_nonblocking(True)
        except IndexError:
            raise Exception(f"device_num {device_num} out of range, detected {len(matches)} controller{'s' if len(matches) != 1 else ''}")
        self._device_num = device_num

        self.rumble_left = 0
        self.rumble_right = 0
        self.light_brightness = 0
        self.player_num = [0, 4, 10, 21, 27, 31][device_num + 1]
        self.pulse_mode = 0
        self.touchpad_color = (0, 0, 255)
        self.mic_light = False
        self.mic_muted = False
        self.left_trigger_mode = 0
        self.right_trigger_mode = 0
        self.left_trigger_forces = [0 for _ in range(7)]
        self.right_trigger_forces = [0 for _ in range(7)]

        self.is_bluetooth = len(self._controller.read(100)) == 78

        self.state = PS5ControllerResult()
        self._running = True
        self.__worker_thread = Thread(target=self._worker_thread)
        self.__worker_thread.start()

    def __str__(self):
        builder = "PS5Controller("
        # for item in [item for item in dir(self) if "__" not in item and item not in ['_read_data_thread', '_send_data_thread', 'close', 'read', 'send_data', 'set_light_brightness', 'set_light_pulse', 'set_mic_light', 'set_mic_muted', 'set_player_num', 'set_rumble', 'set_touchpad_color', 'set_trigger_force', 'set_trigger_mode', "_controller", "_read_thread", "_send_thread"]]:
        for item in ["device_num", "rumble_left", "rumble_right"]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "

        try:
            player_num = [0, 4, 10, 21, 27, 31].index(self.player_num) - 1
        except ValueError:
            player_num = "custom light pattern"

        builder += f"_device_num={player_num}"

        return builder + ")"

    def _worker_thread(self):
        while self._running:
            self.read()
            self.send_data()

    def read(self):
        """
        Reads from the controller
        """
        if not self._running:
            raise Exception("controller.read() called after controller.close()")

        report = self._controller.read(100)
        if report:
            if len(report) == 78:
                report = list(report)[1:]
                # support for disconnecting controller & pairing over bluetooth (i think)
                self.is_bluetooth = True
            else:
                self.is_bluetooth = False

            self.state.LX = report[1] - 127
            self.state.LY = report[2] - 127
            self.state.RX = report[3] - 127
            self.state.RY = report[4] - 127
            self.state.L2 = report[5]
            self.state.R2 = report[6]

            buttonState = report[8]
            self.state.triangle = (buttonState & (1 << 7)) != 0
            self.state.circle = (buttonState & (1 << 6)) != 0
            self.state.x_button = (buttonState & (1 << 5)) != 0
            self.state.square = (buttonState & (1 << 4)) != 0

            dpad = buttonState & 0x0F
            self.state.dpad_up = dpad in [7, 0, 1]
            self.state.dpad_right = dpad in [1, 2, 3]
            self.state.dpad_down = dpad in [3, 4, 5]
            self.state.dpad_left = dpad in [5, 6, 7]

            misc = report[9]
            self.state.right_joystick_pressed = (misc & (1 << 7)) != 0
            self.state.left_joystick_pressed = (misc & (1 << 6)) != 0
            self.state.options = (misc & (1 << 5)) != 0
            self.state.share = (misc & (1 << 4)) != 0
            self.state.right_trigger_btn = (misc & (1 << 3)) != 0
            self.state.left_trigger_btn = (misc & (1 << 2)) != 0
            self.state.R1 = (misc & (1 << 1)) != 0
            self.state.L1 = (misc & (1 << 0)) != 0

            misc2 = report[10]
            self.state.ps_btn = (misc2 & (1 << 0)) != 0
            self.state.touchpad_pressed = (misc2 & 0x02) != 0
            self.state.mic_btn = (misc2 & 0x04) != 0

            self.state.trackpad_touch1.id = report[33] & 0x7F
            self.state.trackpad_touch1.is_active = (report[33] & 0x80) == 0
            self.state.trackpad_touch1.x = ((report[35] & 0x0f) << 8) | (report[34])
            self.state.trackpad_touch1.y = ((report[36]) << 4) | ((report[35] & 0xf0) >> 4)

            self.state.trackpad_touch2.id = report[37] & 0x7F
            self.state.trackpad_touch2.is_active = (report[37] & 0x80) == 0
            self.state.trackpad_touch2.x = ((report[39] & 0x0f) << 8) | (report[38])
            self.state.trackpad_touch2.y = ((report[40]) << 4) | ((report[39] & 0xf0) >> 4)

            # TODO: check if these are right
            self.state.accelerometer.yaw = round(int.from_bytes(([report[16], report[17]]), byteorder='little', signed=True) * 0.01125, 2)
            self.state.accelerometer.pitch = round(int.from_bytes(([report[18], report[19]]), byteorder='little', signed=True) * 0.01125, 2)
            self.state.accelerometer.roll = round(int.from_bytes(([report[20], report[21]]), byteorder='little', signed=True) * 0.01125, 2)

            self.state.gyro.pitch = round(int.from_bytes(([report[22], report[23]]), byteorder='little', signed=True) * 0.01125, 2)
            self.state.gyro.yaw = round(int.from_bytes(([report[24], report[25]]), byteorder='little', signed=True) * 0.01125 + 90, 2)
            self.state.gyro.roll = round(int.from_bytes(([report[26], report[27]]), byteorder='little', signed=True) * 0.01125, 2)

            battery = report[53]
            self.state.battery.percent = min((battery & 0x0F) * 10 + 5, 100)
            tmp = (battery & 0xF0) >> 4
            self.state.battery.charging = tmp in [1, 2]

    def send_data(self):
        to_send = [0] * 64

        to_send[0] = 2
        to_send[1] = 0xff
        to_send[2] = 87  # 0x1 | 0x2 | 0x4 | 0x10 | 0x40

        to_send[3] = self.rumble_right
        to_send[4] = self.rumble_left

        # 5-8 audio stuff idk

        to_send[9] = self.mic_light
        to_send[10] = 0x10 if self.mic_muted is True else 0x00

        to_send[11] = self.right_trigger_mode
        to_send[12] = self.left_trigger_forces[0]
        to_send[13] = self.left_trigger_forces[1]
        to_send[14] = self.left_trigger_forces[2]
        to_send[15] = self.left_trigger_forces[3]
        to_send[16] = self.left_trigger_forces[4]
        to_send[17] = self.left_trigger_forces[5]
        to_send[20] = self.left_trigger_forces[6]

        to_send[22] = self.left_trigger_mode
        to_send[23] = self.right_trigger_forces[0]
        to_send[24] = self.right_trigger_forces[1]
        to_send[25] = self.right_trigger_forces[2]
        to_send[26] = self.right_trigger_forces[3]
        to_send[27] = self.right_trigger_forces[4]
        to_send[28] = self.right_trigger_forces[5]
        to_send[31] = self.right_trigger_forces[6]

        to_send[39] = 3  # 0x01 | 0x02
        to_send[43] = self.light_brightness
        to_send[44] = self.player_num
        to_send[45] = self.touchpad_color[0]
        to_send[46] = self.touchpad_color[1]
        to_send[47] = self.touchpad_color[2]

        if self.is_bluetooth:
            to_send = [49, 2] + to_send[1:] + ([0] * 13)

        self._controller.write(bytes(to_send))

    def set_rumble(self, left_motor, right_motor):
        if not isinstance(left_motor, int) or not isinstance(right_motor, int):
            raise TypeError("Rumble values must be an integer")
        if clamp(left_motor, 0, 255) != left_motor:
            raise ValueError("Rumble values must be between 0 and 255, inclusive")
        if clamp(right_motor, 0, 255) != right_motor:
            raise ValueError("Rumble values must be between 0 and 255, inclusive")
        self.rumble_left = left_motor
        self.rumble_right = right_motor

    def set_light_brightness(self, brightness):
        if not isinstance(brightness, int) or clamp(brightness, 0, 2) != brightness:
            raise ValueError("Brightness values must be 0, 1, or 2")
        self.light_brightness = brightness

    def set_player_num(self, player_num):
        if not isinstance(player_num, int) or clamp(player_num, 0, 5) != player_num:
            raise ValueError("Player num value must be 0-5")
        # bin(0) = 00000 (off)
        # bin(4) = 00100 (p1)
        # bin(10) = 01010 (p2)
        # bin(21) = 10101 (p3)
        # bin(27) = 11011 (p4)
        # bin(31) = 11111 (all)
        self.player_num = [0, 4, 10, 21, 27, 31][player_num]

    # def set_player_lights(self, l1, l2, l3, l4, l5):
    #     if not isinstance(l1, bool) or not isinstance(l2, bool) or not isinstance(l3, bool) or not isinstance(l4, bool) or not isinstance(l5, bool):
    #         raise TypeError("Light values must be booleans")
    #     self.player_num = 0
    #     self.player_num |= 0x1 if l5 else 0
    #     self.player_num |= 0x2 if l4 else 0
    #     self.player_num |= 0x4 if l3 else 0
    #     self.player_num |= 0x8 if l2 else 0
    #     self.player_num |= 0x10 if l1 else 0
        # self.player_num += 1 if l5 else 0
        # self.player_num += 2 if l4 else 0
        # self.player_num += 4 if l3 else 0
        # self.player_num += 8 if l2 else 0
        # self.player_num += 16 if l1 else 0

    def set_light_pulse(self, pulse_mode):
        if not isinstance(pulse_mode, int) or clamp(pulse_mode, 0, 2) != pulse_mode:
            raise ValueError("Pulse mode must be between 0 and 2, inclusive")
        self.pulse_mode = pulse_mode

    def set_touchpad_color(self, color: tuple):
        if len(color) != 3:
            raise ValueError("Color value must have length 3")
        for val in color:
            if clamp(val, 0, 255) != val:
                raise ValueError("Color values must be between 0 and 255, inclusive")
        self.touchpad_color = color

    def set_mic_light(self, light: bool):
        if not isinstance(light, bool):
            raise TypeError("Mic light value must be a boolean")
        self.mic_light = light

    def set_mic_muted(self, muted: bool):
        if not isinstance(muted, bool):
            raise TypeError("Muted value must be a boolean")
        self.mic_muted = muted
        self.set_mic_light(muted)

    def set_trigger_mode(self, mode: int, is_left: bool):
        """
        Sets the trigger resistance mode
        :param mode: mode (0 for off, 1 for rigid, 2 for a trigger-like feel, 252 for calibration)
        :param is_left: if it is the left trigger
        """
        if not isinstance(mode, int) or mode not in [0, 1, 2, 252]:
            raise ValueError("Mode value must be one of [0, 1, 2, 252]")
        if not isinstance(is_left, bool):
            raise TypeError("is_left must be a boolean!")
        if is_left:
            self.left_trigger_mode = mode
        else:
            self.right_trigger_mode = mode

    def set_trigger_force(self, index: int, force: int, is_left: bool):
        if not isinstance(force, int) or clamp(force, 0, 255) != force:
            raise ValueError("Force value must be an integer between 0 and 255, inclusive")
        if not isinstance(index, int) or clamp(index, 0, 7) != index:
            raise ValueError("Force index value must an integer be between 0 and 7, inclusive")
        if not isinstance(is_left, bool):
            raise TypeError("is_left value must be a boolean!")
        if is_left:
            self.left_trigger_forces[index] = force
        else:
            self.right_trigger_forces[index] = force

    def close(self):
        self.set_player_num(0)
        self.set_mic_light(False)
        self.set_touchpad_color((0, 0, 0))
        self.set_trigger_mode(0, False)
        self.set_trigger_mode(0, True)

        # give the worker thread time to send the data
        # for unknown reasons it doesn't work if I kill
        # the thread then call _worker_thread
        sleep(0.05)

        self.__worker_thread.join(timeout=0)
        self._running = False
        self._controller.close()


class PS4ControllerResult:
    """
    Class for formatting data from XboxController.read()
    """
    def __init__(self):
        self.LX = 0
        self.LY = 0
        self.RX = 0
        self.RY = 0

        self.left_stick_pressed = False
        self.right_stick_pressed = False

        self.dpad_up = False
        self.dpad_down = False
        self.dpad_left = False
        self.dpad_right = False

        self.select = False
        self.start = False

        self.x = False
        self.square = False
        self.triangle = False
        self.circle = False

        self.left_bumper = False
        self.left_trigger = 0
        self.left_trigger_btn = False
        self.right_bumper = False
        self.right_trigger = 0
        self.right_trigger_btn = False

        self.ps_btn = False

    def __str__(self):
        builder = "PS4ControllerResult("
        for item in [item for item in dir(self) if "__" not in item]:
            builder += item + "=" + str(eval(f"self.{item}")) + ", "
        return builder[:-2] + ")"


class PS4Controller:
    """
    Class for reading from a PS4 controller using the hidapi library
    """
    def __init__(self, device_num):
        """
        Creates a Controller object
        """

        devices = hid.enumerate(vendor_id=0x054c)
        matches = []
        for device in devices:
            # https://the-sz.com/products/usbid/index.php
            # or list_connected_hid_devices()
            if device["vendor_id"] == 0x054c and device["product_id"] == 0x05C4:
                matches.append(device)

        print(matches)
        try:
            self._controller = hid.device()
            self._controller.open(vendor_id=matches[device_num]["vendor_id"], product_id=matches[device_num]["product_id"])
            self._controller.set_nonblocking(True)
        except IndexError:
            raise Exception(f"device_num {device_num} out of range, detected {len(matches)} controller{'s' if len(matches) != 1 else ''}")
        self._device_num = device_num

        self.state = PS4ControllerResult()

        self._running = True
        self._worker_thread = Thread(target=self.read)
        self._worker_thread.start()

    def read(self):
        """
        Reads from the controller
        """
        if not self._running:
            raise Exception("read() called after close()!")

        while self._running:
            report = self._controller.read(100, timeout_ms=10)
            if report and len(report) > 0:
                self.state.LX = report[1] - 128
                self.state.LY = report[2] - 127
                self.state.RX = report[3] - 128
                self.state.RY = report[4] - 127

                dpad_and_buttons = report[5]
                self.state.square = (dpad_and_buttons & (1 << 4)) != 0
                self.state.x = (dpad_and_buttons & (1 << 5)) != 0
                self.state.circle = (dpad_and_buttons & (1 << 6)) != 0
                self.state.triangle = (dpad_and_buttons & (1 << 7)) != 0
                dpad_and_buttons &= 0x0f  # get last 4 bits
                self.state.dpad_up = dpad_and_buttons in [7, 0, 1]
                self.state.dpad_right = dpad_and_buttons in [1, 2, 3]
                self.state.dpad_down = dpad_and_buttons in [3, 4, 5]
                self.state.dpad_left = dpad_and_buttons in [5, 6, 7]

                misc = report[6]
                self.state.left_bumper = (misc & (1 << 0)) != 0
                self.state.right_bumper = (misc & (1 << 1)) != 0
                self.state.left_trigger_btn = (misc & (1 << 2)) != 0
                self.state.right_trigger_btn = (misc & (1 << 3)) != 0
                self.state.select = (misc & (1 << 4)) != 0
                self.state.start = (misc & (1 << 5)) != 0
                self.state.left_stick_pressed = (misc & (1 << 6)) != 0
                self.state.right_stick_pressed = (misc & (1 << 7)) != 0

                self.state.left_trigger = report[8]
                self.state.right_trigger = report[9]

                self.state.ps_btn = bool(report[7])

    def close(self):
        self._controller.close()
        self._running = False
        self._worker_thread.join(timeout=0)


class XInputController:
    """
    Class for reading from an xinput controller using XInput-Python
    """
    def __init__(self, index):
        self._index = index
        self.state = GenericControllerResult()
        self.battery = ('DISCONNECTED', 'EMPTY')

        self._running = True
        self._worker_thread = Thread(target=self._update)
        self._worker_thread.start()

    def _update(self):
        while self._running:
            xinput_state = xinput.get_state(self._index)
            buttons = xinput.get_button_values(xinput_state)
            triggers = xinput.get_trigger_values(xinput_state)
            joysticks = xinput.get_thumb_values(xinput_state)
            battery = xinput.get_battery_information(self._index)

            self.state.x = buttons["X"]
            self.state.y = buttons["Y"]
            self.state.a = buttons["A"]
            self.state.b = buttons["B"]

            self.state.dpad_up = buttons["DPAD_UP"]
            self.state.dpad_right = buttons["DPAD_RIGHT"]
            self.state.dpad_down = buttons["DPAD_DOWN"]
            self.state.dpad_left = buttons["DPAD_LEFT"]

            self.state.start = buttons["START"]
            self.state.select = buttons["BACK"]

            self.state.left_stick_pressed = buttons["LEFT_THUMB"]
            self.state.right_stick_pressed = buttons["RIGHT_THUMB"]

            self.state.left_bumper = buttons["LEFT_SHOULDER"]
            self.state.right_bumper = buttons["RIGHT_SHOULDER"]

            self.state.LX = round(joysticks[0][0] * 255)
            self.state.LY = round(joysticks[0][1] * 255)
            self.state.RX = round(joysticks[1][0] * 255)
            self.state.RY = round(joysticks[1][1] * 255)

            self.state.left_trigger = round(triggers[0] * 255)
            self.state.right_trigger = round(triggers[1] * 255)

            self.battery = battery

    def set_rumble(self, left_rumble, right_rumble) -> bool:
        """
        Sets the controller rumble
        :param left_rumble: Left rumble from 0 to 255
        :param right_rumble: Right rumble from 0 to 255
        :return: If the operation was successful
        """
        if not isinstance(left_rumble, int) or not isinstance(right_rumble, int):
            raise TypeError("Rumble values must be an integer!")
        if clamp(left_rumble, 0, 255) != left_rumble or clamp(right_rumble, 0, 255) != right_rumble:
            raise ValueError("Rumble values must be between 0 and 255")
        return xinput.set_vibration(self._index, left_rumble, right_rumble)

    def set_deadzone(self, which_one, value) -> None:
        """
        Sets the deadzone
        :param which_one: 0 for left joystick, 1 for right joystick, 2 for triggers
        :param value: the deadzone value (0-32767 for joysticks, 0-255 for triggers)
        :return: None
        """
        xinput.set_deadzone(which_one, value)

    def close(self):
        """
        Closes the controller
        :return:
        """
        self.set_rumble(0, 0)
        self._worker_thread.join(timeout=0)
        self._running = False


"""
Generic HID controller data
self._controller.read(64) returns an array of 14 items. key:
1:
    lstick left/right
3:
    lstick up/down
5:
    rstick left/right
7:
    rstick up/down
8:
    128 if only one trigger is pressed
9:
    128-255: ltrigger, 0-127: rtrigger
10:
    1st bit: x
    2nd bit: o
    3rd bit: square
    4th bit: triangle
    5th bit: lbumper
    6th bit: rbumper
    7th bit: select
    8th bit: start
11:
    1st bit: lstick push down
    2nd bit: rstick push down
"""
"""
ps5 controller data
self.controller.read(64) returns an array of 64 items. key:
1:
    lstick left/right
2:
    lstick up/down
3:
    rstick left/right
4:
    rstick up/down
5:
    left trigger (0-255)
6:
    right trigger (0-255)
7:
    i have no idea, seems to increment up to 255 and then reset
8:
    to binary --> make len 8 --> SPLIT in half
    1st half, back to decimal:
        [0, 1, 7] if up dpad
        [1, 2, 3] if right dpad
        [3, 4, 5] if down dpad
        [5, 6, 7] if left dpad
        8 if nothing pressed
    2nd half, in binary, right to left:
        0th bit: square
        1st bit: x
        2nd bit: circle
        3rd bit: triangle
9:
    in binary, right to left
    0th bit: left bumper
    1st bit: right bumper
    2nd bit: 1 if left trigger is over halfway pressed
    3rd bit: 1 if right trigger is over halfway pressed
    4th bit: button to the top left of the mousepad thing
    5th bit: options button to the top right of the mousepad thing
    6th bit: left stick pressed
    7th bit: right stick pressed 
10:
    0 if True 
11:
    0 if True
12-22: idk
23:
    roll no matter the orientation of the controller
24: idk
25/27:
    pitch but weird (to be math'd)
26:idk
28-63: idk
"""
