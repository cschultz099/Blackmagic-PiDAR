# System Imports
import os
os.environ['OPENCV_VIDEOIO_DEBUG'] = '0' # Debug OpenCV
os.environ['OPENCV_LOG_LEVEL'] = 'OFF' #Debug OpenCV
import dbus
import dbus.service
import time
import threading
import smbus

# Dbus Mainloop Import
from dbus.mainloop.glib import DBusGMainLoop, threads_init

# Custom Imports
from lidar_interface import *
from lens_profile_manager import *

# Third Party Imports
import cv2
import numpy as np
from kivy.app import App
from kivy.graphics.texture import Texture
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.textinput import TextInput
from kivy.uix.slider import Slider
from kivy.uix.button import Label
from kivy.uix.button import Button
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.image import Image
from kivy.uix.popup import Popup
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.dropdown import DropDown
from kivy.uix.gridlayout import GridLayout
from kivy.clock import Clock
from gi.repository import GLib
from scipy.interpolate import interp1d
from kivy.core.window import Window
#Window.fullscreen = True # Application window settings
Window.show_cursor = False # Application window settings

# Dbus threading and mainloop initialization
threads_init()  # Initialize threads for dbus
DBusGMainLoop(set_as_default=True)  # Set up the DBus main loop

# Blackmagic Camera Bluetooth Details
SERVICE_DEVICE_INFORMATION = "0000180a-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_MANUFACTURER = "00002a29-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_MODEL = "00002a24-0000-1000-8000-00805f9b34fb"
SERVICE_BLACKMAGIC = "291d567a-6d75-11e6-8b77-86f30ca893d3"

# Blackmagic Camera Bluetooth UUID Characteristics
OUTGOING_CAMERA_CONTROL = "5dd3465f-1aee-4299-8493-d2eca2f8e1bb"
INCOMING_CAMERA_CONTROL = "b864e140-76a0-416a-bf30-5876504537d9"

# Bluez Bluetooth Details
BUS_NAME = 'org.bluez'
AGENT_INTERFACE = 'org.bluez.Agent1'
AGENT_PATH = "/org/bluez/agent"

# Global D-BUS Variables
bus = None
device_obj = None
dev_path = None
bus = dbus.SystemBus()

#### Bluetooth Connection Methods ####
def set_trusted(path):
        props = dbus.Interface(bus.get_object("org.bluez", path),"org.freedesktop.DBus.Properties")
        props.Set("org.bluez.Device1", "Trusted", True)

def dev_connect(path):
    app_instance = CameraControlApp.get_instance()
    try:
        dev = dbus.Interface(bus.get_object("org.bluez", path), "org.bluez.Device1")
        dev.Connect()
        Clock.schedule_once(app_instance.status_connection_successful)
    except Exception as e:
        print(f"Error connecting to device: {e}")
        Clock.schedule_once(app_instance.status_error_connecting)

def pair_reply():
    print("Device paired")
    set_trusted(dev_path)
    print("Device trusted")
    dev_connect(dev_path)
    print("Device connected")
    CameraControlApp.get_instance().setup_focus_notifications()

def pair_error(error):
    err_name = error.get_dbus_name()
    Clock.schedule_once(CameraControlApp.get_instance().status_error_connecting())
    if err_name == "org.freedesktop.DBus.Error.NoReply":
        print("Timed out. Cancelling pairing")
        #device_obj.CancelPairing()
    else:
        print("Creating device failed: %s" % (error))

#### Shared Resources Class ####
class SharedResources:
    def __init__(self):
        self.distance = None
        self.lock = threading.Lock()
        
    def update_distance(self, new_distance):
        with self.lock:
            self.distance = new_distance
            
    def get_distance(self):
        with self.lock:
            return self.distance
# Instantiate shared resources
shared_resource = SharedResources()

#### Keypad Class ####
class Keypad(GridLayout):
    def __init__(self, target_input, **kwargs):
        super(Keypad, self).__init__(**kwargs)
        self.last_toggle_time = 0  # Initialize last hamburger menu toggle time
        self.target_input = target_input
        self.cols = 3  # 3 columns for the keypad
        self.rows = 4  # 4 rows for the keypad
        self.create_keypad()

    def create_keypad(self):
        # List of buttons for the keypad
        buttons = [
            '1', '2', '3',
            '4', '5', '6',
            '7', '8', '9',
            'clear', '0', 'enter'
        ]
        for label in buttons:
            btn = Button(text=label, size_hint=(None, None), size=(100, 100))
            btn.bind(on_release=self.button_action)
            self.add_widget(btn)

    def button_action(self, instance):
        current_time = time.time()
        if current_time - self.last_toggle_time < 0.1:
            return
        self.last_toggle_time = current_time

        if instance.text == 'clear':
            self.target_input.text = ''
            self.target_input.focus = True
        elif instance.text == 'enter':
            self.target_input.focus = False
            self.opacity = 0
            self.disabled = True
            CameraControlApp.get_instance().on_submit_pin_release()
        else:
            self.target_input.text += instance.text
            self.target_input.focus = True

#### Calibration Class ####
class Calibration(Screen):

#### Calibration Methods ####
    def __init__(self, shared_resource, **kwargs):
        super(Calibration, self).__init__(**kwargs)

        self.profile_manager = LensProfileManager()
        self.shared_resource = shared_resource
        self.toggle_time = 0
        self.calibration_data = [] # Initialize
        self.MAX_RETRIES = 5  # Maximum number of retries for invalid lens position

        # Create a dropdown for lens profiles
        self.profile_dropdown = DropDown()
        self.profile_btn = Button(text ='Select Lens', size_hint =(None, None), size =(150, 50))
        self.profile_btn.bind(on_release = self.profile_dropdown.open)
        self.profile_dropdown.bind(on_select=self.load_profile_on_select)

        # Create buttons for managing lens profiles
        self.save_profile_btn = Button(text='Save Lens', size_hint=(None, None), size=(150, 50))
        self.save_profile_btn.bind(on_release=self.save_current_profile)

        self.delete_profile_btn = Button(text='Delete Lens', size_hint=(None, None), size=(150, 50))
        self.delete_profile_btn.bind(on_release=self.delete_selected_profile)

        self.map_lens_btn = Button(text='Map Lens', size_hint=(None, None), size=(150, 50))
        self.map_lens_btn.bind(on_release=self.start_calibration)

        self.current_profile_name = "Lens 1"  # Set the default profile name
        self.load_selected_profile(self.current_profile_name)  # Load the default profile

        # Add the UI components to the screen
        layout = BoxLayout(orientation='vertical', spacing=10, size_hint=(None, None), pos_hint={'x': 0.1, 'y': 0.2})
        layout.add_widget(self.profile_btn)
        layout.add_widget(self.map_lens_btn)
        layout.add_widget(self.save_profile_btn)
        layout.add_widget(self.delete_profile_btn)

        self.add_widget(layout)

        # Populate the dropdown with existing profiles
        self.update_dropdown_profiles()

        back_button = Button(
            text='Back',
            size_hint=(None, None),
            size=(75, 75),
            pos_hint={'right': 1, 'top': 1}
        )

        # Bind on_release event to switch back to the main screen
        back_button.bind(on_release=self.switch_to_main)

        # Add back button to this screen
        self.add_widget(back_button)

        self.distance_indicator = Label(
            text='Distance: ',
            font_size='20sp',
            size_hint=(None, None),
            size=(200, 100),
            pos_hint={'left': 1, 'top': 1}
            )

        self.add_widget(self.distance_indicator)

        self.calibrating_for = 1  # Initialize the calibration distance to 1m
        
        # Create buttons

        confirm_button = Button(
            text='Confirm',
            size_hint=(None, None),
            size=(100, 50),
            pos_hint={'x': 0.5, 'y': 0.5}
        )

        # Bind button events
        confirm_button.bind(on_release=self.confirm_focus)
        
        # Add buttons to the screen
        self.add_widget(confirm_button)

        threading.Thread(target=self.read_distance, name="DistanceServiceCalibration").start() # Read distance from Lidar

    def read_distance(self):
        while True:
            self.update_distance_indicator()
            time.sleep(0.05)  # Read every 50ms

    def update_distance_indicator(self):
        distance = self.shared_resource.get_distance()
        if distance is not None:
            self.distance_indicator.text = f"Distance: {distance:.2f} m"
        else:
            # Handle invalid distance
            pass

    def confirm_focus(self, instance):
        current_time = time.time()
        if current_time - self.toggle_time < 0.2:  # 0.2 seconds debounce
            return
        self.toggle_time = current_time  # Update the last toggle time

        # Read the current lens position
        current_lens_position = CameraControlApp._instance.read_lens_position()
        # Store the lens position for the current Lidar-measured distance
        CameraControlApp._instance.update_calibration(self.calibrating_for, CameraControlApp._instance.store_lens_position(current_lens_position))

        print(f"Confirmed focus at lens position {current_lens_position} for distance {self.calibrating_for}m")

        # Toggle the calibration distance between 1m and 4m
        self.calibrating_for = 4 if self.calibrating_for == 1 else 1
        print(f"Next calibration will be for {self.calibrating_for}m")

    def switch_to_main(self, instance):
        self.calibrating_for = 1 # Set to calibration for 1m
        self.manager.current = 'main'

#### Lens profile management methods ####
    def update_dropdown_profiles(self):
        # Clear existing profiles
        self.profile_dropdown.clear_widgets()

        # Keep track of added profile names to ensure no duplicates
        added_profiles = set()

        for profile_name in self.profile_manager.profiles.keys():
            if profile_name and profile_name not in added_profiles:
                btn = Button(text=profile_name, size_hint_y=None, height=44)
                btn.bind(on_release=lambda btn=btn: self.profile_dropdown.select(btn.text))
                self.profile_dropdown.add_widget(btn)
                added_profiles.add(profile_name)

    def save_current_profile(self, instance):
        # Find the highest existing profile number
        max_profile_num = 0
        for profile_name in self.profile_manager.profiles.keys():
            if "Lens" in profile_name:
                try:
                    num = int(profile_name.split("Lens")[1].strip())
                    max_profile_num = max(max_profile_num, num)
                except ValueError:
                    pass

        # Create the new profile name based on the next number
        profile_name = f"Lens {max_profile_num + 1}"
        
        # Check if profile already exists, otherwise create a new one
        profile = self.profile_manager.get_profile(profile_name)
        if not profile:
            profile = LensProfile(profile_name)

        # Calibration data
        lidar_distances = CameraControlApp._instance.lidar_distances  # Grab lidar_distances from the CameraControlApp instance
        lens_positions = CameraControlApp._instance.lens_positions  # Grab lens_positions from the CameraControlApp instance
        for focus, position in zip(lidar_distances, lens_positions):
            if focus == 0:
                position = "0"
            if position is not None and (isinstance(position, int) or isinstance(position, str)):
                profile.add_calibration_data(focus, position)
            else:
                print(f"Invalid data not added: focus={focus}, position={position}")

        self.profile_manager.add_profile(profile)
        self.update_dropdown_profiles()
        print(f"Saved profile: {profile_name} with calibration data: {profile.get_calibration_data()}")

    def load_selected_profile(self, profile_name=None):
        if not profile_name:
            profile_name = self.current_profile_name
        profile = self.profile_manager.get_profile(profile_name)
        if profile:

            CameraControlApp._instance.lidar_distances = [0, 1, 4]
            CameraControlApp._instance.lens_positions = [data['position'] for data in profile.calibration_data]

            # Create a new spline function with updated data
            numerical_lens_positions = [CameraControlApp._instance.extract_average_from_string(pos['position']) for pos in profile.calibration_data]
            CameraControlApp._instance.spline = interp1d(CameraControlApp._instance.lidar_distances, numerical_lens_positions, bounds_error=False, fill_value="extrapolate")

            print(f"Updated calibration profile: lidar_distances = {CameraControlApp._instance.lidar_distances}, lens_positions = {CameraControlApp._instance.lens_positions}")
        CameraControlApp._instance.current_profile_name = profile_name
        self.profile_btn.text = profile_name
        print(f"Loaded profile: {profile.name}")

    def load_profile_on_select(self, instance, lens):
        self.current_profile_name = lens
        self.load_selected_profile(None)

    def delete_selected_profile(self, instance):
        self.profile_manager.delete_profile(self.profile_btn.text)
        self.update_dropdown_profiles()

    def start_calibration(self, instance):
        current_time = time.time()
        if current_time - self.toggle_time < 0.2:  # Debounce
            return
        self.toggle_time = current_time  # Update the last toggle time

        # Reset or initialize your calibration data
        self.calibration_data = []
        self.retry_count = 0  # Initialize retry count
        # Begin the calibration process
        self.current_focus = 0.0  # Starting focus value
        self.step_size = 0.01  # Incremental step size

        self.profile = self.profile_manager.get_profile(self.profile_btn.text)

        self.proceed_calibration()

    def proceed_calibration(self):
        # Adjust focus
        CameraControlApp._instance.calibrate_autofocus(self.current_focus)
        # Wait a bit for the lens to adjust and system to get the new lens position
        time.sleep(0.05)
        # Read lens position
        current_lens_position = CameraControlApp._instance.read_lens_position()

        # Check if the lens position is valid
        if current_lens_position is None or current_lens_position == '':
            print("Error: Invalid lens position. Retrying...")
            self.retry_count += 1
            if self.retry_count < self.MAX_RETRIES:
                return self.proceed_calibration()
            else:
                print("Error: Maximum retries reached. Stopping calibration.")
                return self.end_calibration()

        # Check if the current lens position is already in the lens_positions list
        if current_lens_position not in self.profile.lens_position_data:
            # Add the unique lens position to the focus_range_data list
            self.profile.lens_position_data.append(current_lens_position)

        # Check end condition
        if self.current_focus >= 1.0 or "inf" in current_lens_position:
            self.end_calibration()
        else:
            # Reset retry count
            self.retry_count = 0
            # Proceed to the next step
            self.current_focus += self.step_size
            self.proceed_calibration()

    def end_calibration(self):
        # Get the current profile or create a new one if it doesn't exist
        current_profile = self.profile_manager.get_profile(CameraControlApp._instance.current_profile_name)
        if not current_profile:
            current_profile = LensProfile(CameraControlApp._instance.current_profile_name)

        # Update the automapping data for the current profile
        for position in self.calibration_data:
            current_profile.add_lens_position_data(position)

        # Save the updated profile
        self.profile_manager.add_profile(current_profile)
        self.update_dropdown_profiles()
        print(f"Calibration complete! Automapping data saved to profile: {CameraControlApp._instance.current_profile_name}")

#### Main Application Class ####
class CameraControlApp(App, dbus.service.Object):

#### Initialization ####
    def __init__(self, shared_resource, **kwargs):
        self._bus = smbus.SMBus(1)  # Initialize the I2C bus for Battery
        dbus.service.Object.__init__(self, bus, AGENT_PATH)
        manager = dbus.Interface(bus.get_object('org.bluez', '/org/bluez'), 'org.bluez.AgentManager1')
        manager.RegisterAgent(AGENT_PATH, 'KeyboardDisplay')
        manager.RequestDefaultAgent(AGENT_PATH)
        print(f"Agent registered at path: {AGENT_PATH}")

        self.last_toggle_time = 0  # Initialize last hamburger menu toggle time
        self.last_autofocus_toggle_time = 0  # Initialize last autofocus button toggle time
        self.last_pair_button_toggle_time = 0  # Initialize last pair button toggle time

        self.passkey_event = threading.Event()  # Initialize passkey_event here
        self.passkey = None
        self.last_press_time = 0
        self.keypad_state = False

        self.calibration_factor = 0
        self.autofocus_enabled = False
        self.current_calibration_profile = None

        # Parameters for distance smoothing
        self.distance_buffer_size = 5
        self.distance_buffer = []
        # Parameters for distance validation
        self.min_valid_distance = 0.01
        self.max_valid_distance = 8

        self.last_normalized_focus = None
        self.last_focus_value_manual = 0.0
        self.last_focus_value_auto = 0.0

        self.current_lens_position = None
        self.current_focus = 0.0  # Initialize the current focus value

        self.last_lidar_distance = None
        self.oscillations = 0  # track oscillations

        # Calibration data
        self.lidar_distances = [0, 1, 4]  # Initialize as a list (Distance set to 0 meter, 1 meter, 4 meters)
        self.lens_positions = [0, 0, 0]  # Initialize as a list (current lens focus motor position)

        self.calibration_data = {} # Initialize as a dictionary

        self.profile_manager = LensProfileManager()
        self.current_profile_name = None  # This gets updated whenever you load a new profile

        # Create a linear interpolation function
        self.spline = interp1d(self.lidar_distances, self.lens_positions, bounds_error=False, fill_value="extrapolate")

        self.current_popup = None
        self.connecting_popup = None

        self.shared_resource = shared_resource

        super().__init__(**kwargs)

        CameraControlApp._instance = self

#### Passkey Request ####
    @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        try:
            print(f"RequestPasskey called with device: {device}")
            set_trusted(device)
            self.passkey_event.wait()  # Wait for passkey to be submitted
            self.passkey_event.clear()  # Clear the event for future use
            if self.passkey is None:
                print("Error: Passkey is None")
                return dbus.UInt32(0)  # Return a default value
            print(f"Passkey: {self.passkey}")
            return dbus.UInt32(self.passkey)
        except Exception as e:
            print(f"Error in RequestPasskey: {e}")
            return dbus.UInt32(0)  # Return a default value on error

#### Kivy Application Build ####
    def build(self):
        self.stop_event = threading.Event()  # Used to signal the loop thread to stop
        self.loop_thread = threading.Thread(target=self.run_dbus_loop, name="DBusLoopService").start()

        self.bus = dbus.SystemBus()
        self.primary_layout = RelativeLayout()  # Use RelativeLayout as the main layout

        # Initialize Screen Manager
        self.screen_manager = ScreenManager()

        # Add Hamburger Menu Button at the top-right corner
        self.hamburger_menu_button = Button(
            text='☰',
            font_name='DejaVuSans.ttf',
            size_hint=(None, None),
            size=(75, 75),
            pos_hint={'right': 1, 'top': 1}
        )
        self.hamburger_menu_button.bind(on_release=self.toggle_menu)

        # Indicator Layout
        self.indicator_layout = BoxLayout(orientation='vertical', size_hint=(None, None), size=(200, 100))
        self.indicator_layout.pos_hint = {'left': 1, 'top': 1}

        # Indicators
        self.autofocus_indicator = Label(text='Autofocus OFF', font_size='20sp')
        self.distance_indicator = Label(text='Distance: ', font_size='20sp')
        self.lens_position_indicator = Label(text='Focus', font_size='20sp')
        self.battery_percentage_indicator = Label(text='Battery:', font_size='20sp')
        self.indicator_layout.add_widget(self.autofocus_indicator)
        self.indicator_layout.add_widget(self.distance_indicator)
        self.indicator_layout.add_widget(self.lens_position_indicator)
        self.indicator_layout.add_widget(self.battery_percentage_indicator)
        Clock.schedule_interval(self.update_battery_percentage, 5) # Update every 5 seconds

        # Create the Menu Layout (Initially hidden)
        self.menu_layout = BoxLayout(orientation='vertical', size_hint=(None, None), size=(300, 400))
        self.menu_layout.pos_hint = {'right': 1, 'top': 0.835}  # Positioning just below the hamburger button
        self.menu_layout.opacity = 0  # Initially hidden
        self.menu_layout.disabled = True  # Initially disabled

        # Create a separate BoxLayout for UI components
        self.ui_layout = BoxLayout(orientation='vertical')

        self.pair_button = Button(text='Connect Camera')
        self.pair_button.bind(on_release=self.on_pair_button_release)
        self.ui_layout.add_widget(self.pair_button)
        self.dropdown = DropDown()

        # Add a TextInput widget for the pin code
        self.pin_input = TextInput(hint_text='Enter pin code')
        self.ui_layout.add_widget(self.pin_input)
        self.keypad = Keypad(target_input=self.pin_input)
        self.keypad.pos_hint = {'center_x': 0.5, 'center_y': 0.5}
        self.keypad.opacity = 0  # Initially hidden
        self.keypad.disabled = True  # Initially disabled
        self.pin_input.bind(focus=self.toggle_keypad)

        self.calibration_button = Button(text='Calibrate Autofocus')
        self.calibration_button.bind(on_release=self.calibrate_autofocus_button)
        self.ui_layout.add_widget(self.calibration_button)

        self.autofocus_toggle = Button(text='Toggle Autofocus')
        self.autofocus_toggle.bind(on_release=self.toggle_autofocus)
        self.ui_layout.add_widget(self.autofocus_toggle)

        # Shutdown & Restart Widget
        self.shutdown_button = Button(text='Shutdown')
        self.restart_button = Button(text='Restart')
        self.shutdown_button.bind(on_release=self.shutdown)
        self.restart_button.bind(on_release=self.restart)
        self.ui_layout.add_widget(self.shutdown_button)
        self.ui_layout.add_widget(self.restart_button)

        # Monitor Detection Widget
        self.error_label = Label(text='Connect video feed to camera', font_size='20sp', opacity=0)
        self.primary_layout.add_widget(self.error_label)
        Clock.schedule_interval(self.check_device_status, 1.0)  # Check for device status every second

        # Monitor code here
        self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # Create Kivy Image widget, set its size to 800x480, and add to layout
        self.video_image = Image(size=(800, 480), size_hint=(None, None))
        # Schedule frame update
        Clock.schedule_interval(self.update_frame, 1.0 / 60.0)  # Update at 60 FPS

        self.primary_layout.add_widget(self.video_image)
        self.primary_layout.add_widget(self.keypad)
        self.primary_layout.add_widget(self.hamburger_menu_button)
        self.primary_layout.add_widget(self.indicator_layout)
        self.primary_layout.add_widget(self.menu_layout)
        
        # Add the UI layout to the menu layout
        self.menu_layout.add_widget(self.ui_layout)

        # Main screen
        main_screen = Screen(name='main')
        main_screen.add_widget(self.primary_layout)
        self.screen_manager.add_widget(main_screen)

        # Calibration screen
        calibration_screen = Calibration(shared_resource=shared_resource, name='calibration')
        self.screen_manager.add_widget(calibration_screen)

        threading.Thread(target=self.auto_connect_devices, name="AutoConnectService").start() # Check for connected devices
        threading.Thread(target=self.read_distance, name="DistanceService").start()

        # Return the screen manager as the root widget
        return self.screen_manager

#### Singleton (CameraControlApp class access) ####
    @classmethod
    def get_instance(cls):
        return cls._instance

#### D-Bus Main Loop ####
    def run_dbus_loop(self):
        print("Starting D-Bus main loop...")
        loop = GLib.MainLoop()
        while not self.stop_event.is_set():
            loop.get_context().iteration(False)
            time.sleep(0.01)
        print("D-Bus main loop terminated.")

#### Camera Monitor ####
    def update_frame(self, dt):
        if self.capture is not None:  # Check if capture object exists
            ret, frame = self.capture.read()
            if ret:
                buf1 = cv2.flip(frame, 0)
                buf = buf1.tobytes()
                image_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
                image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
                self.video_image.texture = image_texture
            else:
                # Handle frame not read successfully, possibly device disconnected
                self.error_label.opacity = 1  # Show the error label
                self.set_black_screen()
        else:
            # Handle capture object being None, possibly device not connected
            self.error_label.opacity = 1  # Show the error label
            self.set_black_screen()

    def set_black_screen(self):
        black_frame = np.zeros((480, 800, 3), dtype=np.uint8)
        buf1 = cv2.flip(black_frame, 0)
        buf = buf1.tobytes()
        image_texture = Texture.create(
            size=(black_frame.shape[1], black_frame.shape[0]), colorfmt='bgr')
        image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        self.video_image.texture = image_texture

    def check_device_status(self, dt):
        if self.capture is not None and self.capture.isOpened():
            self.error_label.opacity = 0  # Hide the error label
        else:
            self.error_label.opacity = 1  # Show the error label
            self.set_black_screen()  # Set the screen to black
            if self.capture is not None and self.capture.isOpened():
                self.capture.release()  # Release the capture device if it's open
            self.capture = cv2.VideoCapture(0)  # Attempt to reconnect
            
            # If still not opened, explicitly set to None
            if self.capture is not None and not self.capture.isOpened():
                self.capture = None

#### Battery & System Management ####
    def capacity(self):
        try:
            address = 0x57
            battery_percentage = self._bus.read_byte_data(address, 0x2A)
            return battery_percentage
        except Exception as e:
            print(f"Exception encountered (Battery is probably not connected): {e}")
            return 0

    def update_battery_percentage(self, *args):
        self.battery_percentage = self.capacity()
        self.battery_percentage_indicator.text = f"Battery: {self.battery_percentage}%"

    def shutdown(self, instance):
        current_time = time.time()
        if current_time - self.last_toggle_time < 0.1:
            return
        self.last_toggle_time = current_time
        os.system("sudo shutdown -h now")

    def restart(self, instance):
        current_time = time.time()
        if current_time - self.last_toggle_time < 0.1:
            return
        self.last_toggle_time = current_time
        os.system("sudo reboot")

#### Bluetooth Methods ####
    def device_disconnection_monitor(self, device_path):
        self.device_obj = self.bus.get_object("org.bluez", device_path)
        self.device_obj.connect_to_signal("PropertiesChanged", self.on_device_properties_changed)

    def on_device_properties_changed(self, interface, changed_properties, invalidated_properties):
        # If the Connected property is in the changed properties and it's False
        if "Connected" in changed_properties and not changed_properties["Connected"]:
            Clock.schedule_once(self.status_disconnect)

    def discover_characteristics(self):
        device_path = self.device.object_path
        mngd_objs = dbus.Interface(self.bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
        objs = mngd_objs.GetManagedObjects()
        for path, interfaces in objs.items():
            if device_path in path and "org.bluez.GattCharacteristic1" in interfaces:
                char_uuid = interfaces["org.bluez.GattCharacteristic1"]["UUID"]
                print(f"Characteristic UUID: {char_uuid}")

    def get_characteristic_path(self, characteristic_uuid):
        device_path = self.device.object_path
        # Access the Managed Objects
        mngd_objs = dbus.Interface(self.bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
        objs = mngd_objs.GetManagedObjects()
        for path, interfaces in objs.items():
            if "org.bluez.GattCharacteristic1" in interfaces:
                char_uuid = interfaces["org.bluez.GattCharacteristic1"]["UUID"]
                if char_uuid == characteristic_uuid:
                    return path
        print('Characteristic path not found')
        return None

    def setup_focus_notifications(self):
        try:
            # Get the path to the focus characteristic for INCOMING_CAMERA_CONTROL
            focus_incoming_char_path = self.get_characteristic_path(INCOMING_CAMERA_CONTROL)
            
            if focus_incoming_char_path:
                # Get the GattCharacteristic1 interface for the characteristic
                focus_incoming_char = dbus.Interface(self.bus.get_object('org.bluez', focus_incoming_char_path), 'org.bluez.GattCharacteristic1')
                
                # Subscribe to notifications
                focus_incoming_char.StartNotify()
                
                # Add a match for the PropertiesChanged signal on the focus characteristic path
                self.bus.add_signal_receiver(self.handle_focus_notification,
                                            signal_name='PropertiesChanged',
                                            dbus_interface='org.freedesktop.DBus.Properties',
                                            path_keyword='path')
            else:
                print('Incoming focus characteristic path not found')
        
        except Exception as e:
            print(f'Error setting up notifications: {e}')

    def get_connected_devices(self):
        root_obj = self.bus.get_object('org.bluez', '/')
        obj_manager = dbus.Interface(root_obj, 'org.freedesktop.DBus.ObjectManager')
        devices = obj_manager.GetManagedObjects()
        connected_devices = []
        for path, interfaces in devices.items():
            if 'org.bluez.Device1' in interfaces and interfaces['org.bluez.Device1']['Connected']:
                connected_devices.append((path, interfaces['org.bluez.Device1']))
        return connected_devices

    def get_paired_devices(self):
        root_obj = self.bus.get_object('org.bluez', '/')
        obj_manager = dbus.Interface(root_obj, 'org.freedesktop.DBus.ObjectManager')
        devices = obj_manager.GetManagedObjects()
        paired_devices = []
        for path, interfaces in devices.items():
            if 'org.bluez.Device1' in interfaces and interfaces['org.bluez.Device1']['Paired']:
                paired_devices.append((path, interfaces['org.bluez.Device1']))
        return paired_devices

    def check_for_connected_devices(self):
        print("Checking for connected devices...")
        connected_devices = self.get_connected_devices()
        if connected_devices:
            print(f'Found {len(connected_devices)} connected device.')
            self.dropdown.clear_widgets()
            for path, device_info in connected_devices:
                addr = device_info['Address']
                name = device_info.get('Alias', 'Unknown device')
                print(f'  {addr}: {name}')
                self.device = dbus.Interface(self.bus.get_object('org.bluez', path), 'org.bluez.Device1')
                self.discover_characteristics()
                Clock.schedule_once(self.status_connection_successful)
                self.setup_focus_notifications()
                self.device_disconnection_monitor(path)
            return True
        return False

    def check_for_paired_devices(self):
        print("Checking for paired devices...")
        paired_devices = self.get_paired_devices()
        if paired_devices:
            print(f'Found {len(paired_devices)} paired device.')
            self.dropdown.clear_widgets()
            for path, device_info in paired_devices:
                addr = device_info['Address']
                name = device_info.get('Alias', 'Unknown device')
                print(f'  {addr}: {name}')
                self.device = dbus.Interface(self.bus.get_object('org.bluez', path), 'org.bluez.Device1')
                dev_path = f'/org/bluez/hci0/dev_{addr.replace(":", "_")}'
                dev_connect(dev_path)
                self.discover_characteristics()
                self.setup_focus_notifications()
                self.device_disconnection_monitor(path)
            return True
        return False

    def pair_device(self):
        print('Pairing with device...')
        try:
            time.sleep(3)  # Wait a bit to ensure the device is ready
            # Pairing with the device
            self.device.Pair(reply_handler=pair_reply, error_handler=pair_error, timeout=60000)

            Clock.schedule_once(self.status_enter_passkey)

            # Discover services and characteristics after connection
            self.discover_characteristics()
            self.device_disconnection_monitor(self.device)

            popup = Popup(title='Connection Status', content=Label(text='Device Paired & Connected'), size=(200, 150), size_hint=(None, None))
            popup.open()
            Clock.schedule_once(popup.dismiss, 2)

        except Exception as e:
            return

    def auto_connect_devices(self):
        # Check for connected devices
        Clock.schedule_once(self.status_connecting)
        if self.check_for_connected_devices():
            return
        # Check for paired devices
        if self.check_for_paired_devices():
            return
        else:
            Clock.schedule_once(self.status_not_paired)

#### Lidar Methods ####
    def read_distance(self):
        while True:
            distance, _, _ = read_tfluna_data()

            # Validate distance
            if not (self.min_valid_distance <= distance <= self.max_valid_distance):
                continue  # Skip this iteration and continue with the next one

            # Smooth distance
            self.distance_buffer.append(distance)
            if len(self.distance_buffer) > self.distance_buffer_size:
                self.distance_buffer.pop(0)
            smoothed_distance = np.median(self.distance_buffer)  # Use median for smoothing
            self.shared_resource.update_distance(smoothed_distance)
            self.update_distance_indicator()
            time.sleep(0.05)  # Read every 50ms

    def update_distance_indicator(self):
        distance = self.shared_resource.get_distance()
        if distance is not None:
            self.distance_indicator.text = f"Distance: {distance:.2f} m"
        else:
            # Handle invalid distance
            pass

#### Kivy Methods ####
    def on_pair_button_release(self, instance):
        current_time = time.time()
        if current_time - self.last_pair_button_toggle_time < 10.0: # Debounce time
            return
        self.last_pair_button_toggle_time = current_time  # Update the last toggle time

        # Check for connected devices
        if self.check_for_connected_devices():
            return
        # Check for paired devices
        if self.check_for_paired_devices():
            return

        # If no connected devices are found, proceed to discovery and pairing
        print("Starting device discovery...")
        adapter = dbus.Interface(self.bus.get_object('org.bluez', '/org/bluez/hci0'), 'org.bluez.Adapter1')
        adapter.StartDiscovery()
        time.sleep(5)  # Wait a bit to discover devices
        adapter.StopDiscovery()  # Stop discovery after populating the list

        root_obj = self.bus.get_object('org.bluez', '/')
        obj_manager = dbus.Interface(root_obj, 'org.freedesktop.DBus.ObjectManager')
        devices = obj_manager.GetManagedObjects()
        print('Found {} devices'.format(len(devices)))
        self.dropdown.clear_widgets()

        device_list_layout = BoxLayout(orientation='vertical', size_hint=(.5, .5), pos_hint={'center_x': .5, 'center_y': .5})

        for path, interfaces in devices.items():
            if 'org.bluez.Device1' in interfaces:
                addr = interfaces['org.bluez.Device1']['Address']
                name = interfaces['org.bluez.Device1'].get('Alias', 'Unknown device')
                uuids = interfaces['org.bluez.Device1'].get('UUIDs', [])

                if SERVICE_BLACKMAGIC in uuids:
                    print(f'  {addr}: {name} (Blackmagic device)')
                    button = Button(text=name, size_hint=(None, None), size=(800, 75))
                    label = Label(text="Available to Connect:", size_hint=(None, None), size=(800, 75))
                    button.bind(on_release=lambda btn, addr=addr: self.on_device_button_release(btn, addr))
                    device_list_layout.add_widget(label)
                    device_list_layout.add_widget(button)

                    # Remove the old device list layout if it exists
                    if hasattr(self, 'device_list_layout') and self.device_list_layout.parent:
                        self.device_list_layout.parent.remove_widget(self.device_list_layout)

                    # Update the attribute and add the new layout to the parent layout
                    self.device_list_layout = device_list_layout
                    self.primary_layout.add_widget(device_list_layout)

    def on_device_button_release(self, button, address):
        current_time = time.time()
        if current_time - self.last_toggle_time < 0.1:
            return
        self.last_toggle_time = current_time

        #Remove the old device list layout if it exists
        if hasattr(self, 'device_list_layout') and self.device_list_layout.parent:
            self.device_list_layout.parent.remove_widget(self.device_list_layout)

        # Start the discovery process and connect to the device
        adapter = dbus.Interface(self.bus.get_object('org.bluez', '/org/bluez/hci0'), 'org.bluez.Adapter1')
        adapter.StartDiscovery()
        print(f'Connecting to {address}...')
        Clock.schedule_once(self.status_pairing)
        self.device = dbus.Interface(self.bus.get_object('org.bluez', f'/org/bluez/hci0/dev_{address.replace(":", "_")}'), 'org.bluez.Device1')
        
        # Set the global dev_path variable
        global dev_path
        dev_path = f'/org/bluez/hci0/dev_{address.replace(":", "_")}'
        adapter.StopDiscovery()
        
        # Start the pairing process after selecting the device in a separate thread
        threading.Thread(target=self.pair_device, name="ParingService").start()
        self.dropdown.dismiss()

    def on_submit_pin_release(self):
        # Ensure passkey is interpreted as a string to maintain leading zeroes
        passkey_str = self.pin_input.text
        if len(passkey_str) != 6 or not passkey_str.isdigit():
            print(f"Invalid passkey format: {passkey_str}")
            return
        self.passkey = int(passkey_str)
        self.passkey_event.set()  # Signal the event indicating the passkey has been submitted

    def toggle_menu(self, instance):
        current_time = time.time()
        if current_time - self.last_toggle_time < 0.1:
            return
        self.last_toggle_time = current_time  # Update last toggle time

        if self.menu_layout.opacity == 0:
            self.menu_layout.opacity = 1  # Show menu
            self.menu_layout.disabled = False  # Enable widgets
        else:
            self.keypad.opacity = 0
            self.keypad.disabled = True
            self.menu_layout.opacity = 0  # Hide menu
            self.menu_layout.disabled = True  # Disable widgets

    def toggle_keypad(self, instance, value):
        if value:  # If the text input is focused
            self.keypad.opacity = 1
            self.keypad.disabled = False
        else:  # If the text input loses focus
            if self.keypad.opacity == 1:  # Check if the keypad is visible
                instance.focus = True  # Set the focus back to the TextInput to keep keypad open

#### LiDAR Calibration ####
    def handle_focus_notification(self, interface, changed, invalidated, path):
        if 'Value' in changed:
            focus_value_data = [int(b) for b in changed['Value']]

            # Check if this is a focus data packet for manual adjustment
            if focus_value_data[0] == 255 and focus_value_data[4] == 12 and focus_value_data[6] == 5:
                # Dynamically find the start index of the UTF-8 string
                start_idx = 8  # The UTF-8 string starts at index 8
                
                # Extract the UTF-8 encoded string representing the lens position in millimeters
                lens_position_str_bytes = bytes(focus_value_data[start_idx:])
                lens_position_str = lens_position_str_bytes.decode('utf-8').strip()

                # Update the current lens position
                self.current_lens_position = lens_position_str

                # Update Kivy Label
                self.lens_position_indicator.text = f'{lens_position_str}'

    def calibrate_autofocus_button(self, instance):
        self.screen_manager.current = 'calibration'
        self.calibrate_autofocus(0)
        Calibration.current_focus = 0  # Reset the current focus value

    def calibrate_autofocus(self, focus_value):
            # Convert the normalized focus value to the 5.11 fixed number format (range from 0.0 to 1.0)
            focus_value_fixed = int(focus_value * 0x800)

            # Prepare the command packet according to the given instructions
            command_packet = [
                0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
                focus_value_fixed & 0xFF, (focus_value_fixed >> 8) & 0xFF, 0x00, 0x00
            ]
            try:
                # Get the path to the focus characteristic
                focus_char_path = self.get_characteristic_path(OUTGOING_CAMERA_CONTROL)
                if focus_char_path:
                    # Write the command packet to the focus characteristic
                    focus_char = dbus.Interface(self.bus.get_object('org.bluez', focus_char_path), 'org.bluez.GattCharacteristic1')
                    focus_char.WriteValue(command_packet, {})
                else:
                    print('Focus characteristic path not found')
            except Exception as e:
                print(f'Device not found (Probably not Connected): {e}')

    def update_calibration(self, distance, focus_value):
        if distance == 1:
            self.lens_positions[1] = focus_value
        elif distance == 4:
            self.lens_positions[2] = focus_value
        else:
            print(f"Distance {distance} not found in existing calibration points.")

        # Create a new spline function with updated data
        numerical_lens_positions = [self.extract_average_from_string(pos) for pos in self.lens_positions]
        self.spline = interp1d(self.lidar_distances, numerical_lens_positions, bounds_error=False, fill_value="extrapolate")

        print(f"Updated calibration profile: lidar_distances = {self.lidar_distances}, lens_positions = {self.lens_positions}")

    def get_target_lens_position_based_on_calibration(self, lidar_distance):
        # Extract the numeric value from the position string
        def get_numeric_value_from_position(position):
            if not isinstance(position, str):
                print(f"Offending position: {position}")
                raise ValueError(f"Expected string for position but got {type(position)}")

            # Check for "Inf"
            if position == "Inf":
                return float('inf')

            # Check for ranges
            if " to " in position:
                start, end = position.split(" to ")
                start = start.replace("mm", "").strip()
                end = end.replace("mm", "").strip()
                # Return the midpoint of the range
                return (int(start) + int(end)) / 2

            # Single value
            else:
                return int(position.replace("mm", "").strip())

        if not self.current_profile_name:
            print("Error: No profile currently loaded.")
            return None

        profile = self.profile_manager.get_profile(self.current_profile_name)
        if not profile:  # Check if profile is None
            print(f"Error: Profile '{self.current_profile_name}' not found.")
            return None

        calibration_data = profile.get_calibration_data()

        if not calibration_data:
            print("Error: Calibration data not available in the profile.")
            return None

        # Extract positions for 1m and 4m
        pos_1m = get_numeric_value_from_position(next(item['position'] for item in calibration_data if item['focus'] == 1))
        pos_4m = get_numeric_value_from_position(next(item['position'] for item in calibration_data if item['focus'] == 4))

        # Calculate the slope based on 1m and 4m calibration points
        slope = (pos_4m - pos_1m) / 3  # 4m - 1m = 3m

        ALPHA = 4.0  # The extrapolation factor (Lens may need to focus closer, or we're gonna need more calibration points)
        # For distances less than 1m, extrapolate using the modified slope
        if lidar_distance < 1:
            extrapolated_val = pos_1m - (ALPHA * slope) * (1 - lidar_distance)
            return f"{extrapolated_val:.2f}mm"
        
        # For distances more than 4m, extrapolate using the slope
        if lidar_distance > 4:
            extrapolated_val = pos_4m + slope * (lidar_distance - 4)
            return f"{extrapolated_val:.2f}mm"

        # For distances between 1m and 4m, interpolate
        frac = (lidar_distance - 1) / 3
        interpolated_val = pos_1m + frac * (pos_4m - pos_1m)
        return f"{interpolated_val:.2f}mm"

    def should_increase_focus(self, current_position, target_position):
        # Extract
        def get_start_value(position):
            value = position.split(" to ")[0].replace("mm", "").strip()

            # Check if the value is numeric (considering both integer and float strings)
            try:
                float_val = float(value)
            except ValueError:
                print(f"Error: Non-numeric value detected '{value}' in position string '{position}'")
                return None

            if value == "Inf" or value == "-Inf":
                return 0  # Default value
            else:
                return round(float_val)  # Convert the value to float first, then round it to the nearest integer
            
        current_start = get_start_value(current_position)
        target_start = get_start_value(target_position)
        
        # Handle cases where the start value could not be determined
        if current_start is None or target_start is None:
            print(f"Error: Could not determine start value for '{current_position}' or '{target_position}'")
            return False

        return current_start < target_start

    def extract_average_from_string(self, s):
        # If s is already an integer, return it
        if isinstance(s, int):
            return s
        
        # Check for the "Inf" case and return the corresponding value
        if "Inf" in s:
            return 1  # Or any large number that you want to use to represent "Inf"

        # Otherwise, process the string to extract the average
        values = [int(x.replace("mm", "")) for x in s.split(" to ")]
        return sum(values) / len(values)

    def lens_position_difference(self, position1, position2):
        # Extract
        def get_start_value(position):
            value = position.split(" to ")[0].replace("mm", "").strip()
            
            # Check if the value is numeric (considering both integer and float strings)
            try:
                float_val = float(value)
            except ValueError:
                print(f"Error: Non-numeric value detected '{value}' in position string '{position}'")
                return 0  # Defaulting to 0

            if value == "Inf" or value == "-Inf":
                return 0  # default value
            else:
                return float_val  # Return the value as a float
                
        pos1_start = get_start_value(position1)
        pos2_start = get_start_value(position2)

        return abs(pos1_start - pos2_start)

    def find_closest_lens_position(self, target_position):
        """Finds the closest lens position from lens_position_data based on the given target_position."""
        # Extract the numeric value from the position string
        def get_numeric_value_from_position(position):
            position = position.replace("mm", "").strip()
            if " to " in position:
                start, end = position.split(" to ")
                # Return the midpoint of the range
                return (float(start) + float(end)) / 2
            else:
                return float(position)  # Convert position to float instead of int

        target_numeric = get_numeric_value_from_position(target_position)
        lens_positions = self.profile_manager.get_profile(self.current_profile_name).lens_position_data

        # Find the closest position in lens_position_data to the target_numeric
        closest_position = min(lens_positions, key=lambda pos: abs(get_numeric_value_from_position(pos) - target_numeric))
        return closest_position

    def compute_dynamic_step(self, current_position, target_position):
        # Get the difference between the current and target lens positions
        difference = self.lens_position_difference(current_position, target_position)
        
        # Minimum and maximum step sizes
        MIN_STEP = 0.02
        MAX_STEP = 0.1
        
        # Compute step based on the difference. 
        step = MAX_STEP / (1 + difference)
        
        # Ensure the step size is within the defined bounds
        step = min(MAX_STEP, max(MIN_STEP, step))
        
        return step

    def increase_focus(self):
        target_lens_position = self.get_target_lens_position_based_on_calibration(self.shared_resource.get_distance())
        closest_position = self.find_closest_lens_position(target_lens_position)
        should_increase = self.should_increase_focus(self.read_lens_position(), closest_position)

        # Only adjust if we need to increase the focus
        if should_increase:
            step = self.compute_dynamic_step(self.read_lens_position(), closest_position)

            MIN_STEP_CHANGE = 0.01
            new_focus = min(1.0, self.current_focus + max(step, MIN_STEP_CHANGE))
            
            if new_focus != self.current_focus:
                self.send_focus_value_auto(new_focus)
                # Update the current focus value
                self.current_focus = new_focus
                #print(f"Increased focus to {self.read_lens_position()}")
                #print(f"Closest Lens Position: {closest_position}")
                #print(f"Computed Step: {step}")

    def decrease_focus(self):
        target_lens_position = self.get_target_lens_position_based_on_calibration(self.shared_resource.get_distance())
        closest_position = self.find_closest_lens_position(target_lens_position)
        should_decrease = not self.should_increase_focus(self.read_lens_position(), closest_position)

        # Only adjust if we need to decrease the focus
        if should_decrease:
            step = self.compute_dynamic_step(self.read_lens_position(), closest_position)

            MIN_STEP_CHANGE = 0.01
            new_focus = max(0.0, self.current_focus - max(step, MIN_STEP_CHANGE))
            
            if new_focus != self.current_focus:
                self.send_focus_value_auto(new_focus)
                # Update the current focus value
                self.current_focus = new_focus
                #print(f"Decreased focus to {self.read_lens_position()}")
                #print(f"Closest Lens Position: {closest_position}")

    def read_lens_position(self):
        return self.current_lens_position

    def store_lens_position(self, lens_position_str):
        return lens_position_str

#### LiDAR Autofocus ####
    def on_autofocus_change_lidar(self):
        try:
            distance = self.shared_resource.get_distance()
        except IndexError:
            print("Failed to read distance from LiDAR")
            return

        if distance is None:
            print("Error: Distance is None")
            return

        # Use the stored calibration data to determine the target lens position
        target_lens_position = self.get_target_lens_position_based_on_calibration(distance)
        current_lens_position = self.read_lens_position()

        # Make sure camera is ready to assist with focus adjustment
        if not current_lens_position or not target_lens_position:
            return

        # Determine the direction to adjust
        should_increase = self.should_increase_focus(current_lens_position, target_lens_position)

        # Make an adjustment
        if should_increase:
            self.increase_focus()
            #print(f"Increased focus to {self.read_lens_position()}")
        else:
            self.decrease_focus()
            #print(f"Decreased focus to {self.read_lens_position()}")

    def send_focus_value_auto(self, focus_value):
        # Convert the focus value to the 5.11 fixed number format (range from 0.0 to 1.0)
        focus_value_fixed = int(focus_value * 0x800)

        # Prepare the command packet according to the given instructions
        command_packet = [
            0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
            focus_value_fixed & 0xFF, (focus_value_fixed >> 8) & 0xFF, 0x00, 0x00
        ]
        
        try:
            # Get the path to the focus characteristic
            focus_char_path = self.get_characteristic_path(OUTGOING_CAMERA_CONTROL)
            if focus_char_path:
                # Write the command packet to the focus characteristic
                focus_char = dbus.Interface(self.bus.get_object('org.bluez', focus_char_path), 'org.bluez.GattCharacteristic1')
                focus_char.WriteValue(command_packet, {})
                print(f"Focus Value: {focus_value}")
            else:
                print('Focus characteristic path not found')
        except Exception as e:
            print(f'Device not found (Probably not Connected): {e}')

    def toggle_autofocus(self, instance):
        current_time = time.time()
        if current_time - self.last_autofocus_toggle_time < 0.5:  # 0.5 debounce
            return
        self.last_autofocus_toggle_time = current_time  # Update the last toggle time

        self.autofocus_enabled = not self.autofocus_enabled
        if self.autofocus_enabled:
            self.autofocus_thread = threading.Thread(target=self.start_autofocus_loop, name="AutoFocusService").start()
            popup = Popup(title='', content=Label(text='Autofocus ON'), size=(200, 150), size_hint=(None, None))
            popup.open()
            Clock.schedule_once(popup.dismiss, 2)
            self.autofocus_indicator.text = 'Autofocus ON'
        else:
            popup = Popup(title='', content=Label(text='Autofocus OFF'), size=(200, 150), size_hint=(None, None))
            popup.open()
            Clock.schedule_once(popup.dismiss, 2)
            self.autofocus_indicator.text = 'Autofocus OFF'
            self.distance_indicator.text = 'Distance: '

    def start_autofocus_loop(self):
        self.autofocus_enabled = True
        while self.autofocus_enabled:
            self.on_autofocus_change_lidar()
            time.sleep(0.02)  # Adjust focus every so often

#### Status Message System Method dependencies####
    def dismiss_popup_status(self, instance):
        if self.current_popup:
            self.current_popup.dismiss()
            self.current_popup = None

    def retry_connect_status(self, instance):
        if self.current_popup:
            self.current_popup.dismiss()
            self.current_popup = None
            Clock.schedule_once(lambda dt: threading.Thread(target=self.auto_connect_devices, name="AutoConnectService").start(), 0.5)

#### Status Message System Methods ####
    def status_connecting(self, dt=None):
            if self.connecting_popup:
                self.connecting_popup.dismiss()
            popup = Popup(title='Connection Status', content=Label(text='Connecting to Camera'), size=(200, 150), size_hint=(None, None))
            popup.open()
            self.connecting_popup = popup

    def status_connection_successful(self, dt = None):
        if self.connecting_popup:
            self.connecting_popup.dismiss()
            popup = Popup(title='Connection Status', content=Label(text='Camera Connected'), size=(200, 150), size_hint=(None, None))
            popup.open()
            Clock.schedule_once(popup.dismiss, 2)

    def status_disconnect(self, dt=None):
        if self.current_popup:
            return  # If a popup is already shown, do nothing

        content = BoxLayout(orientation='vertical')
        label = Label(text='Camera Disconnected')
        dismiss_button = Button(text='Dismiss')
        retry_button = Button(text='Reconnect')
        
        content.add_widget(label)
        content.add_widget(dismiss_button)
        content.add_widget(retry_button)

        popup = Popup(title='Connection Status', content=content, size=(300, 200), size_hint=(None, None))
        dismiss_button.bind(on_release=self.dismiss_popup_status)
        retry_button.bind(on_release=self.retry_connect_status)

        self.current_popup = popup
        popup.open()

    def status_error_connecting(self, dt = None):
        if self.connecting_popup:
            self.connecting_popup.dismiss()

            if self.current_popup:
                return  # If a popup is already shown, do nothing

            content = BoxLayout(orientation='vertical')
            label = Label(text='Camera unable to connect')
            dismiss_button = Button(text='Dismiss')
            retry_button = Button(text='Retry')
        
            content.add_widget(label)
            content.add_widget(dismiss_button)
            content.add_widget(retry_button)

            popup = Popup(title='Connection Status', content=content, size=(300, 200), size_hint=(None, None))
            dismiss_button.bind(on_release=self.dismiss_popup_status)
            retry_button.bind(on_release=self.retry_connect_status)
        
            self.current_popup = popup
            popup.open()

    def status_pairing(self, dt = None):
            if self.connecting_popup:
                self.connecting_popup.dismiss()
            popup = Popup(title='Connection Status', content=Label(text='Pairing Camera'), size=(200, 150), size_hint=(None, None))
            popup.open()
            self.connecting_popup = popup

    def status_enter_passkey(self, dt = None):
        if self.connecting_popup:
            self.connecting_popup.dismiss()
            popup = Popup(title='Connection Status', content=Label(text='Please Enter Passkey'), size=(250, 150), size_hint=(None, None))
            popup.open()
            Clock.schedule_once(popup.dismiss, 2)

    def status_not_paired(self, dt = None):
        if self.connecting_popup:
            self.connecting_popup.dismiss()

            if self.current_popup:
                return  # If a popup is already shown, do nothing

            content = BoxLayout(orientation='vertical')
            label = Label(text='Please Pair Camera')
            dismiss_button = Button(text='Dismiss')

            content.add_widget(label)
            content.add_widget(dismiss_button)

            popup = Popup(title='Connection Status', content=content, size=(300, 200), size_hint=(None, None))
            dismiss_button.bind(on_release=self.dismiss_popup_status)

            self.current_popup = popup
            popup.open()

#### Application Lifecycle Methods ####
    def on_stop(self):
        # Signal the threads to stop
        self.autofocus_enabled = False
        if hasattr(self, 'autofocus_thread'):
            self.autofocus_thread.join()  # Wait for the autofocus thread to finish
        self.stop_event.set()
        self.loop_thread.join()  # Wait for the thread to finish
        self.capture.release()

CameraControlApp(shared_resource=shared_resource).run()