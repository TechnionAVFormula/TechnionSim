import yaml
from collections import OrderedDict
from os import listdir
from os import path
from os.path import join
from os.path import isfile
from os import walk, getenv
from subprocess import Popen

from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QCheckBox
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtGui import QFont
from qt_gui.plugin import Plugin


class EUFSLauncher(Plugin):
    def __init__(self, context):
        """
        This function handles loading the launcher GUI
        and all the setting-up of the values and buttons displayed.
        """

        super(EUFSLauncher, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName("EUFSLauncher")

        # State variables
        self.node = context.node
        self.logger = self.node.get_logger()
        self.LAUNCHER_SHARE = get_package_share_directory("eufs_launcher")
        self.TRACKS_SHARE = get_package_share_directory("eufs_tracks")
        self.popens = []  # Create array of popen processes

        # Declare Launcher Parameters
        default_config_path = join(self.LAUNCHER_SHARE, "config", "eufs_launcher.yaml")
        yaml_path = self.node.declare_parameter("config", default_config_path).value
        use_gui = self.node.declare_parameter("gui", True).value

        # Load in eufs_launcher parameters
        with open(yaml_path, "r") as stream:
            try:
                self.default_config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName("EUFSLauncherUI")
        context.add_widget(self._widget)

        # Extend the widget with all attributes and children from UI file
        self.main_ui_file = join(self.LAUNCHER_SHARE, "resource", "launcher.ui")
        loadUi(self.main_ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set
        # in _widget). This is useful when you open multiple plugins at once.
        # Also if you open multiple instances of your plugin at once, these
        # lines add number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            the_title = self._widget.windowTitle() + (" (%d)" % context.serial_number())
            self._widget.setWindowTitle(the_title)

        # Give widget components permanent names
        self.TRACK_SELECTOR = self._widget.findChild(QComboBox, "WhichTrack")
        self.LAUNCH_BUTTON = self._widget.findChild(QPushButton, "LaunchButton")
        self.REFRESH_TRACK_BUTTON = self._widget.findChild(
            QPushButton, "RefreshTrackButton"
        )
        self.VEHICLE_MODEL_MENU = self._widget.findChild(QComboBox, "WhichVehicleModel")
        self.COMMAND_MODE_MENU = self._widget.findChild(QComboBox, "WhichCommandMode")
        self.MODEL_PRESET_MENU = self._widget.findChild(QComboBox, "WhichModelPreset")
        self.ROBOT_NAME_MENU = self._widget.findChild(QComboBox, "WhichRobotName")
        self.LAUNCH_FILE_SELECTOR = self._widget.findChild(QComboBox, "WhichLaunchFile")

        # Check the file directory to update drop-down menu
        self.load_track_dropdowns()

        # Hook up buttons to onclick functions
        self.LAUNCH_BUTTON.clicked.connect(self.launch_button_pressed)
        self.REFRESH_TRACK_BUTTON.clicked.connect(self.load_track_dropdowns)

        # Setup Vehicle Models menu
        models_filepath = join(
            get_package_share_directory("eufs_models"), "models/models.txt"
        )
        vehicle_models_ = open(models_filepath, "r")
        vehicle_models = [model.strip() for model in vehicle_models_]  # remove \n
        default_model = self.default_config["eufs_launcher"]["default_vehicle_model"]
        EUFSLauncher.setup_q_combo_box(
            self.VEHICLE_MODEL_MENU, default_model, vehicle_models
        )

        # Setup Command Modes menu
        default_mode = self.default_config["eufs_launcher"]["default_command_mode"]
        modes = ["acceleration", "velocity"]
        EUFSLauncher.setup_q_combo_box(self.COMMAND_MODE_MENU, default_mode, modes)

        # Setup Conditions menu
        default_mode = self.default_config["eufs_launcher"]["default_vehicle_preset"]
        self.MODEL_CONFIGS = {
            "DryTrack": "configDry.yaml",
            "WetTrack": "configWet.yaml",
        }
        EUFSLauncher.setup_q_combo_box(
            self.MODEL_PRESET_MENU, default_mode, self.MODEL_CONFIGS.keys()
        )

        # Setup Robot Name menu
        default_mode = self.default_config["eufs_launcher"]["default_robot_name"]
        robots_filepath = join(get_package_share_directory("eufs_racecar"), "robots")
        modes = listdir(robots_filepath)
        EUFSLauncher.setup_q_combo_box(self.ROBOT_NAME_MENU, default_mode, modes)

        # Setup launch file options
        launch_files = []
        launch_directory_path = self.default_config["eufs_launcher"][
            "default_launch_directory"
        ]
        launch_directory_path = path.expandvars(launch_directory_path)

        eufs_master_path = getenv('EUFS_MASTER')
        eufs_master_src = join(eufs_master_path, 'src')
        submodules = listdir(eufs_master_src)

        # Check that path provided by the user ends with submodule
        if launch_directory_path.split('/')[-1].lower() in submodules:
            submodule_search_flag = True
        else:
            submodule_search_flag = False

        if submodule_search_flag:
            submodule_packages = listdir(launch_directory_path)
            for i, package in enumerate(submodule_packages):
                submodule_packages_path = (join(launch_directory_path, package))
                for directory_path, directory_name, files in walk(submodule_packages_path):
                    for file in files:

                        if file.endswith(".launch.py"):
                            launch_files.append(f"{package}/{file}")

        # Process all submodules under eufs-master
        if launch_directory_path == eufs_master_path:

            for directory_path, directory_name, files in walk(launch_directory_path):

                for file in files:
                    if file.endswith(".launch.py"):
                        if "build" not in directory_path and "install" not in directory_path:

                            temp = directory_path.split('/')
                            temp.pop(0)

                            eufs_master_split = eufs_master_path.split('/')
                            eufs_master_split.pop(0)
                            n = len(eufs_master_split)

                            # When getting the directory path, the output is essentially
                            # $EUFS_MASTER/src/<submodule>/<package>
                            # Remember that $EUFS_MASTER path may be different for everyone, hence the method implemented here is
                            # to ensure that it works regardless of different eufs-master path
                            # The 5th element corresponds to the submodule, whereas the 6th element
                            # corresponds to the package. There will be instances where python searches
                            # launch files not in a submodule which would break the following structure.
                            # Hence, the use of try and exception to catch this.
                            try:
                                if temp[(n - 1) + 2] in submodules:
                                    launch_files.append(join(temp[(n - 1) + 3], file))
                            except IndexError:
                                launch_files.append(join('launch', file))

        default_launch_file = self.default_config["eufs_launcher"][
            "default_launch_file"
        ]
        EUFSLauncher.setup_q_combo_box(
            self.LAUNCH_FILE_SELECTOR, default_launch_file, launch_files
        )

        # Add buttons from yaml file
        checkboxes = OrderedDict(
            sorted(
                self.default_config["eufs_launcher"]["checkboxes"].items(),
                key=lambda x: x[1]["priority"],
            )
        )
        self.checkbox_effect_mapping = []
        self.checkbox_parameter_mapping = []
        starting_xpos = 170
        starting_ypos = 257
        counter = 0
        for key, value in checkboxes.items():
            cur_xpos = starting_xpos + 100 * (counter % 2)
            cur_ypos = starting_ypos + 15 * (counter // 2)
            cur_cbox = QCheckBox(checkboxes[key]["label"], self._widget)
            cur_cbox.setChecked(checkboxes[key]["checked_on_default"])
            cur_cbox.setGeometry(cur_xpos, cur_ypos, 300, 30)
            cur_cbox.setFont(QFont("Sans Serif", 7))
            if "package" in checkboxes[key] and "launch_file" in checkboxes[key]:
                # This handles any launch files that the checkbox will launch
                # if selected.
                if "args" in checkboxes[key]:
                    cur_cbox_args = checkboxes[key]["args"].keys()
                else:
                    cur_cbox_args = []
                # The weird double-lambda thing is because python
                # lambda-captures
                # by reference, not value, and so consequently since the `key`
                # variable is used every loop, at the end of the loops all of
                # these lambda functions would actually be refering to the
                # *last*
                # loop's key, not each individual loop's key.
                # To solve that, we force key into a local variable by wrapping
                # our contents with a lambda taking it as a parameter, then
                # immediately passing it in.
                # We do the same for all other changing variables.
                self.checkbox_effect_mapping.append(
                    (
                        cur_cbox,
                        (
                            lambda key, cur_cbox_args: (
                                lambda: self.launch_with_args(
                                    checkboxes[key]["package"],
                                    checkboxes[key]["launch_file"],
                                    cur_cbox_args,
                                )
                            )
                        )(key, cur_cbox_args),
                        (lambda key: lambda: None)(key),
                    )
                )
            if "parameter_triggering" in checkboxes[key]:
                # This handles parameter details that will be passed to the
                # `simulation.launch.py` backbone file depending on whether the
                # checkbox is on or off
                self.checkbox_parameter_mapping.append(
                    (
                        cur_cbox,
                        checkboxes[key]["parameter_triggering"]["if_on"].keys(),
                        checkboxes[key]["parameter_triggering"]["if_off"].keys(),
                    )
                )

            setattr(self, checkboxes[key]["name"].upper(), cur_cbox)
            counter += 1

        # Looping over all widget to fix scaling issue via manual scaling
        # Scaling done via magically comparing the width to the 'default'
        # 1700 pixels
        rec = QApplication.desktop().screenGeometry()
        scalar_multiplier = rec.width() / 1700.0
        for widget in self._widget.children():
            if hasattr(widget, "geometry"):
                geom = widget.geometry()
                if not isinstance(widget, QLabel):
                    new_width = geom.width() * scalar_multiplier
                else:
                    new_width = geom.width() * scalar_multiplier + 200
                widget.setGeometry(
                    geom.x() * scalar_multiplier,
                    geom.y() * scalar_multiplier,
                    new_width,
                    geom.height() * (scalar_multiplier),
                )

        # If use_gui is false, we jump straight into launching the track
        if not use_gui:
            self.launch_button_pressed()

    @staticmethod
    def setup_q_combo_box(q_combo_box, default_mode, modes):
        q_combo_box.clear()
        # None option is added for the use of launch files selector
        if default_mode.lower() == "none":
            q_combo_box.addItem(default_mode)
        if default_mode in modes:
            q_combo_box.addItem(default_mode)
        for mode in modes:
            if mode != default_mode:
                q_combo_box.addItem(mode)

    def load_track_dropdowns(self):
        """
        Peruses file system for files to add to the drop-down menus of
        the launcher.
        """

        # Clear the dropdowns
        self.TRACK_SELECTOR.clear()
        # Get tracks from eufs_tracks package
        launch_dir_path = join(self.TRACKS_SHARE, "launch")
        launch_files = [
            f for f in listdir(launch_dir_path) if isfile(join(launch_dir_path, f))
        ]

        # Remove "blacklisted" files (ones that don't define tracks)
        blacklist_filepath = join(self.TRACKS_SHARE, "launch/blacklist.txt")
        with open(blacklist_filepath, "r") as f:
            blacklist = [f.strip() for f in f.readlines()]  # remove \n
            launch_files = [f for f in launch_files if f not in blacklist]

        # Add Tracks to Track Selector
        base_track = self.default_config["eufs_launcher"]["base_track"]
        if base_track in launch_files:
            self.TRACK_SELECTOR.addItem(base_track.split(".")[0])
        for f in launch_files:
            if f != base_track:
                self.TRACK_SELECTOR.addItem(f.split(".")[0])

    def launch_button_pressed(self):
        """
        Launches Gazebo.

        Here is our pre-launch process:
          1: Create csv from track_to_launch
          2: Kill random noise tiles in accordance with the noise value
          3: Shuffle cone positions slightly in accordance with cone noise
             value
          4: Convert that to "LAST_LAUNCH.launch"
        """
        self.logger.info("Launching Nodes...")

        # Calculate parameters to pass
        track_layout = f"track:={self.TRACK_SELECTOR.currentText()}"
        vehicle_model = f"vehicleModel:={self.VEHICLE_MODEL_MENU.currentText()}"
        command_mode = f"commandMode:={self.COMMAND_MODE_MENU.currentText()}"
        model_config = self.MODEL_CONFIGS[self.MODEL_PRESET_MENU.currentText()]
        vehicle_model_config = f"vehicleModelConfig:={model_config}"
        robot_name = f"robot_name:={self.ROBOT_NAME_MENU.currentText()}"
        parameters_to_pass = [
            track_layout,
            vehicle_model,
            command_mode,
            vehicle_model_config,
            robot_name,
        ]

        # Get vehicle model information
        self.logger.info(f"Vehicle model: {self.VEHICLE_MODEL_MENU.currentText()}")
        self.logger.info(f"Command mode: {self.COMMAND_MODE_MENU.currentText()}")
        self.logger.info(f"Preset: {model_config}")
        self.logger.info(
            f"Robot description file: {self.ROBOT_NAME_MENU.currentText()}"
        )
        self.logger.info(f"Launch file: {self.LAUNCH_FILE_SELECTOR.currentText()}")

        # Get checkbox parameter information
        for (
            checkbox,
            param_if_on,
            param_if_off,
        ) in self.checkbox_parameter_mapping:
            if checkbox.isChecked():
                self.logger.info(f"Checkbox enabled: {param_if_on}")
                parameters_to_pass.extend(param_if_on)
            else:
                self.logger.info(f"Checkbox disabled: {param_if_off}")
                parameters_to_pass.extend(param_if_off)

        # Here we launch `simulation.launch.py` and custom launch files
        # if it is not None.
        self.launch_with_args(
            "eufs_launcher", "simulation.launch.py", parameters_to_pass
        )
        launch_file_description = f"{self.LAUNCH_FILE_SELECTOR.currentText()}"
        self.roslaunch_launch_file(launch_file_description)

        # Trigger launch files hooked to checkboxes
        for checkbox, effect_on, effect_off in self.checkbox_effect_mapping:
            if checkbox.isChecked():
                effect_on()
            else:
                effect_off()

        self.LAUNCH_BUTTON.setEnabled(False)

    def launch_with_args(self, package, launch_file, args):
        """Launches ros node."""
        command = [
            "ros2",
            "launch",
            package,
            launch_file,
            "use_sim_time:=true",
        ] + args
        self.logger.info(f"Command: {' '.join(command)}")
        process = Popen(command)
        self.popens.append(process)

    def roslaunch_launch_file(self, launch_file_description):
        """
        Launches custom launch file

        launch_file_description can be of 3 forms:
        - ['None']
        - ['launch/file.launch.py']
        - ['package/file.launch.py']
        """

        # Process launch file description
        temp = launch_file_description.split('/')
 
        if "None" in temp:
            self.logger.info(f"No additional launch file will be launched.")

        # If the .launch.py file is within the launch folder in the top-level directory
        # the command to launch the launch file is a bit different
        # e.g. ros2 launch launch/simulation.launch.py
        elif "launch" in temp:
            command = ["ros2", "launch", temp]
            self.logger.info(f"Command: {' '.join(command)}")
            process = Popen(command) 
            self.popens.append(process)

        else:
            package, file = temp[0], temp[1]
            command = ["ros2", "launch", package, file]
            self.logger.info(f"Command: {' '.join(command)}")
            process = Popen(command)
            self.popens.append(process)

    def shutdown_plugin(self):
        """Kill all nodes."""
        self.logger.info("Shutdown Engaged...")

        for process in self.popens:
            process.terminate()

        self.logger.info("All nodes killed")
