#!/usr/bin/env python3
from isaacsim import SimulationApp

# This sample enables a livestream server to connect to when running headless
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,  # Show the GUI
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Set display options to show default grid
}

# Start the omniverse application
kit = SimulationApp(launch_config=CONFIG)

from isaacsim.core.utils.extensions import enable_extension

# Default Livestream settings
kit.set_setting("/app/window/drawMouse", True)

# Enable Livestream extension
enable_extension("omni.kit.livestream.webrtc")

# Enable ROS2 bridge extension:
enable_extension("isaacsim.ros2.bridge")

# Enable the Isaac Sim extension for the ZED camera:
enable_extension("sl.sensor.camera")



import carb
import numpy as np
from scipy.spatial.transform import Rotation as R
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.robots import Robot

from omni.isaac.core import World
from omni.isaac.core.utils import stage as stage_utils
from pxr import UsdPhysics, UsdGeom, Usd, Gf, UsdLux

# import import WheeledRobot
from omni.isaac.wheeled_robots.robots import WheeledRobot
import omni.appwindow  # Contains handle to keyboard

# Import the BaseController from Isaac Sim for implementing custom controllers
from omni.isaac.core.controllers import BaseController

class Leo_rover(object):
    def __init__(self) -> None:#,physics_dt, render_dt) -> None:
        """
        Creates the simulation world with a Leo rover on default ground plane and sets up keyboard listener>
        """
        # Create a world object and add a ground plane to the scene
        self.my_world = World(stage_units_in_meters=1.0)

        # Path to your USD file
        usd_path = "/isaac-sim/assets/mars_yard/mars_yard.obj"
        
        

        # Add the USD scene to the world
        stage_utils.add_reference_to_stage(usd_path, "/World/mars_yard")

        # Get the prim for Mars Yard
        stage = self.my_world.stage
        terrain_prim = stage.GetPrimAtPath("/World/mars_yard")

        # Ensure the terrain has collision enabled
        if terrain_prim.IsValid():
            physics_api = UsdPhysics.CollisionAPI.Apply(terrain_prim)
            physics_api.CreateCollisionEnabledAttr(True)

            # Ensure it's a static collider
            # Apply correct transformation order: Scale -> Rotate -> Translate
            xform = UsdGeom.Xform(terrain_prim)
            xform.ClearXformOpOrder()
            xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))  # Scale first
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))  # Rotate second
            xform.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))  # Translate last

        else:
            print("Error: Could not find Mars Yard in the stage.")

        
        default_light_path = "/World/DomeLight"
        default_light = UsdLux.DomeLight.Define(stage, default_light_path)
        # Correct way to set light attributes
        default_light.GetIntensityAttr().Set(1000)  # Adjust intensity
        default_light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))  # White light
        default_light.GetEnableColorTemperatureAttr().Set(True)  # Enable color temperature mode
        default_light.GetColorTemperatureAttr().Set(6500)  # Set color temperature to 4500K

        # No attributes are modified — using all default settings

        print("DomeLight added with default settings!")
       

        
        # Load the rover
        asset_path = "/isaac-sim/assets/Leo_rover_ZED_XT32.usdz"

        print("asset_path: ", asset_path)
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Leo_rover_ZED_XT32")  # add robot to stage

                # Bindings for keyboard to command
        self._input_keyboard_mapping = {
            # exit commands
            "Q": [0.0, 0.0, True],  # 3th element indicates exit request
            "q": [0.0, 0.0, True],
            "ESCAPE": [0.0, 0.0, True],
            # forward command
            "NUMPAD_8": [1.0, 0.0, False],
            "UP": [1.0, 0.0, False],
            "W": [1.0, 0.0, False],
            "w": [1.0, 0.0, False],
            # back command
            "NUMPAD_2": [-1.0, 0.0, False],
            "DOWN": [-1.0, 0.0, False],
            "S": [-1.0, 0.0, False],
            "s": [-1.0, 0.0, False],
            # turn left / yaw command (positive) 
            "NUMPAD_6": [0.0, -1.0, False],
            "RIGHT": [0.0, -1.0, False],
            "D": [0.0, -1.0, False],
            "d": [0.0, -1.0, False],
            # right right / yaw command (negative)
            "NUMPAD_4": [0.0, 1.0, False],
            "LEFT": [0.0, 1.0, False],
            "A": [0.0, 1.0, False],
            "a": [0.0, 1.0, False],
        }
        self._base_command = np.zeros(3)  # [x, yaw, exit]
        self.running = True
        self.exit = False
        

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback
        """
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
                self.exit=self._base_command[2]
                
                # Check if exit is requested
                if self.exit==True:
                    self.running = False
                print("Key pressed: ", event.input.name)
                print("exit: ", self.exit)
                print("self.running: ", self.running)
                print("base_command: ", self._base_command)


    def setup(self) -> None:
        """
        Set up keyboard listener
        """
        # Initialize the world
        self.my_world.reset()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()


        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)



    

    def run(self):
        """
        Main simulation loop
        """
        
        while self.running:
        # Run in realtime mode, we don't specify the step size
            kit.update()

        kit._app.close()
        




def main():
    rover = Leo_rover()
    rover.setup()
    rover.run()
    
    


if __name__ == "__main__":
    main()

