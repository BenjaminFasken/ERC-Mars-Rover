#!/usr/bin/env python3
from isaacsim import SimulationApp
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.core.utils import stage as stage_utils
from pxr import UsdPhysics, UsdGeom, Gf, UsdLux
import omni.appwindow
from isaacsim.core.utils.extensions import enable_extension

# Configuration for SimulationApp
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,
    "renderer": "RaytracedLighting",
    "display_options": 3286,
}

# Start the omniverse application
kit = SimulationApp(launch_config=CONFIG)

# Enable extensions
kit.set_setting("/app/window/drawMouse", True)
enable_extension("omni.kit.livestream.webrtc")
enable_extension("isaacsim.ros2.bridge")
enable_extension("sl.sensor.camera")

class Leo_rover(object):
    def __init__(self):
        self.my_world = World(stage_units_in_meters=1.0)
        usd_path = "/isaac-sim/assets/mars_yard/mars_yard.obj"
        stage_utils.add_reference_to_stage(usd_path, "/World/mars_yard")
        stage = self.my_world.stage
        terrain_prim = stage.GetPrimAtPath("/World/mars_yard")
        if terrain_prim.IsValid():
            physics_api = UsdPhysics.CollisionAPI.Apply(terrain_prim)
            physics_api.CreateCollisionEnabledAttr(True)
            xform = UsdGeom.Xform(terrain_prim)
            xform.ClearXformOpOrder()
            xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
            xform.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))
        else:
            print("Error: Could not find Mars Yard in the stage.")
        
        default_light_path = "/World/DomeLight"
        default_light = UsdLux.DomeLight.Define(stage, default_light_path)
        default_light.GetIntensityAttr().Set(1000)
        default_light.GetColorAttr().Set(Gf.Vec3f(1, 1, 1))
        default_light.GetEnableColorTemperatureAttr().Set(True)
        default_light.GetColorTemperatureAttr().Set(6500)
        print("DomeLight added with default settings!")
        
        asset_path = "/isaac-sim/assets/Leo_rover_control_lidar_camera3.usdz"
        print("asset_path: ", asset_path)
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Leo_rover_control_lidar_camera3")
        
        self._input_keyboard_mapping = {
            "Q": [0.0, 0.0, True], "q": [0.0, 0.0, True], "ESCAPE": [0.0, 0.0, True],
            "NUMPAD_8": [1.0, 0.0, False], "UP": [1.0, 0.0, False], "W": [1.0, 0.0, False], "w": [1.0, 0.0, False],
            "NUMPAD_2": [-1.0, 0.0, False], "DOWN": [-1.0, 0.0, False], "S": [-1.0, 0.0, False], "s": [-1.0, 0.0, False],
            "NUMPAD_6": [0.0, -1.0, False], "RIGHT": [0.0, -1.0, False], "D": [0.0, -1.0, False], "d": [0.0, -1.0, False],
            "NUMPAD_4": [0.0, 1.0, False], "LEFT": [0.0, 1.0, False], "A": [0.0, 1.0, False], "a": [0.0, 1.0, False],
        }
        self._base_command = np.zeros(3)
        self.running = True
        self.exit = False

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
                self.exit = self._base_command[2]
                if self.exit:
                    self.running = False
                print("Key pressed: ", event.input.name)
                print("exit: ", self.exit)
                print("self.running: ", self.running)
                print("base_command: ", self._base_command)
        return True

    def setup(self):
        self.my_world.reset()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        
        # Signal that setup is complete by creating a marker file
        with open("/tmp/isaac_sim_setup_ready", "w") as f:
            f.write("Setup complete\n")
        print("Leo_rover setup completed, marker file created: /tmp/isaac_sim_setup_ready")

    def run(self):
        while self.running:
            kit.update()
        kit._app.close()

def main():
    rover = Leo_rover()
    rover.setup()
    rover.run()

if __name__ == "__main__":
    main()