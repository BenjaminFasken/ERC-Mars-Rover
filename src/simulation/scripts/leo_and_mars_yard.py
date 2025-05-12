STREAMING = True #False

from isaacsim import SimulationApp

if STREAMING:
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

else:
    simulation_app = SimulationApp({"headless": False,"renderer": "RayTracedLighting",
    "exts": [
        "omni.isaac.ros2_bridge",
        "omni.isaac.core",
    ],})  # start the simulation app, with GUI open


if STREAMING:
    enable_extension("isaacsim.ros2.bridge")
else:
    print("Running in non-streaming mode. No ROS2 bridge enabled.\n")
    print("You can enable the ROS2 bridge by going to the menu bar and selecting Extensions > Isaac > ROS2 Bridge > Enable")
    ### Insert the code to enable the ROS2 bridge here if needed ##

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

class LeoDiffDriveController(BaseController):
    """
    Code for this controller is adapted from diff_drive_controller.cpp from Leo Rover firmware
    """
    def __init__(self):
        super().__init__(name="my_leo_controller")
        # An open loop controller that uses a unicycle model
        #scale: (1.0, 1.0),
        #offset: (0.0, 0.0),
        #self._wheel_radius: 0.1175
        #self._wheel_base = 0.470 #track_width

        """
        Defalt values for the Leo rover:
        """
        self._wheel_radius=0.0625
        self._wheel_separation=0.33
        self._angular_velocity_multiplier=1.91
        self._input_timeout=500
        self._encoder_resolution=878.4
        self._encoder_pullup=1
        self._max_speed=800.0 #The maximum reachable speed of the motors in encoder counts per second. Used>
        self._wheels_pid_p =0.0
        self._wheels_pid_i =0.005
        self._wheels_pid_d =0.0
        self._pwm_duty_limit=100.0 #The value should be between 0.0 and 100.0
        self._battery_min_voltage=10.0 # If the battery voltage drops below this value, the firmware will s>
        self._power_limit=1000.0 #Limit of the PWM duty applied to the motors. The value should be between >
        self._torque_limit=1000.0 #This value applies an additional power limit depending on the current sp>
        #wheel_joint_names (string list, default: [wheel_FL_joint, wheel_RL_joint, wheel_FR_joint, wheel_RR>
        #robot_frame_id (string, default: "base_link")
        #odom_frame_id (string, default: "odom")
        #imu_frame_id (string, default: "imu_frame")
        #tf_frame_prefix (string, default: "")
        #wheel_odom_twist_covariance_diagonal (float list, default: [0.0001, 0.0, 0.0, 0.0, 0.0, 0.001])
        #imu_angular_velocity_covariance_diagonal (float list, default: [0.000001, 0.000001, 0.00001])
        #imu_linear_acceleration_covariance_diagonal (float list, default: [0.001, 0.001, 0.001])


        self.maxLinearSpeed = 0.5
        self.maxAngularSpeed = 1.2
        return
    
    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        #%joint_velocities = [0.0, 0.0, 0.0, 0.0]
        #joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_separation)) / (2 * self._whee>
        #joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_separation)) / (2 * self._whee>
        # A controller has to return an ArticulationAction
        return #ArticulationAction(joint_velocities=joint_velocities)

    def setSpeed(self, linear_x, angular_z):
        angular_multiplied = angular_z * self._angular_velocity_multiplier
        wheel_L_lin_vel = linear_x - (angular_multiplied * self._wheel_separation / 2.0)
        wheel_R_lin_vel = linear_x + (angular_multiplied * self._wheel_separation / 2.0)
        wheel_L_ang_vel = wheel_L_lin_vel / self._wheel_radius
        wheel_R_ang_vel = wheel_R_lin_vel / self._wheel_radius

        wheel_FL=wheel_L_ang_vel
        wheel_BL=wheel_L_ang_vel
        wheel_FR=wheel_R_ang_vel
        wheel_BR=wheel_R_ang_vel

        wheel_velocities=[wheel_FL, wheel_BL, wheel_FR, wheel_BR] # Leo rove in isaac sim wheel setup [FL, >
        print("wheel_velocities: ", wheel_velocities)
        return wheel_velocities

class Leo_rover(object):
    def __init__(self) -> None:#,physics_dt, render_dt) -> None:
        """
        Creates the simulation world with a Leo rover on default ground plane and sets up keyboard listener>


        """
        # Create a world object and add a ground plane to the scene
        self.my_world = World(stage_units_in_meters=1.0)

        # Path to your USD file
        if STREAMING:
            usd_path = "/isaac-sim/assets/mars_yard.usd"
        else:
            #usd_path = "/home/ehb/isaacsim/assets/mars_yard.usd"
            usd_path = "/home/ehb/isaacsim/assets/Test_folder/Mars_Yard.usd"
        
        

        # Add the USD scene to the world
        stage_utils.add_reference_to_stage(usd_path, "/World/Mars_yard")

        # Get the prim for Mars Yard
        stage = self.my_world.stage
        terrain_prim = stage.GetPrimAtPath("/World/Mars_yard")

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
        if STREAMING:
            asset_path = "/isaac-sim/assets/leo_rover.usd"
        else:
            asset_path = "/home/ehb/isaacsim/assets/leo_rover/leo_rover.usd"
        print("asset_path: ", asset_path)
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/leo_rover")  # add robot to stage

        self.leo_rover=self.my_world.scene.add(WheeledRobot(
                prim_path="/World/leo_rover",
                name="my_leo_rover",
                #wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],#["bgl","bgr","fl", "bl","fr","b>
                wheel_dof_indices=[2, 3, 4, 5],# [0, 1, 2, 3, 4, 5],
                create_robot=True,
                usd_path=asset_path,
                position = np.array([0.0,-1.625,0.0]) #x,y,z
                #,orientation = R.from_euler('xyz', [np.pi, 0.0, 0.0]).as_quat()
            ))

        #self.rover.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / get_stage_units())



        # Bindings for keyboard to command
        self._input_keyboard_mapping = {
            # exit commands
            "Q": [0.0, 0.0, True],  # 3th element indicates exit request
            #"ESCAPE": [0.0, 0.0, True],
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

        self.leo_wheel_velocities = np.zeros(4)
        self.linear_x=0.0
        self.angular_z=0.0
        self.lin_x_multiplier=1.0
        self.ang_z_multiplier=1.0
        self.exit=False


    def setup(self) -> None:
        """
        Set up keyboard listener
        """
        # Initialize the world
        self.my_world.reset()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()

        leo_controller = LeoDiffDriveController()
        self.lin_x_multiplier=leo_controller.maxLinearSpeed
        self.ang_z_multiplier=leo_controller.maxAngularSpeed
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        #self._leo_articulation_controller = self.leo_rover.get_articulation_controller()
        self.my_world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)

        # print wheel ids
        print("wheel_dof_names: " + str(self.leo_rover._wheel_dof_indices)) # prints [0, 1, 2, 3, 4, 5]


        #print("Num of degrees of freedom after first reset: " + str(self.leo_rover.num_dof)) # prints 2

        #print("Joint Positions after first reset: " + str(self.leo_rover.get_joint_positions()))    

    def send_robot_actions(self, step_size):
        # Every articulation controller has apply_action method
        # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
        # as optional args. It accepts numpy arrays of floats OR lists of floats and None
        # None means that nothing is applied to this dof index in this step
        # ALTERNATIVELY, same method is called from self._jetbot.apply_action(...)
        # Create a 6-element array with random values

        #velocities=np.array([3.0, 3.0, 5.0, 5.0])
        # bugy1, bugy2_l, FL, BL,FR,BR
        #print(velocities)
        self.leo_rover.apply_wheel_actions(ArticulationAction(joint_velocities=self.leo_wheel_velocities))
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback
        """
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
                self.linear_x=self._base_command[0]*self.lin_x_multiplier
                self.angular_z=self._base_command[1]*self.ang_z_multiplier
                self.exit=self._base_command[2]
                self.leo_wheel_velocities = LeoDiffDriveController().setSpeed(self.linear_x, self.angular_z)

                # Check if exit is requested
                if self.exit==True:
                    self.running = False


        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
                self.linear_x=self._base_command[0]*self.lin_x_multiplier
                self.angular_z=self._base_command[1]*self.ang_z_multiplier
                self.exit=self._base_command[2]
                self.leo_wheel_velocities = LeoDiffDriveController().setSpeed(self.linear_x, self.angular_z)

                # Check if exit is requested
                if self.exit==True:
                    self.running = False


        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
                self.linear_x=self._base_command[0]*self.lin_x_multiplier
                self.angular_z=self._base_command[1]*self.ang_z_multiplier
                self.leo_wheel_velocities = LeoDiffDriveController().setSpeed(self.linear_x, self.angular_z)
                print("base_command: ", self._base_command)
        return True

    def run(self):
        """
        Main simulation loop
        """
        if STREAMING:
            while self.running:
            # Run in realtime mode, we don't specify the step size
                kit.update()

            kit._app.close()
        else:
            while self.running:
                # Step the simulation, both rendering and physics
                self.my_world.step(render=True)
            simulation_app.close()




def main():

    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0
    # Initialize and run the rover controller
    #rover = Leo_rover(physics_dt=physics_dt, rendering_dt=render_dt)
    rover = Leo_rover()
    rover.setup()
    #leo_controller = LeoDiffDriveController()
    #print("Num of degrees of freedom after first reset: " + str(rover.num_dof)) # prints 2
    #print("Joint Positions after first reset: " + str(rover.get_joint_positions()))
    #rover.setup_post_load()
    rover.run()
    
    


if __name__ == "__main__":
    main()
