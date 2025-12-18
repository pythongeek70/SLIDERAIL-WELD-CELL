import os
os.environ["WEBOTS_CONTROLLER_ALLOW_NEW_CONNECTION"] = "1"
from controller import Supervisor, Emitter, Receiver
import numpy as np
import json
import time
from scipy.spatial.transform import Rotation as R
from pynput import keyboard

# Initialize the Supervisor
supervisor = Supervisor()

# Time step
timestep = int(supervisor.getBasicTimeStep ())
#Initialize Emitters and set Channels
emitter_1 = supervisor.getDevice("emitter_1")
emitter_1.setChannel(1)
receiver_1 = supervisor.getDevice("receiver_1")
receiver_1.enable(timestep)
receiver_1.setChannel(1)

#Enable Keyboard Input
supervisor.keyboard.enable(timestep)
supervisor.step(timestep)

# Access The Root Node To Add New Spheres
root_children = supervisor.getRoot().getField("children")
# Dictionary to store absolute positions
absolute_positions = {}
orientations = {}

end_effector_node = supervisor.getFromDef("End_Effector")

                                   
STOP_1 = False  
STOP_2 = False
STOP_3 = False
STOP_4 = False
space_pressed = False
point_count = 0
sphere_count = 0 

def on_press(key):
    global space_pressed
    print("Key pressed:", key)
    if key == keyboard.Key.space:
        space_pressed = True
        print("Space Pressed = ", space_pressed)
        
    elif key == keyboard.Key.esc:
         return False
        
def on_release(key):
    global space_pressed, sphere_count    
    print("Key released:", key) 
    print("Space Pressed = ", space_pressed)
    
    if key == keyboard.Key.space and space_pressed:
        space_pressed = False
    
        end_effector_position = np.array(end_effector_node.getPosition()) 
        
        
        end_effector_orientation = np.array(end_effector_node.getOrientation()).reshape(3,3)
        #print(end_effector_orientation) 
        
        #Convert Orientation To Axis-Angle
        rotation = R.from_matrix(end_effector_orientation) 
        axis_angle = rotation.as_rotvec()   
        axis = axis_angle / np.linalg.norm(axis_angle)
        angle = np.linalg.norm(axis_angle)
        
        # Increment sphere count and create a new sphere
        sphere_count += 1
        sphere_name = f"Sphere({sphere_count})"
        new_sphere_proto = f"""
        DEF {sphere_name} Solid {{ 
          translation {end_effector_position[0]} {end_effector_position[1]} {end_effector_position[2]}
          rotation {axis[0]} {axis[1]} {axis[2]} {angle}
          children [
            Shape {{
              appearance Appearance {{
                material Material {{
                  diffuseColor 1 0 0
                }}
              }}
              geometry Sphere {{
                radius 0.05
              }}
            }}
          ]
        }}
        """
        root_children.importMFNodeFromString(-1, new_sphere_proto)
       
        
    elif key == keyboard.KeyCode(char='r'):
        message = {"label" : "run program"}
        print("Sending Message - Run Program") 
        serialized_message = json.dumps(message).encode('utf-8')
        emitter_1.send(serialized_message)
        
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start() 


####################################################################################
########                             Main Loop                              ########
####################################################################################

while supervisor.step(timestep) != -1:

    t = supervisor.getTime()


   # ✅ Process incoming messages from the robot
    if receiver_1.getQueueLength() > 0:
        message = receiver_1.getString().decode('utf-8')
        print(f"✅ Received from robot: {message}")
        receiver_1.nextPacket()  # ✅ Clears the received message
