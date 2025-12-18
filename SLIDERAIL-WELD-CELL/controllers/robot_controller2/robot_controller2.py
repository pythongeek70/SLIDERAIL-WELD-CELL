import controller
import json
import ikpy
import numpy as np
import time
from controller import Supervisor, Emitter, Receiver, PositionSensor, VacuumGripper, Connector
from scipy.spatial.transform import Rotation as R
from ikpy.chain import Chain
from pathlib import Path

# Initialize Supervisor & Robot
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

gripper = supervisor.getDevice("vacuum gripper")
connector = supervisor.getDevice("connector")
     
if gripper is None:
    print("❌ Error: Gripper not found.")
else:
    print("✅ Gripper found.")

if connector is None:
    print("❌ Error: Connector not found.")
else:
    print("✅ Connector found.")
    connector.enablePresence(timestep)

# Initialize Receiver & Emitter
receiver_1 = supervisor.getDevice("receiver_1")
receiver_1.enable(timestep)
receiver_1.setChannel(1)
emitter_1 = supervisor.getDevice("emitter_1")
emitter_1.setChannel(1)

# Objects to Track
objects_to_track = ["ROBOT_A", "End_Effector", "Guide_Ball", "X_Mem_1"]
absolute_position, orientation = {}, {}

for obj in objects_to_track:
    node = supervisor.getFromDef(obj)
    if node:
        absolute_position[obj] = node.getPosition()
        orientation[obj] = node.getOrientation()

urdf_path = Path(__file__).resolve().parents[2] / "urdf" / "ROBOT_A.urdf"
armChain = Chain.from_urdf_file(str(urdf_path), active_links_mask=[False, True, True, True, True, True, True, False])

# Joint Configuration
joint_names = [f"Joint_{i}" for i in range(1, 7)]
sensor_names = [f"Joint_{i}_sensor" for i in range(1, 7)]
vel = 0.5
threshold = 0.0055

robot_joints, joint_sensors = [], []

for joint_name, sensor_name in zip(joint_names, sensor_names):
    motor = supervisor.getDevice(joint_name)
    sensor = supervisor.getDevice(sensor_name)
    if motor and sensor:
        motor.setPosition(0.0)
        motor.setVelocity(vel)
        sensor.enable(timestep)
        robot_joints.append(motor)
        joint_sensors.append(sensor)

# Counting Functions     
def count_spheres():
    print("Inside 'Count Spheres' function.")
    root_children = supervisor.getRoot().getField("children")
    sphere_count = 0
     
    for i in range(root_children.getCount()):
        child_node = root_children.getMFNode(i)
        if child_node:
            #Check the DEF name to match "Sphere" prefix
            if child_node.getDef() and child_node.getDef().startswith("Sphere"):
                sphere_count += 1
             
    print(f"Number of Spheres: {sphere_count}")
    return sphere_count
    
def count_x_mem():
    root_children = supervisor.getRoot().getField("children")
    x_mem_count = 0
    
    for i in range(root_children.getCount()):
        child_node = root_children.getMFNode(i)
        if child_node:
            if child_node.getDef() and child_node.getDef().startswith("X_Mem"):
                x_mem_count += 1 
    #print(f"Number of Cross Members: {x_mem_count}")            
    return x_mem_count   

# Movement Functions
def has_reached_target(current_position, target_position, threshold):
    pos_error = np.linalg.norm(np.array(current_position) - np.array(target_position))
    return pos_error < threshold
     
def get_joint_positions():
    return [sensor.getValue() for sensor in joint_sensors]   
    
def goto(position, orientation, tolerance, max_attempts=30):
    rel_position = np.array(position) - np.array(absolute_position["ROBOT_A"])
    orientation = np.array(orientation).reshape(3, 3)
    
    for attempt in range(max_attempts):
        ikSolution = armChain.inverse_kinematics(
            rel_position, target_orientation=orientation, orientation_mode="all"
        )
        for i in range(len(robot_joints)):
            robot_joints[i].setPosition(ikSolution[i+1])
            
        supervisor.step(timestep*10)

        if has_reached_target(absolute_position["End_Effector"], position, tolerance):
            return
  
    
def pick_up():
    """Attach X_Mem_1 to End_Effector using a Connector."""
    connector.enablePresence(5000)
    print(f"Get Presence:  {connector.getPresence()}")
    if connector and connector.getPresence():
        connector.lock()  # Engage the connector
        print("✅ X_Mem_1 successfully attached to End_Effector using Connector.")
    else:
        print("❌ Error: No object detected for attachment.")
    
    gripper.turnOn()
    gripper.isOn = True
    supervisor.step(timestep * 10)
    
    
def release():
    """Detach the cross member from the gripper."""
    x_mem_joint = supervisor.getFromDef("X_Mem_Joint")
    
    if x_mem_joint:
        x_mem_joint.getField("jointParameters").setSFBool(False)  # Disable joint control
        print("✅ X_Mem_1 successfully released from End_Effector.")
    
    if gripper:
        gripper.turnOff()
        gripper.isOn = False
        print("✅ Gripper released.")
        supervisor.step(timestep * 10)




# Main Loop
while supervisor.step(timestep) != -1:
    for obj in objects_to_track:
        node = supervisor.getFromDef(obj)
        if node:
            absolute_position[obj] = node.getPosition()
            orientation[obj] = node.getOrientation()
    
    goto(absolute_position["Guide_Ball"], orientation["Guide_Ball"], threshold)
    
    if receiver_1.getQueueLength() > 0:
        data = json.loads(receiver_1.getString())
        receiver_1.nextPacket()
        
        if data["label"] == "run program":
            print("Message Received - Run Program")
            for i in range(1, count_spheres() + 1):
                
                sphere_node = supervisor.getFromDef(f"Sphere({i})")
                if sphere_node:
                    sphere_position = sphere_node.getPosition()
                    print(f"i: {i}")
                    print(f"Sphere Position: {sphere_position}")
                    sphere_orientation = sphere_node.getOrientation()                            
                                
                # Move until the end effector reaches the sphere
                    if np.linalg.norm(np.array(absolute_position["End_Effector"]) - np.array(sphere_position)) > threshold:
                        
                        goto(sphere_position, sphere_orientation, threshold) 
                        supervisor.step(timestep)
    
                    # ✅ Ensure the latest position is always used
                        absolute_position["End_Effector"] = supervisor.getFromDef("End_Effector").getPosition()
    
                        print(f'📡 Updating End Effector Position: {absolute_position["End_Effector"]}')
                        goto(sphere_position, sphere_orientation, threshold)
                  # Check if end effector is close enough to attach cross member
                    
                        sphere = supervisor.getFromDef(f"Sphere({i})")
                        sphere_position = sphere.getPosition()
                        sphere_orientation = sphere.getOrientation()
                        shape = sphere.getField("children").getMFNode(0)
                        appearance = shape.getField("appearance").getSFNode()
                        material = appearance.getField("material").getSFNode()
                        sphere_color = material.getField("diffuseColor").getSFVec3f()
                        sphere_err = np.linalg.norm(np.array(absolute_position["End_Effector"]) - np.array(sphere_position))
                        print(f'End Effector Position: {absolute_position["End_Effector"]}')
                        print(f"Sphere Error: {sphere_err}")
                        if sphere_color == [1, 1, 0] and sphere_err < threshold:
                            #print(f"Cross Member Position: {x_mem_position}")
                            print("Attaching Cross Member...")
                            pick_up()  # ✅ Now using  VacuumGripper
                            supervisor.step(10)
                        if sphere_color == [0, 1, 0] and sphere_err < threshold:
                           print("Releasing Crossmember")
                           release()
                        
