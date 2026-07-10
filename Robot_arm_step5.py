import pybullet as p
import pybullet_data
import time

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

p.loadURDF("plane.urdf")
arm_id = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])

target_angles = [0.5,0.5,0.0,-1.0,0.0,1.0,0.0]

for joint_index in range (7):
    p.setJointMotorControl2(
        bodyIndex=arm_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angles[joint_index],
        force=500

    )
#give arm time to sett before reading psoition
    for step in range(300):
        p.stepSimulation()
        time.sleep(1.0/240.0)


#get link state returns state fo specific link
#end-effector is the last link or the arm
#[0] is world position (x,y,z)
#[1] is work orientation (x,y,z,w)
end_effector_state = p.getLinkState(arm_id,6)

position = end_effector_state[0]
orientation = end_effector_state[1]

print(f"End-effector position:")
print(f" X = {position[0]:.4f} m")
print(f" Y = {position[1]:.4f} m")
print(f" Z = {position[2]:.4f} m")
print(f"End-effector orientation:")
print(f" X={orientation[0]:.3f}, Y = {orientation[1]:.3f},"
      f"Z = {orientation[2]:.3f}, W = {orientation[3]:.3f}")


#change the pose to see the position difference
print(f"\n Changing Pose...")

new_angles = [0.0,0.8,0.0,-0.8,0.0,0.5,0.0]
for joint_index in range(7):
    p.setJointMotorControl2(
        bodyIndex=arm_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=new_angles[joint_index],
        force=500
    )
for step in range(300):
    p.stepSimulation()
    time.sleep(1.0/240.0)


end_effector_state = p.getLinkState(arm_id, 6)
position = end_effector_state[0]

print(f"New end-effector position:")
print(f"  X = {position[0]:.4f} m")
print(f"  Y = {position[1]:.4f} m")
print(f"  Z = {position[2]:.4f} m")

p.disconnect()