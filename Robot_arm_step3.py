import pybullet as p
import pybullet_data
import time


physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

p.loadURDF("plane.urdf")
arm_id = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])

#set target angles for each of the 7 joints in radians
# 0 = straight/default, can be changed to see diferent angle

target_angles = [0.5,0.5,0.0,-1.0,0.0,1.0,0.0]

#command each join to move to its target angle using position control
#position control is like a servo motor

for joint_index in range(7):
    p.setJointMotorControl2(
        bodyIndex=arm_id,
        jointIndex = joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition = target_angles[joint_index],
        force = 500 #mac force the motor can apply in newtons
    )
#set simulation run time to allow for movement
for _ in range (1000):
    p.stepSimulation()
    time.sleep(1.0/240.0)

p.disconnect()