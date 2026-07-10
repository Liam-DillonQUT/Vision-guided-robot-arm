import pybullet as p
import pybullet_data
import time


physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

p.loadURDF("plane.urdf")
arm_id = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])

target_angles = [0.5,0.5,0.0,-1.0,0.0,1.0,0.0]

for joint_index in range(7):
    p.setJointMotorControl2(
        bodyIndex=arm_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_angles[joint_index],
        force=500
    )

# run simulation and read joint states every 60 steps, which is 4 times a second
for step in range(500):
    p.stepSimulation()
    time.sleep(1.0/240.0)

    if step % 60 == 0:
        print(f"\n--- Step {step} ---")
        #getJointState returns 4 values:
        # [0] = current position (angle/radians)
        # [1] = current velcoity (radians per second)
        # [2] = reaction forces
        # [3] = motor torque being applied
        for joint_index in range(7):
            joint_state = p.getJointState(arm_id,joint_index)
            current_angle = joint_state[0]
            current_velocity = joint_state[1]
            current_torque = joint_state[3]

            print(f"  Joint{joint_index}:"
                f"angle={current_angle:.3f} rad,"
                f"velocity={current_velocity:.3f} rad/s,"
                f"torque = {current_torque:.3f} Nm")

p.disconnect()
                  