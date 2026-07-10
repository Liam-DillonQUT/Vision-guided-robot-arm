import pybullet as p
import pybullet_data
import time
import random
import cv2
import numpy as np

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
arm_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0.65])
table_id = p.loadURDF("table/table.urdf", basePosition=[0.5,0,0],
                      baseOrientation=p.getQuaternionFromEuler([0,0,1.5707]))
table_height = 0.65
sphere_radius = 0.03


end_effector_link = 6

def generate_sphere_positions(colors, x_range=(0.35, 0.65), y_range=(-0.25, 0.25), min_dist=0.12):
    positions = []
    for _ in colors:
        for attempt in range(100):
            x = random.uniform(*x_range)
            y = random.uniform(*y_range)
            # keep away from the drop zone corner
            if x > 0.42 and y > 0.15:
                continue
            # ensure minimum spacing from already-placed spheres
            if all(((x - px) ** 2 + (y - py) ** 2) ** 0.5 >= min_dist for px, py in positions):
                positions.append((x, y))
                break
        else:
            positions.append((x, y))  # fallback if 100 attempts fail
    return positions

sphere_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
random_positions = generate_sphere_positions(sphere_colors)

spheres = [
    {"pos": [px, py, table_height], "color": color}
    for (px, py), color in zip(random_positions, sphere_colors)
]

def reset_to_neutral():
      neurtal = [0,0,0,0,0,0,0]
      for j in range(7):
            p.resetJointState(arm_id,j,neurtal[j])

def get_camera_image():
      #positions camera above the table looking straight down
      camera_eye = [0.5,0.0,1.5]
      camera_target = [0.5,0.0,0.0]
      camera_up = [0.0,1.0,0.0]
      
      view_matrix = p.computeViewMatrix(camera_eye,camera_target,camera_up)

      #projection matrix defines the needed field of view
      projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, #field of view in degrees
            aspect=1.0, #square image
            nearVal=0.1, #closest variable distance
            farVal=10.0 #futherest variable distance
      )

      width, height = 640,640

      #grab imgae from renderer
      #retursn width, height, pixels, depth and color

      _,_, rgba, _, _ = p.getCameraImage(
            width,height,
            viewMatrix=view_matrix,
            projectionMatrix = projection_matrix
      )

      #convert RGBA to BGR (openCv used BGR not RGB)
      rgba_array = np.array(rgba,dtype=np.uint8).reshape(height,width,4)
      bgr=cv2.cvtColor(rgba_array, cv2.COLOR_RGBA2BGR)
      return bgr
def check_quit():
    keys = p.getKeyboardEvents()
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        print ("Q pressed - exiiting simulation")
        p.disconnect()
        exit()



def detect_spheres(image):
    #convert BGR to HSV, better for color dettection, seperates colour from brightness
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    #define colour ranges in HSV for each sphere
    #Format: [Lower Bound, Upper bound] for hue, saturation, value
    colours ={
        "red":   ([0, 120, 70],   [10, 255, 255]),
        "green": ([40, 50, 50],   [80, 255, 255]),
        "blue":  ([100, 150, 50], [140, 255, 255]),
    }

    detected = {}


    for colour_name,(lower,upper) in colours.items():
        lower=np.array(lower)
        upper=np.array(upper)

        #create a mask, white where colour matches black everywhere else
        mask = cv2.inRange(hsv,lower,upper)

        #find contours (outlines) of white regions in mask
        contours, _ =cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        if contours:
                #take largets contour value (will be sphere)
                largest = max(contours,key=cv2.contourArea)
                area = cv2.contourArea(largest)

                #only count it if its big enough to be a real sphere
                if area>100:
                    M=cv2.moments(largest)
                    if M["m00"] > 0:
                            cx = int(M["m10"]/M["m00"])
                            cy = int(M["m01"]/M["m00"])
                            detected[colour_name] = (cx,cy,area)
                            print(f" Detected {colour_name} sphere at pixel({cx}, {cy}),area = {area:.0f}px")
    return detected
                              
def pixel_to_world(pixel_x, pixel_y, image_width=640, image_height=640):
    # Camera is at height 1.5m looking down at table (height 0.655m)
    # Camera FOV is 60 degrees, image is 640x640
    # real world coordinates are calcuated

    camera_height = 1.5
    table_height = 0.655
    fov_degrees = 60
    camera_target_x = 0.5  # where camera is pointing in world X
    camera_target_y = 0.0  # where camera is pointing in world Y

    # How much real world distance each pixel represents
    distance_to_table = camera_height - table_height
    fov_radians = np.radians(fov_degrees)
    world_width = 2 * distance_to_table * np.tan(fov_radians / 2)
    metres_per_pixel = world_width / image_width

    # Convert pixel coords (origin top-left) to world coords
    # Pixel centre of image maps to camera_target
    world_x = camera_target_x + (pixel_x - image_width/2) * metres_per_pixel
    world_y = camera_target_y + (image_height/2 - pixel_y) * metres_per_pixel

    return world_x, world_y

def grab_sphere(sphere_id):
    ee_state = p.getLinkState(arm_id, end_effector_link)
    ee_pos = ee_state[0]

    constraint_id = p.createConstraint(
        parentBodyUniqueId=arm_id,
        parentLinkIndex=end_effector_link,
        childBodyUniqueId=sphere_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
    print(f"  Grabbed sphere {sphere_id}")
    return constraint_id


def release_sphere(constraint_id):
    # Remove the constraint - sphere drops with gravity
    p.removeConstraint(constraint_id)
    print(f"  Released sphere")

def check_in_container(sphere_id, container_pos, half_extents):
    pos, _ = p.getBasePositionAndOrientation(sphere_id)
    dx = abs(pos[0] - container_pos[0])
    dy = abs(pos[1] - container_pos[1])
    in_bounds = dx <= half_extents[0] and dy <= half_extents[1]
    print(f"  Placement check: sphere at ({pos[0]:.3f}, {pos[1]:.3f}) — "
          f"{'INSIDE' if in_bounds else 'MISSED'} container")
    return in_bounds

def move_to_position(target_pos):
    target_orientation = p.getQuaternionFromEuler([0,3.14159,0])

    joint_angles = p.calculateInverseKinematics(
        arm_id,
        end_effector_link,
        target_pos,
        target_orientation,
        lowerLimits=[-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054],
        upperLimits=[2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054],
        jointRanges=[5.934, 4.188, 5.934, 4.188, 5.934, 4.188, 6.108],
        restPoses=[0, -0.5, 0, -1.0, 0, 0.5, 0],
        maxNumIterations=200,
        residualThreshold=0.0001
    )
    print(f"  IK solved angles: {[f'{a:.3f}' for a in joint_angles[:7]]}")

    for j in range(7):
        p.setJointMotorControl2(bodyIndex=arm_id, jointIndex=j,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_angles[j], force=500, maxVelocity=1.0)

    for step in range(2000):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
        check_quit()
        if step % 10 == 0:
            velocities = [abs(p.getJointState(arm_id, j)[1]) for j in range(7)]
            if max(velocities) < 0.001:
                print(f"  Settled after {step} steps")
                break
    else:
        print(f"  WARNING: did not settle. Max joint velocity: {max(velocities):.4f} rad/s")
        print(f"  Velocities per joint: {[f'{v:.4f}' for v in velocities]}")
    
    contacts = p.getContactPoints(bodyA=arm_id)
    if contacts:
        for c in contacts:
            print(f"  COLLISION: link {c[3]} touching body {c[2]} at {c[6]}")
    else:
        print("  No collisions detected")

    return p.getLinkState(arm_id, end_effector_link)[0]

def print_error(target, actual):
    for axis, t, a in zip(['X','Y','Z'], target, actual):
        print(f"  {axis} error: {abs(t-a)*1000:.2f} mm")




sphere_ids = []
for sphere in spheres:
    col = p.createCollisionShape(p.GEOM_SPHERE, radius = sphere_radius)
    vis = p.createVisualShape(p.GEOM_SPHERE,radius=sphere_radius, rgbaColor=sphere["color"])
    sid = p.createMultiBody(baseMass=0.1,baseCollisionShapeIndex=col,baseVisualShapeIndex=vis,basePosition=sphere["pos"])

    sphere_ids.append(sid)
    print(f"Spawned sphere at {sphere['pos']}")

#Create a flat box as a visual drop zone marker
drop_zone_pos = [0.5,0.4,0.625]
drop_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.001],
                                rgbaColor=[1, 1, 0, 0.5])
drop_zone_id = p.createMultiBody(baseMass=0,baseVisualShapeIndex=drop_vis,
                                  basePosition=drop_zone_pos)
print(f"Drop zone created at {drop_zone_pos}")

def create_container_wall(pos, half_extents):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.6, 0.6, 0.6, 1])
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                              baseVisualShapeIndex=vis, basePosition=pos)

wall_h = 0.06  # wall height
cx, cy, cz = drop_zone_pos
create_container_wall([cx + 0.08, cy, cz + wall_h], [0.005, 0.08, wall_h])  # right
create_container_wall([cx - 0.08, cy, cz + wall_h], [0.005, 0.08, wall_h])  # left
create_container_wall([cx, cy + 0.08, cz + wall_h], [0.08, 0.005, wall_h])  # back
create_container_wall([cx, cy - 0.08, cz + wall_h], [0.08, 0.005, wall_h])  # front

print("Waiting...")
for _ in range(200):
    p.stepSimulation()
    time.sleep(1.0/240.0)


print("\nSphere positions after settling:")
for i, sid in enumerate (sphere_ids):
    pos, _ = p.getBasePositionAndOrientation(sid)
    print(f" Sphere {i}: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")

print("Capturing camera image...")
image = get_camera_image()
detected = detect_spheres(image)

# Draw detections on image
for colour, (cx, cy, area) in detected.items():
    cv2.circle(image, (cx, cy), 15, (255, 255, 255), 2)
    cv2.putText(image, colour, (cx-20, cy-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

cv2.imshow("Detected Spheres", image)
cv2.waitKey(3000)
cv2.destroyAllWindows()

#STATE MACHINE PYTHON: PICK AND PLACE EACH SPHERE

hover_z = 0.655+0.15 #15cm above table surface
grab_z = 0.655+sphere_radius+0.01 #sphere top height
drop_hover_pos = [drop_zone_pos[0], drop_zone_pos[1], drop_zone_pos[2] + 2*wall_h + 0.10]
drop_release_z = drop_zone_pos[2] + 0.05  # 5cm above container floor
drop_pos = [drop_zone_pos[0], drop_zone_pos[1], drop_release_z]

#map deetcted colour name to sphere_id
colour_to_sphere_id = {}
for colour, (cx,cy,are) in detected.items():
    world_x,world_y = pixel_to_world(cx,cy)
    #match deettction to nearest sphere
    best_id= None
    best_dist = None
    for sid, sphere_info in zip(sphere_ids,spheres):
        sx,sy,sz = sphere_info["pos"]
        dist = ((sx-world_x) ** 2+(sy - world_y) **2) **0.5
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_id = sid
    colour_to_sphere_id[colour] = best_id
print ("\nStatrting pick-and-place state machine...")

for colour, (cx,cy,area) in detected.items():
     sphere_id = colour_to_sphere_id[colour]
     world_x,world_y = pixel_to_world(cx,cy)

     print(f"\n=== Processing {colour} sphere (id={sphere_id})===")

     #State: Hover
     print("State: HOVER")
     move_to_position([world_x,world_y,hover_z])

     #State: DESCEND
     print("State: DESCEND")
     move_to_position([world_x,world_y,grab_z])

     #Stat: GRAB
     print("State: GRAB")
     constraint_id = grab_sphere(sphere_id)

     #State: LIFT
     print("State: LIFT")
     move_to_position([world_x,world_y,hover_z])

     #STATE: MOVE TO DROP
     print("State: Move_to_Drop")
     midpoint = [(world_x + drop_hover_pos[0]) / 2,
                 (world_y + drop_hover_pos[1]) / 2,
                 hover_z + 0.10]
     move_to_position(midpoint)
     move_to_position(drop_hover_pos)
     move_to_position(drop_pos)

     #STATE RELEASE
     print("State: Release")
     release_sphere(constraint_id)

     #State RESET
     for _ in range(100):
          p.stepSimulation()
          time.sleep(1.0/240.0)

     check_in_container(sphere_id, drop_zone_pos, [0.08, 0.08])

print("\n All Spheres Moved")
p.disconnect()