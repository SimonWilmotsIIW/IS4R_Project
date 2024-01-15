#! /bin/python3.10
import pybullet as p
import pybullet_data
from robot_descriptions.loaders.pybullet import load_robot_description
import math, time, os, random

PHYSICS_CLIENT_ID = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

NPCs = []
def loadNPCs(npc_count):
    for i in range(npc_count):
        npc = p.loadURDF("r2d2.urdf", [random.uniform(-10, 10), random.uniform(-10, 10), 1])
        NPCs.append(npc)

def moveNPCs(NPCs):
    for npc in NPCs:
        rand_pos = [random.uniform(-10, 10), random.uniform(-10, 10), 1]
        curr_pos, curr_rot = p.getBasePositionAndOrientation(npc)
        rand_pos2 = [curr_pos[0] + random.uniform(-.1, .1), curr_pos[1] + random.uniform(-.1, .1), 1]
        p.resetBasePositionAndOrientation(npc, rand_pos2, [0, 0, 0, 1])

def setup_lidar():
    rayFrom = []
    rayTo = []
    rayIds = []

    numRays = int(1024/1)

    rayLen = 13

    rayHitColor = [1, 0, 0]
    rayMissColor = [0, 1, 0]

    replaceLines = True

    for i in range(numRays):
        rayFrom.append([0, 0, 0.5])
        rayTo.append([
            rayLen * math.sin(2. * math.pi * float(i) / numRays),
            rayLen * math.cos(2. * math.pi * float(i) / numRays), 1
        ])
        if (replaceLines):
            rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))
        else:
            rayIds.append(-1)
    return rayFrom, rayTo, rayIds, numRays, rayMissColor, rayHitColor, replaceLines

def setup_camera(width=128, height=128, fov=60, near=0.02, far=1):
    aspect = width / height

    view_matrix = p.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [1, 0, 0])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    # Get depth values using the OpenGL renderer
    images = p.getCameraImage(width,height, view_matrix, projection_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.

# Set up Mars-like parameters
earth_gravity = -9.81    # Gravity in m/s^2
mars_gravity = -3.72076  # Gravity in m/s^2
p.setGravity(0, 0, earth_gravity)

mars_terrain = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
#stadium = p.loadSDF("stadium.sdf")

#rover = p.loadURDF("./simon_rover.urdf")
#rover = p.loadURDF("./rover/m2020.urdf")
rover = p.loadURDF("./robots/husky/huskyV2.urdf")
#rover = p.loadURDF("./robots/humanoid/humanoid.urdf")
#rover = p.loadURDF("/laikago/laikago.urdf") #/aliengo/aliengo.urdf
#rover = p.loadURDF("/kuka_iiwa/model.urdf")
#rover = p.loadURDF("cartpole.urdf")

loadNPCs(npc_count=5)

rayFrom, rayTo, rayIds, numRays, rayMissColor, rayHitColor, replaceLines = setup_lidar()

start_time = time.time()

while 1 + 1 != 3:
    p.stepSimulation()

    time.sleep(1/240)
    elapsed_time = time.time() - start_time


    print(f">> Time: {str(round(elapsed_time,1))}")

    # lidar_results = (hitObjectId, linkIndexOfHitObject, hitFraction? [0-1], hitPosition, hitNormal)
    lidar_results = p.rayTestBatch(rayFrom, rayTo, PHYSICS_CLIENT_ID)

    # Uncomment to temp. remove laser rays
    # p.removeAllUserDebugItems(physicsClientId=PHYSICS_CLIENT_ID)

    if (not replaceLines):
        p.removeAllUserDebugItems()

    for i in range(numRays):
        hitObjectUid = lidar_results[i][0]

        if (hitObjectUid < 0):
            hitPosition = [0, 0, 0]
            p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
        else:
            hitPosition = lidar_results[i][3]
            p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])
    
    moveNPCs(NPCs=NPCs)


p.disconnect()
