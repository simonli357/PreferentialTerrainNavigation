import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the base plane
planeId = p.loadURDF("plane.urdf")

# Load sidewalk
sidewalk = p.loadURDF("plane.urdf", [0, 0, 0])
p.changeDynamics(sidewalk, -1, lateralFriction=0.8, restitution=0.5)

# Load road
road = p.loadURDF("plane.urdf", [5, 0, 0])
p.changeDynamics(road, -1, lateralFriction=0.4, restitution=0.2)

# Load gravel
gravel = p.loadURDF("plane.urdf", [10, 0, 0])
p.changeDynamics(gravel, -1, lateralFriction=1.2, restitution=0.6)

# Load snow
snow = p.loadURDF("plane.urdf", [15, 0, 0])
p.changeDynamics(snow, -1, lateralFriction=0.3, restitution=0.1)

# Load textures
sidewalk_texture = p.loadTexture("/home/slsecret/PreferentialTerrainNavigation/bullet/assets/Snow011_1K-JPG/Snow011_1K-JPG_Color.jpg")
road_texture = p.loadTexture("/home/slsecret/PreferentialTerrainNavigation/bullet/assets/Snow011_1K-JPG/Snow011_1K-JPG_NormalGL.jpg")
gravel_texture = p.loadTexture("/home/slsecret/PreferentialTerrainNavigation/bullet/assets/Snow011_1K-JPG/Snow011_1K-JPG_Roughness.jpg")
snow_texture = p.loadTexture("/home/slsecret/PreferentialTerrainNavigation/bullet/assets/Snow011_1K-JPG/Snow011_1K-JPG_AmbientOcclusion.jpg")

# Assign textures to terrains
p.changeVisualShape(sidewalk, -1, textureUniqueId=sidewalk_texture)
p.changeVisualShape(road, -1, textureUniqueId=road_texture)
p.changeVisualShape(gravel, -1, textureUniqueId=gravel_texture)
p.changeVisualShape(snow, -1, textureUniqueId=snow_texture)

# Load a robot
husky = p.loadURDF("husky/husky.urdf", [0, 0, 0.5])

# Add gravity
p.setGravity(0, 0, -9.8)

# Simulate
for i in range(1000):
    p.stepSimulation()
    time.sleep(1 / 120.)

# Disconnect
p.disconnect()
