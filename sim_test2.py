import pybullet as p
import time
import pybullet_data
import numpy as n

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.resetSimulation()
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,1])
#xarm = p.loadURDF("xarm/xarm6_with_gripper.urdf",startPos ,startOrientation ,useFixedBase = 1)
kuka = p.loadURDF("kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf",startPos ,startOrientation ,useFixedBase = 1)
objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
gripper = objects[0]

teddy = p.loadURDF("teddy_vhacd.urdf", 1.050000, -0.500000, 0.700000, 0.000000, 0.000000, 0.707107, 0.707107)
cube = p.loadURDF("cube_small.urdf", 0.950000, -0.100000, 0.700000, 0.000000, 0.000000, 0.707107, 0.707107)
sphere = p.loadURDF("sphere_small.urdf", 0.850000, -0.400000, 0.700000, 0.000000, 0.000000, 0.707107,0.707107)
duck = p.loadURDF("duck_vhacd.urdf", 0.850000, -0.400000, 0.900000, 0.000000, 0.000000, 0.707107,0.707107)


Orientation = p.getQuaternionFromEuler([3.14,0.,0.])

print("kuka joints: {}".format(p.getNumJoints(kuka)))

cubePos, cubeOrn = p.getBasePositionAndOrientation(duck)
print(cubePos)

def step():
  cubePos, cubeOrn = p.getBasePositionAndOrientation(duck)
  keys = p.getKeyboardEvents()
  if p.B3G_LEFT_ARROW in keys:
    print("test left keyyy ahhh")
    target_pos = p.calculateInverseKinematics(kuka, 7, [0.,0.,0.4],targetOrientation = Orientation)#targetOrientation = Orientation
    p.setJointMotorControlArray(kuka, range(7), p.POSITION_CONTROL, targetPositions=target_pos) 
  elif p.B3G_RIGHT_ARROW in keys:
    print("test right keyyy ahhh")
    target_pos = p.calculateInverseKinematics(kuka, 7, [0.3,0.1,0.4],targetOrientation = Orientation)#targetOrientation = Orientation
    p.setJointMotorControlArray(kuka, range(7), p.POSITION_CONTROL, targetPositions=target_pos)
  elif p.B3G_UP_ARROW in keys:  
    print("test right keyyy ahhh")
    target_pos = p.calculateInverseKinematics(kuka, 7, [0,0,0],targetOrientation = Orientation)#targetOrientation = Orientation
    p.setJointMotorControlArray(kuka, range(7), p.POSITION_CONTROL, targetPositions=target_pos)  
  elif p.B3G_DOWN_ARROW in keys:  
    print("test right keyyy ahhh")
    target_pos = p.calculateInverseKinematics(kuka, 7, cubePos,targetOrientation = Orientation)#targetOrientation = Orientation
    p.setJointMotorControlArray(kuka, range(7), p.POSITION_CONTROL, targetPositions=target_pos)  
  else:
    print("no input")

while(1):
    step()
    p.stepSimulation()
    time.sleep(1./240.)
'''
for i in range (10000):
  target_pos = p.calculateInverseKinematics(kuka, 7, [0.3,0.1,0.4],targetOrientation = Orientation)#targetOrientation = Orientation
  p.setJointMotorControlArray(kuka, range(7), p.POSITION_CONTROL, targetPositions=target_pos)
  p.stepSimulation()
  time.sleep(1/10)
  target_pos = p.calculateInverseKinematics(kuka, 7, [0.,0.,0.4],targetOrientation = Orientation)#targetOrientation = Orientation
  p.setJointMotorControlArray(kuka, range(7), p.POSITION_CONTROL, targetPositions=target_pos)
  p.stepSimulation()
  time.sleep(1/10)
  '''

p.disconnect()

