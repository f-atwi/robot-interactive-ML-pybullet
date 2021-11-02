from qibullet import SimulationManager
from qibullet import RomeoVirtual
from qibullet import PepperVirtual
from qibullet import NaoVirtual
import time
import pybullet as p
import cv2

def main():
	simulation_manager = SimulationManager()
	client_id = simulation_manager.launchSimulation(gui=True)

	# Loading pepper
	pepper = simulation_manager.spawnPepper(client_id,spawn_ground_plane=True)
	# nao = simulation_manager.spawnNao(client_id,translation=[0, 2, 0],quaternion=[0, 0, 0, 1])
	# romeo = simulation_manager.spawnRomeo(client_id,translation=[0, 4, 0],spawn_ground_plane=False)

	p.connect(p.DIRECT)

	# Loading cube
	cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.125,0.125,0.125])
	cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25,0.25,0.25])
	cube_body = p.createMultiBody( baseMass=0, baseCollisionShapeIndex=cube_collision,baseVisualShapeIndex=cube_visual, basePosition = [2,1, 0.725])

	# Loading 3D objects
	p.loadURDF("../urdf/table/table.urdf", basePosition = [2,1,0], globalScaling = 1)
	p.loadURDF("../urdf/chair/chair.urdf", basePosition = [3,1,0], globalScaling = 1)
	p.loadURDF("../urdf/chair/chair.urdf", basePosition = [4,1,0], globalScaling = 1)

	# # Posture
	# pepper.goToPosture("Crouch", 0.6)
	# time.sleep(3)
	# pepper.goToPosture("Stand", 0.6)
	# time.sleep(3)
	# pepper.goToPosture("StandZero", 0.6)
	# time.sleep(5)

	# # Controlling joints
	# joint_parameters = list()

	# for name, joint in pepper.joint_dict.items():
	# 	if "Finger" not in name and "Thumb" not in name:
	# 		joint_parameters.append((p.addUserDebugParameter(name,joint.getLowerLimit(),joint.getUpperLimit(),pepper.getAnglesPosition(name)),name))
			
	# while True:
	# 	for joint_parameter in joint_parameters:pepper.setAngles(joint_parameter[1],p.readUserDebugParameter(joint_parameter[0]), 1.0)	

	# # Motion
	# pepper.moveTo(3,3,0,frame=2,_async=False)


if __name__ == "__main__":
	main()


