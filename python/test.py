from qibullet import SimulationManager
from qibullet import PepperVirtual
import numpy as np
import time
import pybullet as p
import cv2
import math
from poseDetection import PoseDetection

def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def main():
	simulation_manager = SimulationManager()
	client_id = simulation_manager.launchSimulation(gui=True)

	# Loading pepper
	pepper = simulation_manager.spawnPepper(client_id,spawn_ground_plane=True)
	# nao = simulation_manager.spawnNao(client_id,translation=[0, 2, 0],quaternion=[0, 0, 0, 1])
	# romeo = simulation_manager.spawnRomeo(client_id,translation=[0, 4, 0],spawn_ground_plane=False)

	p.connect(p.DIRECT)

	# Loading cube
	# cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.125,0.125,0.125])
	# cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25,0.25,0.25])
	# cube_body = p.createMultiBody( baseMass=0, baseCollisionShapeIndex=cube_collision,baseVisualShapeIndex=cube_visual, basePosition = [2,1, 0.725])

	# Loading 3D objects
	p.loadURDF("./urdf/table/table.urdf", basePosition = [1,0,0], baseOrientation = euler_to_quaternion(math.pi/2,0,0), globalScaling = 1)

	p.setGravity(0,0,-10)
	
	# Subscrbing to cameras
	# handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)
	handle2 = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
	# handle3 = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH)

	pd = PoseDetection()
	try:
		while True:
			img = pepper.getCameraFrame(handle2)
			img = pd.getPose(img)
			cv2.imshow("top camera", img)
			cv2.waitKey(1)



	except KeyboardInterrupt:
		simulation_manager.stopSimulation(client_id)
		cv2.destroyAllWindows()

if __name__ == "__main__":
	main()


