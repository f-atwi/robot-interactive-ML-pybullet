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
	p.loadURDF("./urdf/table/table.urdf", basePosition = [2,1,0], globalScaling = 1)
	p.loadURDF("./urdf/chair/chair.urdf", basePosition = [3,1,0], globalScaling = 1)
	p.loadURDF("./urdf/chair/chair.urdf", basePosition = [4,1,0], globalScaling = 1)

	# Subscrbing to cameras
	handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)
	handle2 = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)
	handle3 = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH)

	try:
		while True:
			img = pepper.getCameraFrame(handle)
			cv2.imshow("bottom camera", img)

			img2 = pepper.getCameraFrame(handle2)
			cv2.imshow("top camera", img2)

			img3 = pepper.getCameraFrame(handle3)
			cv2.imshow("depth camera", img3)

			filename="ImageDepth.png"
			cv2.imwrite(filename,img3)
			im_gray = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
			im_color = cv2.applyColorMap(im_gray, cv2.COLORMAP_HSV)
			cv2.imshow("colorBar depth camera", im_color)

			cv2.waitKey(1)
	except KeyboardInterrupt:
		simulation_manager.stopSimulation(client_id)
		cv2.destroyAllWindows()

if __name__ == "__main__":
	main()


