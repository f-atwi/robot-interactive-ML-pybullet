import cv2
import time
import mediapipe as mp


class PoseDetection:
    def __init__(self,
               static_image_mode=False,
               model_complexity=1,
               smooth_landmarks=True,
               enable_segmentation=False,
               smooth_segmentation=True,
               min_detection_confidence=0.5,
               min_tracking_confidence=0.5):
        
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose 
        self.pose = self.mp_pose.Pose(static_image_mode,
                                    model_complexity,
                                    smooth_landmarks,
                                    enable_segmentation,
                                    smooth_segmentation,
                                    min_detection_confidence,
                                    min_tracking_confidence)

    def  getPose(self, image, draw = True):
        image_rgb = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        if draw and results.pose_landmarks:
            self.mp_draw.draw_landmarks(image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        return image
        







def main():
    pass


if __name__ == "__main__":
    main()