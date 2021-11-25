# robot-interactive-machine-learning-pybullet

An interactive machine learning application on a virtual robot within a pybullet simulated environment

A Pepper robot uses a camera to capture and mimic the arm movements of another Pepper robot.

## Required libraries

* tensorflow 2.6.0
* qibullet 1.4.3
* opencv-contrib-python 4.5.4.58
* scikit-learn 1.0.1

## The code

### poseDetection.py

A custom class that uses the mediapipe library for pose detection

### generate_dataset.ipynb

Generates dataset for training.
It spawns two Pepper robots facing each other. One uses its camera to track the arm movement of the second, the second tracks the arms position it does.
Three landmarks are used for each side (Shoulder, Elbow, Wrist) making then a total of 6 points each with 3 coordinates.
Eight angles are recorded by the second Pepper corresponding to arm movement.
The dataset is created in a numpy array and then saved as an ```dataset<dataset_size>.npz``` where .npz is a zipped numpy format.

### train_model.ipynb

Creates a sequential keras model to train on the previously generated dataset.
Has a normalization as a first layer.
The model is saved in the directory ```model```

### MultiOutputRegressor.ipynb

Creates a model using the multiOutputRegressor of scikit-learn. An alternative to the previously created keras model.
Uses pickle to save the model as ```model.pkl```

### immitate.ipynb

Deploys the model on the first Pepper robot to try to mimic the second robot.

## The dataset

A dataset with 50,000 datapoints have been previously generated. The dataset is named ```dataset50000.npz```
