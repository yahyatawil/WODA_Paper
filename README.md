[![License CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-blue.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
![Python 3.7](https://img.shields.io/badge/python-3.7-green.svg)

# Towards Only-Vision Autonomous Wheelchair: A Deep Learning Obstacle Detection and Image-based Avoidance

## Overview of the developed system

Our system presented in the paper uses Mobilenetv2 SSD fine-tunned using a developed dataset built by us. The dataset is captured from Hasan Kalyoncu University campus sidewalks. The dataset and fine-tunning is done using EdgeImpulse and the overall code is Python-based. The object detection model passes the  obstacle position to a control law to calculate the required linear and angular speed to push the center of deteted bounding box to the sides of image. The controle law is image based. Our aim in this project is to build an object detection and avoidance system for smart wheelchair using Camera only. We used one Raspberry Pi 4, Raspberry Pi camera and Sabertooth for motor control.  

<img src="https://user-images.githubusercontent.com/1148381/133652321-5524d03b-3849-4a80-b796-4c1c6b2d2c3e.png" width="500" height="700" class="center"/>

### Project members ###

* Yahya Tawil
* Abdul Hafez Abdul Hafez (supervisor).

## This Repository Contents:

 Directory|File | Info
 --|--|--
 code|detection_avoidance.py|Main code:Loading `.eim` EdgeImpulse model, object detection, drawing using openCV on frames, and control law implementation.
 code|augmentation.py| Apply 4 types of augmentation to original images (Gamma - contrast - quality - noise). The scripts should be executed inside the images directory.
 code/model|model_without_augmentation.eim(lite)|The resultant model after fine-tunning Mobilenetv2 SSD with our dataset.
 code|labels.py |Modify the `bounding_boxes.labels` file exported from EdgeImpulse to add new images to it.
 Documentation/control_law_samples | - | 3 expirments logs including (frames, velocity log and video)
 Documentation/diagrams | - | source code of the diagrams/art-work provided in the paper. 
 Documentation |PR-Data.xlsx | Precision Recall of the fine-tunned model.
 Documentation |velo_curves.ods | Velocity (angular and linear) curve for one of the experiments. 


## Dataset Download Link:

Please fill this form to download our dataset WODD: [form link](https://docs.google.com/forms/d/1vZ1UEZ5PWfPGneoYeWwGmcVp_zfim9n1pFBp5WdtkEY/edit?usp=sharing).

You can access our Edge Impulse project used to annotate and train the model publicly from here: [Project](https://studio.edgeimpulse.com/studio/44851)

## Software Development Tools:

<img src="https://user-images.githubusercontent.com/1148381/133627604-cbebfeaf-e156-408e-9287-69f465e5605a.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133627090-b5ec48b8-3129-45c7-8a79-843168bff5d2.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133627783-38b8968a-c9b8-4b63-95ad-e3cf645e796f.png " width="100" height="100" />| <img src="https://user-images.githubusercontent.com/1148381/133628429-ef1725a8-323b-4667-89b9-67538669c844.png " width="100" height="100" />| <img src="https://user-images.githubusercontent.com/1148381/133629463-f0b31e97-74e0-4b31-ab5b-65da89f833a4.png " width="100" height="100" />|
--|--|--|--|--
Edge Impulse: Annotation, training and versioning|Tensorflow: Dataset augmentation|OpenCV: frames processing |Python: Development language| Gimp: Dataset images croping and resizing

## Paper Preparation Tools:

<img src="https://user-images.githubusercontent.com/1148381/133629164-24492c86-35db-476d-903f-a0462495209c.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133629301-6d1a78ae-73c5-4a6b-86df-cc04e70e6dad.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133631025-1e3ba446-77ff-406e-9bcc-85a82242fa94.png" width="100" height="100" />| <img src="https://user-images.githubusercontent.com/1148381/133631491-f083d6b4-dc94-4810-ad84-2fc678937b55.png " width="100" height="100" />|
--|--|--|--
Overleaf: Latex editing collaboratively|Draw IO: Digrams|kdenlive: Video Editing|PlantUML: UML diagrams

## Hints
* Downloading OpenCV to Raspberry Pi is a little bit tricky. [This guide](https://gist.github.com/willprice/abe456f5f74aa95d7e0bb81d5a710b60 )  seems the best one. Swap area may need to be increased if the installation stuck. 
* If you intend to use another Hardware than Raspberry Pi (.i.e. Jetson Nano), then you need to connect to Edge Impulse [project](https://studio.edgeimpulse.com/studio/44851) and build it again for your new Hardware. 


