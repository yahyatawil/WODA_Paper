## Overview of the developed system
<img src="https://user-images.githubusercontent.com/1148381/133635755-495bed8d-4c8e-4c60-8d3e-89024b53fb79.png" width="500" height="700" class="center"/>

## This Repository Contents:

 Directory|File | Info
 --|--|--
 code|detection_avoidance.py|Main code:Loading `.eim` EdgeImpulse model, object detection, drawing using openCV on frames, and control law implementation.
 code|augmentation.py| Apply 4 types of augmentation to original images (Gamma - contrast - quality - noise). The scripts should be executed inside the images directory.
 code|labels.py |Modify the `bounding_boxes.labels` file exported from EdgeImpulse to add new images to it.
 Documentation/control_law_samples | - | 3 expirments logs including (frames, velocity log and video)
 Documentation/diagrams | - | source code of the diagrams/art-work provided in the paper. 
 Documentation |PR-Data.xlsx | Precision Recall of the fine-tunned model.
 Documentation |velo_curves.ods | Velocity (angular and linear) curve for one of the experiments. 


## Dataset Download Link:

Please fill this form to download our dataset WODD: [form link](https://docs.google.com/forms/d/1vZ1UEZ5PWfPGneoYeWwGmcVp_zfim9n1pFBp5WdtkEY/edit?usp=sharing).

You can access our Edge Impulse project used to annotate and train the model pubicly from here: [Project](https://studio.edgeimpulse.com/studio/44851)

## Software Development Tools:

<img src="https://user-images.githubusercontent.com/1148381/133627604-cbebfeaf-e156-408e-9287-69f465e5605a.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133627090-b5ec48b8-3129-45c7-8a79-843168bff5d2.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133627783-38b8968a-c9b8-4b63-95ad-e3cf645e796f.png " width="100" height="100" />| <img src="https://user-images.githubusercontent.com/1148381/133628429-ef1725a8-323b-4667-89b9-67538669c844.png " width="100" height="100" />| <img src="https://user-images.githubusercontent.com/1148381/133629463-f0b31e97-74e0-4b31-ab5b-65da89f833a4.png " width="100" height="100" />|
--|--|--|--|--
Edge Impulse: Annotation, training and versioning|Tensorflow: Dataset augmentation|OpenCV: Manpulate frames (drawing)|Python: Development language| Gimp: Dataset images croping and resizing

## Paper Preparation Tools:

<img src="https://user-images.githubusercontent.com/1148381/133629164-24492c86-35db-476d-903f-a0462495209c.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133629301-6d1a78ae-73c5-4a6b-86df-cc04e70e6dad.png " width="100" height="100" />|<img src="https://user-images.githubusercontent.com/1148381/133631025-1e3ba446-77ff-406e-9bcc-85a82242fa94.png" width="100" height="100" />| <img src="https://user-images.githubusercontent.com/1148381/133631491-f083d6b4-dc94-4810-ad84-2fc678937b55.png " width="100" height="100" />|
--|--|--|--
Overleaf: Latex editing collaburativly|Draw IO: Digrams|kdenlive: Video Editing|PlantUML: UML diagrams


