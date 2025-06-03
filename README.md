# Project - 236874
In this project, we aim to predict the rotation required for a robotic hand to grab an object using an ESP32-CAM and an ultrasonic sensor.

We have experimented with several methods to solve the following problem: <br />
How given an image and a reading of an ultrasonic sensor from an object, can we calculate the angle of rotation of the system relative to the object?<br />
Initially we thought on approaching this by using deep learning models for segmentation and depth perception from which we can identify the shape and analytically estimate the angle.
But, this approach was problematic due to its compute requirements that made it infeasible for the ESP to run the computation on its own. 
That would have required a stronger computer to run the calculation and transmit them to the ESP.
<br />
Due to that cost, we have chosen to try classical algorithms for image processing.
They come at the cost of being less reliable and accurate, but they are significantly lighter and can allow to use the ESP as a standalone.
After experimenting with several methods such as: 
* Canny edges algorithm to detect the edges of the image
* Active contours to try and latch onto the contour of the object at the center of the image
* Image matching to try and use the information of several images to gain a more accurate estimate of the object contour.

We finally settled on the following algorithm, that is represented in the image:
* Initially, the algorithm checks if the object is in working distance, if so it continues to segmentation. 
* The first step of the segmentation algorithm is to blur the image and perform morphological transformation on the image to remove noise.
* Then it performs canny edges to detect the contours in the image.
* All the contours are transformed in Convex hulls which are then filtered by their aspect ratio to remove noisy objects.
* The algorithm iteratively merges the largest convex hulls with the hulls that are near it. The logic was that due to noise the object can be broken down into several incomplete hulls.
* After the final convex hull is returned from the segmentation, the algorithm estimate the angle to be the angle between the center of the hull to the optical axis.
We see this as a good approximation of the required rotation, since if the center of the camera is pointing into center of the object, it should mean that the hand is aligned with the object.
## Our model pipeline
![Image of our model pipeline](/images/Pipe.png)

## Install
> [!TIP]
> For using the high_computation_solutions folder it is recommended to create a conda environment. 
```bash
git clone https://github.com/orel-david/project-236874.git
```
For running the ESP code you will need to go to the Arduino IDE, select board as `AI Thinker ESP32-CAM` and the board to which you are connected.<br />
Then from the `.ino` file that you want to run select the upload button. After the upload is over reset the ESP from the reset button on the board and the code will start running.

## Usage
There are two options for running the algorithm on the system:
* Segmentation in debug mode - Running `stream_esp.ino` would cause the ESP would transmit the image it has captured along with the image's segmentation to a predetermined IP address. <br />
That is in addition to printing the required angle to terminal. In order to connect to the stream, it would print the address of the stream which can be pasted in the browser.<br />
This option is more clear in terms of how to algorithm works, but in terms of performance it is slower, and it requires inserting WI-FI credentials into the file after cloning.
* Segmentation - Running `segment_esp.ino` the ESP would perform the segmentation algorithm on the captured image and output the required angle to the terminal. This option is better in terms of performance.

## Code structure
### `./segmentation`
This folder contains the code for the segmentation algorithm that runs on the ESP32-CAM.
* #### `Python files`
  `utils.py` contains the code we used for prototyping the algorithm. On the `classical_test.py` We checked the behaviour of the algorithm on the `./seq` and `./set` images to approximate how well it works.
* #### `./segmentation/esp code`
  This folder is where the c\cpp code that is meant to run on the ESP32-CAM. This code is build as follows:
  * `hull.h` - Here is where the geometrical struct of `Point`, `Circle` and `Hull` are defined.
  * `segment.h` - Here the segmentation algorithm is defined.
  * `utils.h` - Here many utility function are defined for the preprocessing of the images.
  * `stream_esp.ino` - In this file there is the debugging loop of the algorithm. It is required to pass WI-FI credentials in the file, in order to view the image that is passed to the esp and segmented image. <br />
  In the console it prints the time per frame and the recommended angle rotation. <br />
  *note*: The streaming of the esp images impact the performance of the system.
  * `segment_esp.ino` - This version does not stream the images and just prints the recommended rotation and time per frame.<br />
  Note: in order to compile one of the .ino files it needs to be in a directory of the same name with header and cpp files. 

### `./high_computation_solutions`
This folder has our code for experiments that can't directly run on the esp.<br />

* #### `./high_computation_solutions/active_contur`
    * Contains an example.py file for using active contur technique for segmenting the object.
      Can potentially replace the current segmentation, but slower.
* #### `./high_computation_solutions/depth anything`
    * Code for creating a point-cloud of an object from a given object.
      This is done by using depth anything v2, and mirroring.
      This is a high computational solution, and commands will need to be sent to the ESP via network from a strong server/computer running the algorithms.
      After obtaining the 3D representation, can use physic simulation to find the correct orientation.
* #### `./high_computation_solutions/image_matching`
    * Contains a code for cropping the image around the object in real time, using ORB technique.
      After adding the ultrasonic sensor, less needed and doesn't worth the computational added cost.
* #### `./high_computation_solutions/seq_images`
    * Contains several image sequences that demonstrate ESP video inputs.

* Note - For using depth anything v2:
    * Clone depth anything v2 repository.
    * Place the content of our depth anything directory inside the cloned "metric_depth" directory.
    * Add their weights to the checkpoint directory.
    * Follow our depth anything/visualize_point_cloud.py instructions for running in the terminal.

## Requirements
* ### ESP32-CAM:
  * Arduino IDE: can be downloaded from the official website, we have used version 2.3.4
  * eso32 board in the IDE: From the board manager search esp32 and download the version published by `Espressif Systems`. We have used version 3.07
  
* ### Python code
     The minimal versions are the ones we used and they can be installed with: <br /> `pip install (package)==(version)`.
  * Python interpreter version >=3.10
  * opencv version >= 4.10.0.84
  * numpy version >= 1.24.3
  * Scipy version >= 1.14.1
  * matplotlib version >= 3.8.2

## Credits
This project was intended to be a component in the robotic hand open-source project of Haifa3D, a non-profit organization, which can be viewed in the following repositories: <br />
https://github.com/Haifa3D/hand-mechanical-design
https://github.com/Haifa3D/hand-electronic-design
We give a special thanks to Dean Zadok, PhD candidate at the Technion – Israel Institute of Technology, for his guidance and support throughout this project.

