# Project - 236874
In this project, we aim to predict the rotation required for a robotic hand to grab an object using an ESP32-CAM and an ultrasonic sensor.

## Our model pipeline
![Image of our model pipeline](/images/Pipe.png)

## Install
> [!TIP]
> For using the high_computation_solutions folder it is recommended to create a conda environment. 
```bash
git clone https://github.com/orel-david/project-236874.git
```

## Usage
There are two options for running the algorithm on the system:
* Segmentation in debug mode - By running `stream_esp.ino` after changing the WIFI credentials, the ESP would transmit the image it has captured along with the image's segmentation to a predetermined IP address. That is in addition to printing the required angle to terminal.
* Segementation - By running `segment_esp.ino` the ESP would perform the segmentation algorithm on the captured image and output the required angle to the terminal. This option is better in terms of performence.

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
  * `stream_esp.ino` - In this file there is the debugging loop of the algorithm. It is possible to pass WIFI credentials, in order to view the image that is passed to the esp and segmented image. In the console it prints the time per frame and the recommended angle rotation. *note*: The streaming of the esp images impact the performence of the system.
  * `segment_esp.ino` - This version does not stream the images and just prints the recommended rotation and time per frame.
  Note: in order to compile one of the .ino files it needs to be in a directory of the same name with header and cpp files. 

### `./high_computation_solutions`
This folder has our code for experiments that can't directly run on the esp.<br />

* #### `./high_computation_solutions/active_contur`
    * Contains an example.py file for using active contur technique for segmenting the object.
      Can potentially replace the current segmentation, but slower.
* #### `./high_computation_solutions/depth anything`
    * Code for creating a point-cloud of an object from a given object.
      This is done by using depth anything v2, and mirroring.
      This is a high computational solution, and commands will need to be sent to the ESP via network from a strong server/computer runnig the algorithems.
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
