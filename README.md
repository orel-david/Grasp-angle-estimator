# Project - 236874


## Code structure
#### `segmentation`
   *

### `./high_computation_solutions`
This folder has our code for experiments that can't directly run on the esp.<br />

* #### `./high_computation_solutions/active_contur`
    * Contains an example.py file for using active contur technique for segmenting the object.
      Can potentially replace the current segmentation.
* #### `./high_computation_solutions/depth anything`
    * Code for creating a point-cloud of an object from a given object.
      This is done by using depth anything v2, and mirroring.
      This is a high computational solution.
      After obtaining the 3D representation, can use physic simulation to find the correct orientation.
* #### `./high_computation_solutions/image_matching`
    * Contains a code for cropping the image around the object in real time, using ORB technique.
      After adding the ultrasonic sensor, less needed and doesn't worth the computational add cost.
* #### `./high_computation_solutions/seq_images`
    * Contains several image sequences that demonstrate esp input.

* Note - For using depth anything v2:
    * Clone depth anything v2 repository.
    * Place the content of our depth anything directory inside the cloned "metric_depth" directory.
    * Add their weights to the checkpoint directory.
    * Follow our depth anything/visualize_point_cloud.py instructions for running in the terminal.
