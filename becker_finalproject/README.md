# Intro to Robotics Final Project
### Kyle Becker

## Running the program

Fire up the *getwaypoint* server using `rosrun projectserver wpserver.py`. To run the program, launch the husky in gazebo, run AMCL SLAM, and launch `move_base`, use the command `roslaunch final_project playpen.launch`. 

## Algorithm

The program uses a simple feature matching algorithm. The realsense camera feed is decomposed into SIFT
features, and these are compared using a bruteforce matcher to the SIFT features of a gallery of pictures
of the symbols that are divided into red and green classes. The camera image is coded as the class that 
has the most average matches.

I also implemented an algorithm that uses bitmasks to filter the colors of the images before running
the SIFT feature extraction/matching, but the version without the red/green filtering actually performed
*much* better at correctly classifying the objects.

## Limitations

Unfortunately, this project does have some limitations. I couldn't figure out how to get the correct information
to localize the symbols using the camera feed, so I had to rely on prior knowledge of the symbols' locations.

Using only the launch file to launch the python script also sometimes results in some strange behavior, so I
recomment commenting out the line in the launch file that runs the node and running the script seperately
using `rosrun becker_finalproject control.py` if you have any issues. The program also responds poorly to `Ctrl-C`
to stop the program, for which I can't figure out the reason. I would recommend closing the tab if you need to kill
the program.

## Resources:

Much of the code for `helpers.py` comes from <https://kushalvyas.github.io/BOV.html>.  
The color filtering algorithm mainly draws on the code examples from 
<https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/>
and <https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/>.  
The algorithm for the image matching comes from the [OpenCV documentation](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html)  
The code for the `movebase_client` comes from [this github post](https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/) and are an adaptation of the C++ code on the [ROS documentation](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals).

