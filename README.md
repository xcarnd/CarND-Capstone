## Udacity Self-Driving Car Engineer Nanodegree
# Final Project: System Integration - The PAW Patrol Team


This is the last project for the Udacity Self-Driving Car Nanodegree Program. Also, this is the first team project! Members of our team comes from three continents, three time zones: US (San Francisco Bay Area), Europe (a small country - Serbia) and China. 


Meet the PAW Patrol Team: 

![System Integration](/imgs/pawpatrol_resize.png)


Name | GitHub | Location | Task
------------ | ------------- | ------------- | -------------
Jelena Kocic (team lead) | @cvetakg | Belgrade, Serbia| Waypoint Uploader Partial
Zhuhui Chen | @xcarnd | Guangzhou, China | Traffic Light Detection - Detection and Waypoint publishing, Waypoint Uploader Partial
Haocheng Yang | @cabbagehao | Shenzhen, China | DBW Node, Waypoint Uploader Full
Adarsha Badarinath | @adarshakb | San Francisco Bay Area, CA| Traffic Light Detection - Detection
Zhenpeng Chen | @IChappie | Guangzhou, China | Waypoint Uploader Full

# The Scope

The capstone project has an aim that each team make a program for a real Self-Driving Car, the Carla. Program was written on Ubuntu Linux, under Robotic Operating System (ROS) using Python. Development and the tests are done using Udacity simulator for the system integration project. And the code is going to be putted in the car and tested in the real conditions!  
## What we have done  
     1. The vehicle can run stably and complete the entire road(7KM).  
     2. The vehicle can accurately recognize the state and position of traffic lights.
     3. The vehicle will stop at the red light and pass by the green light  
     4. The vehicle can be taken over half way, and continue after canceling the takeover.  

![System Integration](/imgs/P1.png)

![System Integration](/imgs/final-project-ros-graph-v2.png)


## Results


### Normal drive:

![System Integration](/imgs/1.png)


### Car stops on red light:

![System Integration](/imgs/2.png)


### Car pass on green light:

![System Integration](/imgs/3.png)

### Performance result:

> Test environment:   
>     Intel E5-2640 + 8G RAM + GTX1050Ti GPU  
>     Ubuntu16.04 + ROS-kinetic  
 1. Module delay: (the runtime of main callback function )  
     *server* and *WayPointUpdater* delay below 5ms  
     *WaypointFollower*,*TLDetector*and*DBWNode* delay below 0.5ms  
 2. Stable control at **30HZ**.   
 3. The vehicle can complete the entire road at the maximum of **170km/h** (The best speed is 40km/h, PID parameters are adjusted according to this.)  
 4. The *average distance error* is **less than 30cm** when the vehicle finish the circle at 40km/h. (Measured by the distance to get the nearest waypoint every time)  
 5. The vehicle can stop in front of the red light line **within 1.5m** when running at the speed of 40km/h.   
    
## Instalation


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

## Implementation Details

### Approach for traffic light detection

We currently trained CNNs and use them for traffic light detection
problem. Because of the difference between the image captured in
simulator and in Carla, we've trained two separate models, one for the
simulator and the other for Carla.

The models we used are largely inspired by the NVidia architecture
introduced in Term1 Project3. The main idea is, when an image is
coming, we use the model and classify the image as either containing
red light, yellow light or green light. We don't train the model to
recognize "no lights" because we're training a model from scratch and
we don't have enough samples for the ambient (especially when
trainning models for Carla).

Because of this, the model cannot give out meaningful results when
there's no light in sight. To avoid uncessary detection (and thus
meaningless wrong detection results), the detector will be activated
only if the current position of the vehicle is close enough to a
traffic light, e.g., 100M. For Carla, we also use the camera info to
calculate the FOV in horizontal direction, and check if the nearest
traffic light is within the range of FOV. It turns out the FOV
criteria is not very accurate, but good enough to filter out the major
case when Carla is not facing to the traffic light.

We also stop detection once the vehicle has passed the corresponding
stop line for the traffic light.

Below are the architectures for the two models.

+ Model for simulator

![Model for simulator](/imgs/model_styx.png)

+ Model for Carla

![Model for Carla](/imgs/model_carla.png)

#### Model training

For simulator, the training set is extracted from the first traffic
light (and only the first traffic light) in the simulator. After that,
the training set is augmented by translation and flipping. We
augmented the training set from 180 original samples to ~9000
samples.

For testing, we captured images from different traffic lights in the
simulator.

With these samples we're able to train a model with 99%+
train/validation accuracy and about 92% test accuracy.

For Calar, we used a similar strategy for preparing samples: we
extract training set from one of the rosbag Udacity has provided and
extract testing set from the other one. Images from real scene are
more complicated than those in the simulator, so we applied more
augmentation to the train set we've extracted, including scaling and
changing brightness. We augmented the training set from about 500
samples to 15000 samples.

The training/valid accuracy for Carla's model is also 99+%. For
testing accuracy, we archive complete correct for the red/green lights
detection but are unable to test against yellow lights since there's
no yellow lights in the rosbag we used for testing.

#### Shortcomings

As we've mentioned, our approach is not able to tell apart whether
there're lights or not, thus we've used FOV for detection
assisting. FOV on the other hands, although simple, is not a stable
way. For a better solution, we may head for other popular objection
detection networks, such as YOLO and SSD, and use transfer learning to
make them fitting the needs for driving Carla (both in simulator and
real world).

### What problems we have encountered?  
1. The vehicle stopped or ran out of control after a few minutes of normal driving  (performance issues)

        1. waypointUpdater release final waypoint was time-consuming:  
                Reduced the number of publications   
                Reused waypoints object list instead of create new object each time.  
                Used distance squared rather than distance.  
                canceled the use of lambda (lambda slightly affect performance)  
 
        2. waypointUpdater used a service to find the closed waypoint, which leaded to greater delay:  
                Used python module instead of service. Service is not suitable for frequent calls.  

        3. In server image callback function, the operation of np.asarray () takes more than 10ms  
                solution:  
                    Used a pre-function of start_background_task () before the callback function of image for asynchronous operations   
                Failed attempt:  
                    1. Used python multithreading  
                    2. Used other numpy functions instead of np.asarray's conversion  
                    3. Set the server asynchronous mode  
                    4. Set topic buffer size  
                    5. Have considered shared memory / delay conversion, it involves changes to the server interface, so I gave up.  
        4. Canceled all unnecessary log printing  


    2. Acceleration and braking are not accurate(stability problem)  
    
        Solution:  
            Throttle and barke can not publish at the same time, even if a certain value is 0.  
            Throttle is the percentage used, the brakes use torque, and the torque is multiplied by the vehicle mass and wheelbase  
        Failed attempt:  
            1. Adjust PID  
            2. Set the system delay  

    3. The vehicle target speed was sometimes negative  
        Modified persuit in the speed calculation. Added a fabs () to solve.  

    4. The vehicle can not stop precisly before the red light.  
        Using a linear function of distance to set the target speed starts braking a long way and may not stop when approaching the target point.  
        So I used a sqrt function instead, it converges to 0 more quickly as it approaches the target.   
        Set the target speed to 0 directly when the target speed is <3.  
        
        
        
        
