# ros-semantic-mapper

https://wiki.qut.edu.au/display/cyphy/Vision-based+Semantic+Mapping


## Place categorization and semantic mapping on a mobile robot

Sunderhauf, Niko, Dayoub, Feras, McMahon, Sean, Talbot, Ben, Schulz, Ruth, Corke, Peter, Wyeth, Gordon, Upcroft, Ben, & Milford, Michael (2016) Place categorization and semantic mapping on a mobile robot. In Proceedings of the International Conference on Robotics and Automation, IEEE, Stockholm, Sweden.

```
@inproceedings{95288,
booktitle = {IEEE International Conference on Robotics and Automation (ICRA 2016)},
month = {May},
title = {Place categorization and semantic mapping on a mobile robot},
author = {Niko Sunderhauf and Feras Dayoub and Sean McMahon and Ben Talbot and Ruth Schulz and Peter Corke and Gordon Wyeth and Ben Upcroft and Michael Milford},
address = {Stockholm, Sweden},
publisher = {IEEE},
year = {2016}
}
```

## Abstract

In this paper we focus on the challenging problem of place categorization and semantic mapping on a robot with-out environment-specific training. Motivated by their ongoing success in various visual recognition tasks, we build our system upon a state-of-the-art convolutional network. We overcome its closed-set limitations by complementing the network with a series of one-vs-all classifiers that can learn to recognize new semantic classes online. Prior domain knowledge is incorporated by embedding the classification system into a Bayesian filter framework that also ensures temporal coherence. We evaluate the classification accuracy of the system on a robot that maps a variety of places on our campus in real-time. We show how semantic information can boost robotic object detection performance and how the semantic map can be used to modulate the robot’s behaviour during navigation tasks. The system is made available to the community as a ROS module.

rosbag files and network model available [here](https://cloudstor.aarnet.edu.au/plus/index.php/s/n63jLJyL2JjcCHq)


# How to use:

* First clone the repository from the link in catkin workspace
```
git clone https://github.com/fdayoub/ros-semantic-mapper.git
```
* Run setup file which contains necessary steps to download model files and configure paths
```
./setup.sh
```
* In subsequent runs you can use
```
./ros_launch.sh
```
* Play the bagfile you downloaded in separate terminal
```
source devel/setup.sh
rosbag play office.bag
```
* Check the topics available in separate terminal
```
source devel/setup.sh
rostopic list
```
* Note the pointcloud semantic_mapper/cloud
```
rosrun rviz rviz  
```  
* in rviz display: map, odom, semantic_mapper/cloud
* call the following service to get a semantic map that code the probability distribution of a place over a covered area: rosservice call /semantic_mapper_node/get_semantic_map "label_id: x" replace x by a number between 1 to 11 to select which label you want from my_cats.txt file (of course different list in my_cats.txt will means different x range)
* Display the served map (/oneLabel_cloud) in rvis
* Also check the topic semantic_label
* The image /sem_label_image display the probability distribution over all place labels in the current view.  
