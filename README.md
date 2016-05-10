# ros-semantic-mapper

https://wiki.qut.edu.au/display/cyphy/Vision-based+Semantic+Mapping


Place categorization and semantic mapping on a mobile robot

Sunderhauf, Niko, Dayoub, Feras, McMahon, Sean, Talbot, Ben, Schulz, Ruth, Corke, Peter, Wyeth, Gordon, Upcroft, Ben, & Milford, Michael (2016) Place categorization and semantic mapping on a mobile robot. In Proceedings of the International Conference on Robotics and Automation, IEEE, Stockholm, Sweden.

Abstract

In this paper we focus on the challenging problem of place categorization and semantic mapping on a robot with-out environment-specific training. Motivated by their ongoing success in various visual recognition tasks, we build our system upon a state-of-the-art convolutional network. We overcome its closed-set limitations by complementing the network with a series of one-vs-all classifiers that can learn to recognize new semantic classes online. Prior domain knowledge is incorporated by embedding the classification system into a Bayesian filter framework that also ensures temporal coherence. We evaluate the classification accuracy of the system on a robot that maps a variety of places on our campus in real-time. We show how semantic information can boost robotic object detection performance and how the semantic map can be used to modulate the robot’s behaviour during navigation tasks. The system is made available to the community as a ROS module.
