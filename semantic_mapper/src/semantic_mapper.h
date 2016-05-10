/**
Copyright (C) 2016, by 
Feras Dayoub (feras.dayoub@gmail.com) 

This is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
 
This software package is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Leser General Public License.
If not, see <http://www.gnu.org/licenses/>.
**/
#include "semantic_mapper/SemLabel.h"
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>

#include "message_filters/cache.h"
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>


#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap_types.h>

#include "semantic_mapper/GetSemMap.h"



boost::mutex m_lock_;

typedef pcl::PCLPointCloud2 PclCloud;
typedef pcl::PCLPointCloud2::Ptr PclCloudPtr;
typedef sensor_msgs::PointCloud2 RosCloud;
typedef sensor_msgs::PointCloud2Ptr RosCloudPtr;


typedef octomap::ColorOcTreeNode SemCell;
typedef octomap::ColorOcTree SemMap;
typedef octomap::ColorOcTreeNode::Color CellColor;

#define Pr 0.299;
#define Pg 0.587;
#define Pb 0.114;

using namespace message_filters ;


class SemanticMapper{
    private:
        std::string inputLabelTopic_;
        std::string outputCloudTopic_;
        std::string laserTopic_;
        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;
        ros::Subscriber labelSub_;
        ros::Subscriber laserSub_;
        ros::Publisher cloudPub_;
        ros::Publisher oneLabel_pub_;
        sensor_msgs::Image image_;

        ros::ServiceServer save_image_srv;

        tf::TransformListener *tf_listener_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_cloud_ptr_;
        bool send_semantic_cloud;
        laser_geometry::LaserProjection projector_;
        int current_color[3];
        ros::Time label_time;

        message_filters::Cache<sensor_msgs::LaserScan> cache;

        SemMap *semMap;
        double cell_resolution_;
        double sem_ProbHit_;
        double sem_ProbMiss_;

        double prob_thres_;


    public:
        SemanticMapper(
                        ros::NodeHandle nh,
                        ros::NodeHandle local_nh
                ):
            local_nh_(local_nh),
            nh_(nh)

        {
            tf_listener_ = new tf::TransformListener(local_nh_, ros::Duration(100));

            local_nh_.param<std::string>("label_topic",inputLabelTopic_,"/semantic_label");
            local_nh_.param<std::string>("output_cloud",outputCloudTopic_,"semantic_mapper/cloud");
            local_nh_.param<std::string>("laser_topic",laserTopic_,"laser");

            local_nh_.param<double>("prob_thres",prob_thres_,0.5);

            labelSub_ = nh_.subscribe(inputLabelTopic_, 1,&SemanticMapper::label_CB,this);

            laserSub_ = nh_.subscribe(laserTopic_, 1,&SemanticMapper::scanCallback,this);

            cloudPub_ = nh_.advertise<sensor_msgs::PointCloud2> (outputCloudTopic_,1);

            save_image_srv  = local_nh_.advertiseService("get_semantic_map",&SemanticMapper::get_sem_map_CB_, this);

            oneLabel_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("oneLabel_cloud", 30);


            send_semantic_cloud = false;
            current_color[0] = 0;
            current_color[1] = 0;
            current_color[2] = 0;
            cache.setCacheSize(1000);
            cell_resolution_ = 0.1;
            semMap = new SemMap(cell_resolution_);
            semMap->setClampingThresMax(1.0);
            semMap->setOccupancyThres(0);

        }

         ////////////////////////////////////////////////////////
        bool get_sem_map_CB_(semantic_mapper::GetSemMap::Request& req,semantic_mapper::GetSemMap::Response& res){
            std::ofstream outFile;
            outFile.open("~/semMap.txt");
            std::stringstream ss;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOut(new pcl::PointCloud<pcl::PointXYZRGB>());
            std::cout << "receive request for label_id  " << req.label_id  << "\n";
            for( SemMap::iterator itr = semMap->begin_leafs(), end = semMap->end_leafs(); itr != end; itr++){
                ss << itr.getCoordinate().x() <<" "<<itr.getCoordinate().y()<<" "<<itr.getCoordinate().z()<<" ";
                SemCell *cell0 = semMap->search(itr.getCoordinate());
                CellColor c = cell0->getColor();
                uint8_t r = (uint8_t)c.r;
                uint8_t g = (uint8_t)c.g;
                uint8_t b = (uint8_t)c.b;

                this->changeSaturation(r,g,b, cell0->getOccupancy());

                ss << cell0->getOccupancy() <<" "<<(int)r<<" "<<(int)g<<" "<< (int)b <<"\n";

                if (int(itr.getCoordinate().z()) == (req.label_id-1) ){
                    SemCell *cell = semMap->search(itr.getCoordinate());

                    pcl::PointXYZRGB pcl_point;

                    pcl_point.x = itr.getCoordinate().x();pcl_point.y = itr.getCoordinate().y();pcl_point.z = cell->getOccupancy();
                    CellColor c = cell->getColor();
                    pcl_point.r = c.r; pcl_point.g = c.g; pcl_point.b = c.b;
                    this->changeSaturation(pcl_point.r,pcl_point.g,pcl_point.b, cell->getOccupancy());

                    pOut->push_back(pcl_point);
                }

            }
            //pcl::io::savePCDFileBinary("/media/Work/test.pcd",*pOut);
            std::cout<<"found "<< pOut->size() << " points\n";
            PclCloudPtr pclOut(new PclCloud());
            pcl::toPCLPointCloud2(*pOut,*pclOut);
            RosCloud outputCloud;
            pcl_conversions::fromPCL(*pclOut,outputCloud);
            outputCloud.header.frame_id = "map";
            outputCloud.header.stamp = ros::Time::now();
            oneLabel_pub_.publish(outputCloud);
            res.done = true;
            outFile << ss.rdbuf();
            outFile.close();
            return true;
        }

        ////////////////////////////////////////////////////////

        void label_CB(const semantic_mapper::SemLabel msg){
            boost::mutex::scoped_lock l(m_lock_);
            std::cout<< "messgage received" << std::endl;
            label_time = msg.header.stamp;
            sensor_msgs::LaserScan::ConstPtr laser_scan = cache.getElemBeforeTime(label_time);

            if ((laser_scan) && (msg.prob[msg.lvl] > prob_thres_)){
                pub_sem_cloud(laser_scan,msg.prob,msg.lvl,msg.r,msg.g,msg.b);
            }else{
                     std::cout << "no scan found" << "\n";
                }
            
            }


        void pub_sem_cloud(const sensor_msgs::LaserScanConstPtr& laser_scan,std::vector<float> prob,int lvl,std::vector<int> cr,std::vector<int> cg,std::vector<int> cb){


                std::cout << "scan received\n";
                sensor_msgs::LaserScan scan(*laser_scan);
                scan.header = laser_scan->header;

                for (int i=0; i < scan.ranges.size(); i++){
                    if ((scan.ranges[i] == 0) || (scan.ranges[i] >  5)){
                          scan.ranges[i] = 3;
                    }
                }

                RosCloud input;
                try{

                projector_.transformLaserScanToPointCloud("base_link", scan, input, *tf_listener_);

                }catch(std::exception&){

                }
                PclCloudPtr source(new PclCloud());
                pcl_conversions::toPCL(input,*source);
                pcl::PointCloud<pcl::PointXYZ>::Ptr raw_c (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filled_c (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromPCLPointCloud2(*source, *raw_c);

                for( pcl::PointCloud<pcl::PointXYZ>::iterator iter = raw_c->begin(); iter != raw_c->end(); ++iter)
                {
                    float alpha =  std::atan2(iter->x,iter->y);
                    if ((alpha < 1.0) || (alpha > 2.0))
                            continue;

                    float d = std::sqrt(iter->x*iter->x + iter->y*iter->y);
                    int max_n = int (d/0.1);
                    for (int i=10; i <max_n; i++){
                        for (int l=0;l < prob.size();l++){
                            pcl::PointXYZRGB p;
                            p.x = (iter->x * i) / max_n;
                            p.y = (iter->y * i) / max_n;
                            p.z = l;
                            uint8_t r = 0, g =0, b = 0;
                            r = cr[l];
                            g = cg[l];
                            b = cb[l];
                            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                            p.rgb = *reinterpret_cast<float*>(&rgb);
                            filled_c->push_back(p);
                        }
                        }
                }



                //PclCloudPtr pclOut(new PclCloud());
                //pcl::toPCLPointCloud2(*filled_c,*pclOut);
                //RosCloud outputCloud;
                //pcl_conversions::fromPCL(*pclOut,outputCloud);
                //outputCloud.header.frame_id = laser_scan->header.frame_id;
                //outputCloud.header.stamp = laser_scan->header.stamp;
                //cloudPub_.publish(outputCloud);

                tf::StampedTransform sensor_tf;
                try {
                    tf_listener_->waitForTransform("map", "base_link",ros::Time::now(),ros::Duration(0.2));
                    tf_listener_->lookupTransform("map", "base_link",
                            laser_scan->header.stamp, sensor_tf);
                } catch (tf::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                    return;
                }

                geometry_msgs::Pose curr_pose;
                curr_pose.position.x = sensor_tf.getOrigin().x();
                curr_pose.position.y = sensor_tf.getOrigin().y();
                curr_pose.orientation.w = sensor_tf.getRotation().getW();
                curr_pose.orientation.z = sensor_tf.getRotation().getZ();


            filled_c->sensor_origin_[0] = sensor_tf.getOrigin().x();
            filled_c->sensor_origin_[1] = sensor_tf.getOrigin().y();
            filled_c->sensor_origin_[2] = sensor_tf.getOrigin().z();
            filled_c->sensor_orientation_.z() = sensor_tf.getRotation().z();
            filled_c->sensor_orientation_.w() = sensor_tf.getRotation().w();
            filled_c->sensor_orientation_.x() = sensor_tf.getRotation().x();
            filled_c->sensor_orientation_.y() = sensor_tf.getRotation().y();
            repositionCloud(filled_c, filled_c);

             octomap::Pointcloud octoCloud;
             
             for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it= filled_c->begin(); it != filled_c->end(); it++){
                 octomap::point3d point(it->x, it->y, it->z);
                 float lo = std::log(prob[it->z]/(1-prob[it->z]));
                 semMap->updateNode(point,lo);
                 SemCell* cell = semMap->search(point);
                 CellColor cS(it->r,it->g,it->b);
                 cell->setColor(cS);
             }


             pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOut(new pcl::PointCloud<pcl::PointXYZRGB>());
             for( SemMap::iterator itr = semMap->begin_leafs(), end = semMap->end_leafs(); itr != end; itr++){

                 SemCell *cell = semMap->search(itr.getCoordinate());
                 for (int l=0;l < prob.size();l++){
                    SemCell *tcell = semMap->search(itr.getCoordinate().x(),itr.getCoordinate().y(),l);

                    if (tcell->getOccupancy() > cell->getOccupancy()){
                        cell = tcell;                        

                    }
                 }
                 pcl::PointXYZRGB pcl_point;
                 pcl_point.x = itr.getCoordinate().x();pcl_point.y = itr.getCoordinate().y();pcl_point.z = 0;
                 CellColor c = cell->getColor();
                 pcl_point.r = c.r; pcl_point.g = c.g; pcl_point.b = c.b;
                 this->changeSaturation(pcl_point.r,pcl_point.g,pcl_point.b, cell->getOccupancy());
                 pOut->push_back(pcl_point);
             }

             PclCloudPtr pclOut(new PclCloud());
             pcl::toPCLPointCloud2(*pOut,*pclOut);
             RosCloud outputCloud;
             pcl_conversions::fromPCL(*pclOut,outputCloud);
             outputCloud.header.frame_id = "map";
             outputCloud.header.stamp = laser_scan->header.stamp;
             cloudPub_.publish(outputCloud);

        }
        ////////////////////////////////////////////////////////
        void scanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan){         
            cache.add(laser_scan);
        }
private:
        void repositionCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) {
                Eigen::Matrix3f rotMatrix =
                        cloud_in->sensor_orientation_.toRotationMatrix();
                Eigen::Matrix4f t;
                t(0, 0) = rotMatrix(0, 0);
                t(0, 1) = rotMatrix(0, 1);
                t(0, 2) = rotMatrix(0, 2);
                t(1, 0) = rotMatrix(1, 0);
                t(1, 1) = rotMatrix(1, 1);
                t(1, 2) = rotMatrix(1, 2);
                t(2, 0) = rotMatrix(2, 0);
                t(2, 1) = rotMatrix(2, 1);
                t(2, 2) = rotMatrix(2, 2);

                t(3, 0) = t(3, 1) = t(3, 2) = 0;
                t(3, 3) = 1;
                t(0, 3) = cloud_in->sensor_origin_[0];
                t(1, 3) = cloud_in->sensor_origin_[1];
                t(2, 3) = cloud_in->sensor_origin_[2];
                pcl::transformPointCloud(*cloud_in, *cloud_out, t);

            }


        // Robert Atkins, December 2010 (ratkins_at_fastmail_dot_fm).
        //
        // https://github.com/ratkins/RGBConverter
        void changeSaturation(uint8_t& r_in, uint8_t& g_in, uint8_t& b_in, double change) {


            double rd = (double) r_in;///255;
                double gd = (double) g_in;///255;
                double bd = (double) b_in;///255;
                double max = threeway_max(rd, gd, bd), min = threeway_min(rd, gd, bd);
                double h, s, v = max;

                double d = max - min;
                s = max == 0 ? 0 : d / max;

                if (max == min) {
                    h = 0; // achromatic
                } else {
                    if (max == rd) {
                        h = (gd - bd) / d + (gd < bd ? 6 : 0);
                    } else if (max == gd) {
                        h = (bd - rd) / d + 2;
                    } else if (max == bd) {
                        h = (rd - gd) / d + 4;
                    }
                    h /= 6;
                }


                s = s * change;

                 double r, g, b;
                int i = int(h * 6);
                    double f = h * 6 - i;
                    double p = v * (1 - s);
                    double q = v * (1 - f * s);
                    double t = v * (1 - (1 - f) * s);

                    switch(i % 6){
                        case 0: r = v, g = t, b = p; break;
                        case 1: r = q, g = v, b = p; break;
                        case 2: r = p, g = v, b = t; break;
                        case 3: r = p, g = q, b = v; break;
                        case 4: r = t, g = p, b = v; break;
                        case 5: r = v, g = p, b = q; break;
                    }

                    r_in = (uint8_t)r;
                    g_in = (uint8_t)g;
                    b_in = (uint8_t)b;

        }


        double threeway_max(double a, double b, double c) {
            return std::max(a, std::max(b, c));
        }

        double threeway_min(double a, double b, double c) {
            return std::min(a, std::min(b, c));
        }

        double hue2rgb(double p, double q, double t) {
            if(t < 0) t += 1;
            if(t > 1) t -= 1;
            if(t < 1/6) return p + (q - p) * 6 * t;
            if(t < 1/2) return q;
            if(t < 2/3) return p + (q - p) * (2/3 - t) * 6;
            return p;
        }

};
