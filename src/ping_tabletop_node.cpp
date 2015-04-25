
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <string>
#include <vector>

#include "tabletop_object_detector/TabletopSegmentation.h"
#include "tabletop_object_detector/TabletopObjectRecognition.h"

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

using namespace std;

/*! Simply pings the tabletop segmentation and recognition services and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_tabletop_node");
  ros::NodeHandle nh;

  string service_name("/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  tabletop_object_detector::TabletopSegmentation segmentation_srv;
  if (!ros::service::call(service_name, segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    exit(0);
  }
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    exit(0);
  }
  //ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
  if (segmentation_srv.response.clusters.empty()) exit(0);

  //ROS_INFO("Table position: x_min:%f x_max:%f y_min:%f y_max:%f", segmentation_srv.response.table.x_min, segmentation_srv.response.table.x_max, segmentation_srv.response.table.y_min, segmentation_srv.response.table.y_max);

  service_name = "/tabletop_object_recognition";
  if ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) )
  {
    ROS_INFO("Recognition service %s is not available", service_name.c_str());
    exit(0);
  }

  tabletop_object_detector::TabletopObjectRecognition recognition_srv;
  recognition_srv.request.table = segmentation_srv.response.table;
  recognition_srv.request.clusters = segmentation_srv.response.clusters;
  recognition_srv.request.num_models = 5;

  ros::NodeHandle priv_nh_;
  bool perform_fit_merge_;
  priv_nh_.param<bool>("perform_fit_merge", perform_fit_merge_, true);
  recognition_srv.request.perform_fit_merge = perform_fit_merge_;

  if (!ros::service::call(service_name, recognition_srv))
  {
    ROS_ERROR("Call to recognition service failed");
    exit(0);
  }

  ROS_INFO("Table pose: x:%f, y:%f, z:%f", segmentation_srv.response.table.pose.pose.position.x,segmentation_srv.response.table.pose.pose.position.y,segmentation_srv.response.table.pose.pose.position.z);
  ROS_INFO("Table orientation: x:%f, y:%f, z:%f", segmentation_srv.response.table.pose.pose.orientation.x,segmentation_srv.response.table.pose.pose.orientation.y,segmentation_srv.response.table.pose.pose.orientation.z, segmentation_srv.response.table.pose.pose.orientation.w);

  for (size_t i=0; i<segmentation_srv.response.clusters.size(); i++) 
  {
    pcl::PointXYZ average_points = pcl::PointXYZ(0.0, 0.0, 0.0);
    for (size_t j=0; j<segmentation_srv.response.clusters[i].points.size(); j++)
    {
      average_points.x += segmentation_srv.response.clusters[i].points[j].x;
      average_points.y += segmentation_srv.response.clusters[i].points[j].y;
      average_points.z += segmentation_srv.response.clusters[i].points[j].z;
    }

    average_points.x /= segmentation_srv.response.clusters[i].points.size();
    average_points.y /= segmentation_srv.response.clusters[i].points.size();
    average_points.z /= segmentation_srv.response.clusters[i].points.size();
    cout << "XYZ: " << average_points.x << " " << average_points.y << " " << average_points.z << endl;
  }

  ROS_INFO("Recognition results:");
  for (size_t i=0; i<recognition_srv.response.models.size(); i++)
  {
    if (recognition_srv.response.models[i].model_list.empty())
    {
      ROS_INFO("  Unidentifiable cluster");
    }
    else
    {
      ROS_INFO("  Model id %d",recognition_srv.response.models[i].model_list[0].model_id);
      ROS_INFO("  Model pose: x:%f y:%f z:%f", recognition_srv.response.models[i].model_list[0].pose.pose.position.x, recognition_srv.response.models[i].model_list[0].pose.pose.position.y, recognition_srv.response.models[i].model_list[0].pose.pose.position.z);
    }
  }

  ROS_INFO("Clusters correspond to following recognition results:");
  for (size_t i=0; i<recognition_srv.response.cluster_model_indices.size(); i++)
  {
    ROS_INFO("  Cluster %u: recognition result %d", (unsigned int)i, 
             recognition_srv.response.cluster_model_indices.at(i));
  }

  // JSON output, for python processing
  cout << "{" << endl;
  cout << "  \"clusters\": {" << endl;
  for (size_t i=0; i<segmentation_srv.response.clusters.size(); i++)
  {
    cout << "    \"" << i << "\": {" << endl;
    cout << "      \"average\": {" << endl;

    pcl::PointXYZ average_points = pcl::PointXYZ(0.0, 0.0, 0.0);
    for (size_t j=0; j<segmentation_srv.response.clusters[i].points.size(); j++)
    {
      average_points.x += segmentation_srv.response.clusters[i].points[j].x;
      average_points.y += segmentation_srv.response.clusters[i].points[j].y;
      average_points.z += segmentation_srv.response.clusters[i].points[j].z;
    }
    average_points.x /= segmentation_srv.response.clusters[i].points.size();
    average_points.y /= segmentation_srv.response.clusters[i].points.size();
    average_points.z /= segmentation_srv.response.clusters[i].points.size();

    cout << "        \"x\":" << average_points.x << "," << endl;
    cout << "        \"y\":" << average_points.y << "," << endl;
    cout << "        \"z\":" << average_points.z << endl;
    cout << "      }," << endl;
    cout << "      \"models\": [" << endl;

    for (size_t k=0; k<recognition_srv.response.models[i].model_list.size(); k++)
    {
      cout << "        {" << endl;
      cout << "          \"model_id\": " << recognition_srv.response.models[i].model_list[k].model_id << "," << endl;
      cout << "          \"pose\": {" << endl;
      cout << "            \"x\": " << recognition_srv.response.models[i].model_list[k].pose.pose.position.x << "," << endl;
      cout << "            \"y\": " << recognition_srv.response.models[i].model_list[k].pose.pose.position.y << "," << endl;
      cout << "            \"z\": " << recognition_srv.response.models[i].model_list[k].pose.pose.position.z << endl;
      cout << "          }," << endl;
      cout << "         \"confidence\": " << recognition_srv.response.models[i].model_list[k].confidence << endl;
      cout << "        }";
      
      if (k != recognition_srv.response.models[i].model_list.size() - 1)
      {
        cout << "," << endl;
      }
      else
      {
        cout << endl;
      }
    }
    cout << "      ]" << endl;
    cout << "    }";

    if (i != segmentation_srv.response.clusters.size() - 1)
    {
      cout << "," << endl;
    }
    else
    {
      cout << endl;
    }
  }
  cout << "  }" << endl;
  cout << "}";


  return true;
}
