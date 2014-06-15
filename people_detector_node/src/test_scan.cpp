/*
 * Copyright (c) 2010 Oscar Martinez Mozos <omozos@googlemail.com> 
 * Dejan Pangercic <pangercic -=- cs.tum.edu> (ROS maintaniner)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/** 
@file

@brief for testing only. Node only subscribes to topic with LaserScan message and outputs
message fields.

@par Subscribes
- \b /scan with LaserScan message

@par Parameters
*/

//standards
#include <iostream>
#include <float.h>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;


class TestNode 
{
public:
  ros::NodeHandle nh_;
  //Subscribers/Publishers
  ros::Subscriber scan_;
  //Parameters
  string input_scan_topic_;
  int scan_id_; // identifier for the scans
  

  /**
   * \brief Constructor
   */
  TestNode() : nh_("~")
  {
    nh_.param("input_scan_topic", input_scan_topic_, std::string("/shoulder_scan"));
    scan_ = nh_.subscribe<sensor_msgs::LaserScan>(input_scan_topic_, 10, &TestNode::scan_cb, this);
  }
  
  /**
   * \brief Destructor
   */
  ~TestNode()
  {
  }

  /**
   * \brief getting range messages from your system and output fields of choice
   * \param msg input laser scan
   */
  void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    ROS_INFO("Intensities and Ranges size: %ld | %ld", msg->ranges.size(), msg->intensities.size());
   }
  //end TestNode

};
  

/* ---[ */
int main (int argc, char** argv)
{
		
  ros::init (argc, argv, "test_node");
  TestNode lg_node;
  ros::spin ();
  return (0);
}
/* ]--- */

