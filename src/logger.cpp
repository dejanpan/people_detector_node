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

@brief Logger for the 2D laser people detector

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


class LoggerNode 
{
public:
  ros::NodeHandle nh_;
  //Subscribers/Publishers
  ros::Subscriber scan_;
  //Parameters
  string filename_;
  string input_scan_topic_;
  FILE *f_out_;
  int scan_id_; // identifier for the scans
  

  /**
   * \brief Constructor
   */
  LoggerNode() : nh_("~")
  {
    nh_.param("input_scan_topic", input_scan_topic_, std::string("/base_scan_front"));
    scan_ = nh_.subscribe<sensor_msgs::LaserScan>(input_scan_topic_, 10, &LoggerNode::scan_cb, this);
  }
  
  /**
   * \brief Destructor
   */
  ~LoggerNode()
  {
    fclose( f_out_ );
  }

  /**
   * \brief Open output file
   */
  void init ()
  {
	  scan_id_ = 0;
	  
      f_out_ = fopen(filename_.c_str(), "w");
      if ( f_out_ == NULL ) 
      {
        ROS_ERROR("ERROR opening output file %s", filename_.c_str());
      }
  }
  
  /**
   * \brief getting range messages from your system and write them to a file
   * \param msg input laser scan
   */
  void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
	double value;
	double angle; // radians
  int num_readings = msg->ranges.size();
    
	scan_id_ ++;
	
	fprintf(f_out_, "%d %.8f 1 %d", scan_id_, msg->header.stamp.toSec(), num_readings);
	for (int i = 0; i < num_readings; i++)
    {
      value = msg->ranges[i];
      angle = msg->angle_min + i * msg->angle_increment;
	  
      fprintf(f_out_, " %.8f %.8f", angle, value);	  
    }
    fprintf(f_out_, "\n");
    ROS_INFO("Scan nr %d saved to disk.", scan_id_);
  }
  //end LoggerNode
};
  

/* ---[ */
int main (int argc, char** argv)
{
		
  if (argc < 2) {
	cerr << "USE: " << argv[0] << " <output_file>" << endl;
	return 1;
  }
  ros::init (argc, argv, "logger_node");
  LoggerNode lg_node;
  lg_node.filename_.assign(argv[1]);
  ROS_INFO("filename %s", lg_node.filename_.c_str());
  lg_node.init();
  ros::spin ();
  return (0);
}
/* ]--- */

