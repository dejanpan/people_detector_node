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

@brief Wrapper node around the Oscar Martinez Mozos's people detector code.

@par Advertises
- \b people_detected topic with LaserScan message (values 1 in the intensities channel
denote true positive classification of human legs)

@par Subscribes
- \b /scan with LaserScan message

@par Parameters
- \b string input_scan_topic_, output_scan_topic_;
- \b string hypotheses_filename_;
- \b int num_hypotheses_;
- \b double threshold_;
- \b FILE *f_hypotheses;
- \b int list_size_;
*/

//standards
#include <iostream>
#include <float.h>
//People detector
#include <WeakHypothesis.h>
#include <PeopleDetector.h>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;


class PeopleDetectorNode 
{
public:
  ros::NodeHandle nh_;
  //ROS msgs
  sensor_msgs::LaserScan scan_ouput_;
  //Subscribers/Publishers
  ros::Subscriber scan_;
  ros::Publisher scan_publish_;
  //Parameters
  string input_scan_topic_, output_scan_topic_;
  string hypotheses_filename_;
	int num_hypotheses_;
  double threshold_;
  FILE *f_hypotheses;
  int list_size_;
  //People Detector object
  PeopleDetector pd;

  /**
   * \brief Constructor
   */
  PeopleDetectorNode() : nh_("~")
  {
    nh_.param("input_scan_topic", input_scan_topic_, std::string("/scan"));
    nh_.param("output_scan_topic", output_scan_topic_, std::string("people_detected"));
    nh_.param("threshold", threshold_, 0.15);
    nh_.param("hypotheses_filename", hypotheses_filename_, std::string(""));
    nh_.param("num_hypotheses", num_hypotheses_, 100);
    nh_.param("list_size", list_size_, 500);
    if (hypotheses_filename_ == "")
    {
      ROS_ERROR("Provide hypotheses file!!");
      exit(2);
    }
    scan_ = nh_.subscribe<sensor_msgs::LaserScan>(input_scan_topic_, 10, &PeopleDetectorNode::scan_cb, this);
    scan_publish_ = nh_.advertise<sensor_msgs::LaserScan> (output_scan_topic_, 1);
  }
  
  /**
   * \brief Destructor
   */
  ~PeopleDetectorNode()
  {
    fclose( f_hypotheses );
  }

  /**
   * \brief read out file with trained hypotheses
   */
  void init ()
  {
    try 
    {
      f_hypotheses = fopen(hypotheses_filename_.c_str(), "r");
      if ( f_hypotheses == NULL ) 
      {
        ROS_ERROR("ERROR opening hypotesis file %s", hypotheses_filename_.c_str());
        throw -1;
      }
    }
    catch (int e) 
    {
    }
    // configure the people detector object		
    pd.load(f_hypotheses, num_hypotheses_);
  }
  
  /**
   * \brief getting range messages from your system and classify them
   * \param msg input laser scan
   */
  void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    scan_ouput_.intensities.clear();
    scan_ouput_ = *msg;
    // This is a list of segments.
    // For more information have a look at ../common/dynamictable.h/hxx
    dyntab_segments *list_segments=NULL;
    int num_readings = msg->ranges.size();
    double *angles = new double[num_readings]; // array of angles
    double *ranges = new double[num_readings]; // array of measurements
    
     for (unsigned int i = 0; i < num_readings; i++)
    {
      ranges[i] = msg->ranges[i];
      angles[i] = msg->angle_min + i * msg->angle_increment;
    }
    list_segments = new dyntab_segments (list_size_);
    // segment the scan
    pd.segmentScan(threshold_, num_readings, angles, ranges, list_segments);
		
    // Classiy segments
    for (int i=0; i < list_segments->num(); i++) 
    {
      Segment *s = list_segments->getElement(i);
      // discard segments with less than three points
      if ( s->num() < 3 ) 
      {
        s->type = -1;
      }
      else 
      {	
        pd.classify(s); 
        if ( s->type == 1) 
        {
          printf("People found\n");
        }
        else 
        {
          printf("__NO__ People found\n");
          // not a person
        }
     
      }
    }
    //send out classification results (to rviz)
    for (int i=0; i < list_segments->num(); i++) 
    {
      Segment *s = list_segments->getElement(i);
      for (int out = 0; out < s->num(); out++)
      {
        if (s->type == 1)
          scan_ouput_.intensities.push_back(1);
        else
          scan_ouput_.intensities.push_back(0);
      }
    }
    scan_publish_.publish(scan_ouput_);
    // delete the list of segments 
    list_segments->setAutoDelete(true);
    delete list_segments;
    list_segments = NULL;
		
    // free memory
    delete [] ranges;
    delete [] angles;
  }
  //end PeopleDetectorNode
};
  

/* ---[ */
int main (int argc, char** argv)
{
  ros::init (argc, argv, "people_detector_node");
  PeopleDetectorNode pd_node;
  pd_node.init();
  ros::spin ();
  return (0);
}
/* ]--- */

