// use <> for system headers
#include <stdio.h>
#include <vector>
// ROS headers
#include <ros/ros.h>
#include <ros/message_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
//#include <rosbag/bag.h>

// use "" for local headers
#include "owl.h"
#include "phase_space/AllMarkers.h"

namespace phase_space {

class PhaseSpaceClient
{
  public:

    // suggested/quasi_mandatory members, note the member_name_ naming convention

    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    
    // phase space related members
    int marker_count_;
    int tracker_;
    int init_flags_;
    std::string server_ip_;
    OWLMarker* markers_;

    // publishers
    ros::Publisher pub_phase_space_markers_;

    ros::Publisher pub_Allmarkers_;

    // the markers
    visualization_msgs::MarkerArray phase_space_markers_;

    // it is very useful to have a listener and broadcaster to know where all frames are
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    

  public:

    // callback functions
    void publishMarkers();

    // constructor
    PhaseSpaceClient(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

        // initialize the phase space client
        priv_nh_.param<std::string>("ip", server_ip_, "192.168.1.230");
        char* ip = new char[server_ip_.size() + 1];
        std::copy(server_ip_.begin(), server_ip_.end(), ip);
        ip[server_ip_.size()] = '\0'; // don't forget the terminating 0

        priv_nh_.param<int>("init_flags", init_flags_, 0);
        
        priv_nh_.param<int>("marker_count", marker_count_, 72);

        markers_=new OWLMarker[marker_count_];

        if(owlInit(ip, init_flags_) < 0)
        {
            ROS_ERROR("error in owl initialization");
        }


        // create tracker 0
        tracker_ = 0;
        owlTrackeri(tracker_, OWL_CREATE, OWL_POINT_TRACKER);

        // set markers
        for(int i = 0; i < marker_count_; i++)
            owlMarkeri(MARKER(tracker_, i), OWL_SET_LED, i);

        // activate tracker
        owlTracker(tracker_, OWL_ENABLE);

        // flush requests and check for errors
        if(!owlGetStatus())
        {
            ROS_ERROR("error in point tracker setup %i", owlGetError());
        }

        // set default frequency
        owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);
  
        // start streaming
        owlSetInteger(OWL_STREAMING, OWL_ENABLE);

        // advertise topics
        pub_phase_space_markers_ = nh.advertise<visualization_msgs::MarkerArray>(nh.resolveName("/phase_space_markers"), 10);
        
        pub_Allmarkers_ = nh.advertise<AllMarkers>(nh.resolveName("/all_markers"), 10);
        //ros::Rate loop_rate(10);
    }
    
    // ROS marker initialization
    void inizialize_markers(visualization_msgs::Marker&);
    
    //! Empty stub
    ~PhaseSpaceClient() {delete[] markers_;}
    //~PhaseSpaceClient() {}

};

void PhaseSpaceClient::inizialize_markers(visualization_msgs::Marker& marker)
{
     marker.header.frame_id = "/world";
     marker.ns = "/phase_space";
     marker.header.stamp = ros::Time::now();
     marker.type = visualization_msgs::Marker::SPHERE;
     marker.action = visualization_msgs::Marker::ADD;
     marker.scale.x = 0.01;
     marker.scale.y = 0.01;
     marker.scale.z = 0.01;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     marker.color.r = 1.0f;
     marker.color.g = 0.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
     marker.lifetime = ros::Duration();

}

// this function is called as fast as ROS can from the main loop directly
void PhaseSpaceClient::publishMarkers()
{
    int err;
    int pos=0;
    // get some markers
    int n = owlGetMarkers(markers_, marker_count_);

    // check for error
    if((err = owlGetError()) != OWL_NO_ERROR)
    {    ROS_INFO("error %d", err);
        return;
    }

    // no data yet
    if(n == 0)
        return;

  
    if(n > 0)
    {
        for (int id = 0; id<n; id++)
        {
            if(markers_[id].cond > 0)
            {   ROS_INFO("%i %f %f %f", n, 0.001*markers_[id].x, 0.001*markers_[id].y, 0.001*markers_[id].z);
                visualization_msgs::Marker mark;
                inizialize_markers(mark);
    
                mark.pose.position.x = 0.001*markers_[id].x;
                mark.pose.position.y = 0.001*markers_[id].y;
                mark.pose.position.z = 0.001*markers_[id].z;
                mark.id = id;
                phase_space_markers_.markers.push_back(mark);
           
            } 
        }
   
        AllMarkers appoMarker;
        appoMarker.header=phase_space_markers_.markers[0].header;
        appoMarker.LEDs=phase_space_markers_;
        pub_Allmarkers_.publish(appoMarker);
        pub_phase_space_markers_.publish(phase_space_markers_);

    }
    phase_space_markers_.markers.clear();
}

} // namespace phase_space

int main(int argc, char **argv)
{
    ros::init(argc, argv, "phase_space_node");
    ros::NodeHandle nh;

    phase_space::PhaseSpaceClient node(nh);

    while(ros::ok())
    {
        node.publishMarkers();
        ros::spinOnce();
        
    }
    // cleanup
    owlDone();
    return 0;
}
