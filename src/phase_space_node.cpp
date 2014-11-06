
// standard headers
#include <stdio.h>
#include <vector>

// ROS headers
#include <ros/ros.h>
//#include <ros/message_operations.h>
#include <geometry_msgs/PointStamped.h>

// local headers
#include "owl.h"
#include "phase_space/PhaseSpaceMarkerArray.h"

namespace phase_space {

class PhaseSpaceClient
{
  public:

    // the node handle
    ros::NodeHandle nh_;

    // node handle in the private namespace
    ros::NodeHandle priv_nh_;
    
    // phase space related members
    int marker_count_;
    int tracker_;
    int init_flags_;
    std::string server_ip_;
    OWLMarker* markers_;

    // publishers
    ros::Publisher pub_phase_space_markers_;

    // the marker coordinates (visualization is handled by another node)
    phase_space::PhaseSpaceMarkerArray phase_space_markers_;

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

        markers_ = new OWLMarker[marker_count_];

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
        pub_phase_space_markers_ = nh.advertise<phase_space::PhaseSpaceMarkerArray>(nh.resolveName("/phase_space_markers"), 10);
        
        //ros::Rate loop_rate(10);
    }
    
    //! Empty stub
    ~PhaseSpaceClient() {delete[] markers_;}
    //~PhaseSpaceClient() {}

};

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
            {   
                ROS_DEBUG("%i %f %f %f", n, 0.001*markers_[id].x, 0.001*markers_[id].y, 0.001*markers_[id].z);
    
                // phase space gives the coordinates in mm, but in ROS, everything is MKS, so let it be hard-coded.
                // fill only the id and coordinates, the rviz marker is filled in the viz node.
                phase_space::PhaseSpaceMarker marker;
                marker.id = id;
                marker.point.x = 0.001*markers_[id].x;
                marker.point.y = 0.001*markers_[id].y;
                marker.point.z = 0.001*markers_[id].z;
                phase_space_markers_.markers.push_back(marker);
           
            } 
        }
   
        phase_space_markers_.header.stamp = ros::Time::now();
        phase_space_markers_.header.frame_id = "/phase_space_world";
        pub_phase_space_markers_.publish(phase_space_markers_);

    }

    // empty the vector
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
