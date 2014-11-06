// standard headers
#include <fstream>
#include <iostream>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>

#include <Eigen/Eigen>

// local headers
#include "PoseTrackingFilter.h"
#include "phase_space/PhaseSpaceMarkerArray.h"

namespace phase_space {

class PoseTracker
{
  private:
    // the node handle
    ros::NodeHandle nh_;

    // ode handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers
    ros::Subscriber sub_phase_space_markers_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

  	std::string reference_frame_;
  	std::string object_frame_;
  	std::vector<int> marker_id;

  	// data points, in local reference and measured ones
  	int Nleds_;
  	Eigen::MatrixXd local_points_;
	Eigen::MatrixXd measured_points_;

	// the tracking filter
	PoseTrackingFilter filter_;

	Eigen::VectorXd transfParameters_;

	// the parameters loaded to param server
	XmlRpc::XmlRpcValue leds_;

	// helper functions
  	void parseParameters(const XmlRpc::XmlRpcValue &leds);

  	void computeOptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP);

  public:

    // callback functions
	void estimatePose(const phase_space::PhaseSpaceMarkerArray & msg);
   
    // constructor
    PoseTracker(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~"), filter_()
    {
        //priv_nh_.param<std::string>("cad_file", cad_file_path_, "");
        priv_nh_.param<std::string>("reference_frame", reference_frame_, "");
        priv_nh_.param<std::string>("object_frame", object_frame_, "");

        // get the led location in local frame from ros param server
        nh_.getParam("leds", leds_);
		parseParameters(leds_);

    	transfParameters_.resize(7);
    	
    	// initialize subscriber
        sub_phase_space_markers_ = nh_.subscribe(nh_.resolveName("/phase_space_markers"), 10, &PoseTracker::estimatePose,this);
    }

    //! Empty stub
    ~PoseTracker() {}

};



void PoseTracker::parseParameters(const XmlRpc::XmlRpcValue &leds)
{
	ROS_ASSERT(leds.getType() == XmlRpc::XmlRpcValue::TypeArray);

	// if parameters is good, resize the matrix to account for number of leds used to track 
	// 4 =  id + 3Dposition
	local_points_=Eigen::MatrixXd::Zero(leds.size(),4);
	Nleds_ = leds.size();

	for( int i = 0; i < Nleds_; ++i)
	{
		XmlRpc::XmlRpcValue current_led = leds_[i];

		// parse ID
		ROS_ASSERT(current_led.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    	if( current_led.hasMember("id") )
    	{
    		ROS_ASSERT(current_led["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    		int id = current_led["id"];
    		local_points_(i, 0) = (double) id;
    	}
    	else
    	{
    		ROS_ERROR("No id value for the current led. Check the yaml configuration for this object");
    		return;
    	}

    	// parse position
    	std::vector<double> position;
    	if( current_led.hasMember("position") )
    	{
			for (int j = 0; j < current_led["position"].size(); ++j) 
			{
				ROS_ASSERT(current_led["position"][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				position.push_back( current_led["position"][j] );
			}
			local_points_(i, 1) = position[0];
			local_points_(i, 2) = position[1];
			local_points_(i, 3) = position[2];
    	}
    	else
		{
			ROS_ERROR("No id value for the current led. Check the yaml configuration for this object");
			return;
		}
		
	}

	ROS_INFO("Succesfully parsed all LED parameters!");
	//std::cout << local_points_ << std::endl;

	return;
}

void PoseTracker::estimatePose(const phase_space::PhaseSpaceMarkerArray & msg)
{	
	// measured points is (at most) equal to the number of leds in the model
	// 4 =  id + 3Dposition
    measured_points_.resize(1, 3);

    // found leds are copied here
    Eigen::MatrixXd found_points;
    found_points.resize(1, 3) ;
    int counter = 0;

	// apply match criteria (the led id) in order to provide the 
	// available measured points. measured_points are filled in the same order
	// as they appear in local_points
	// is there any other way to do this? 
	for (int i = 0; i < Nleds_; ++i)
	{
		for(int j = 0; j < msg.markers.size(); ++i)
		{
			// criteria, there should be only one match per i iteration
			if(local_points_(i,0) == msg.markers[j].id)
			{
				measured_points_(counter,0) = msg.markers[j].point.x;
				measured_points_(counter,1) = msg.markers[j].point.y;
				measured_points_(counter,2) = msg.markers[j].point.z;
				found_points(counter,0) = local_points_(i,1);
				found_points(counter,1) = local_points_(i,2);
				found_points(counter,2) = local_points_(i,3);
				counter++;

				measured_points_.conservativeResize(counter+1, Eigen::NoChange_t());
				found_points.conservativeResize(counter+1, Eigen::NoChange_t());
			}
		}
	}

	Eigen::MatrixXd startP = found_points;

	Eigen::MatrixXd finalP = measured_points_;
	
	if (filter_.isInitialized() == 0)
	{	
		filter_.initialize(1);
		computeOptimalRigidTransformation(startP, finalP);
		filter_.setKalman_x(transfParameters_);
	}		
	else
	{	
		filter_.prediction(startP);
		transfParameters_ = filter_.update(finalP);
	}

	// fill and publish optimal pose
	tf::Transform pose;
	pose.setOrigin( tf::Vector3(transfParameters_(0),transfParameters_(1),transfParameters_(2)));
   	pose.setRotation( tf::Quaternion(transfParameters_(3),transfParameters_(4),transfParameters_(5),transfParameters_(6)));
	tf_broadcaster_.sendTransform(tf::StampedTransform(pose, ros::Time::now(), reference_frame_.c_str(), object_frame_.c_str()));
 
}


// computes the optimal rigid body transformation given a set of points
void PoseTracker::computeOptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP)
{	
	Eigen::Matrix4d transf;
	
	if (startP.rows()!=finalP.rows())
	{	
		ROS_ERROR("We need that the rows be the same at the beggining");
		exit(1);
	}

	Eigen::RowVector3d centroid_startP = Eigen::RowVector3d::Zero(); 
	Eigen::RowVector3d centroid_finalP = Eigen::RowVector3d::Zero(); //= mean(B);
	Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

	//calculate the mean
	for (int i=0;i<startP.rows();i++)
	{	
		centroid_startP = centroid_startP+startP.row(i);
		centroid_finalP = centroid_finalP+finalP.row(i);
	}
	
	centroid_startP = centroid_startP/startP.rows();
	centroid_finalP = centroid_finalP/startP.rows();

	for (int i=0;i<startP.rows();i++)
		H = H + (startP.row(i)-centroid_startP).transpose()*(finalP.row(i)-centroid_finalP);

   	Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
   
    Eigen::MatrixXd U = svd.matrixU();
   	Eigen::MatrixXd V = svd.matrixV();
  
    if (V.determinant()<0)
   		V.col(2)=-V.col(2)*(-1);

	Eigen::MatrixXd R = V*U.transpose();

	Eigen::Matrix4d C_A = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d C_B = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d R_new = Eigen::Matrix4d::Identity();
			
	C_A.block<3,1>(0,3) = -centroid_startP.transpose();
	R_new.block<3,3>(0,0) = R;
	
	C_B.block<3,1>(0,3) = centroid_finalP.transpose();


	transf = C_B * R_new * C_A;

	Eigen::Quaterniond mat_rot(transf.block<3,3>(0,0));

	Eigen::Vector3d trasl = transf.block<3,1>(0,3).transpose();

	transfParameters_ << trasl(0), trasl(1), trasl(2), mat_rot.x(), mat_rot.y(), mat_rot.z(), mat_rot.w();

}

} // namespace phase_space

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseTracker");

    ros::NodeHandle nh;

    phase_space::PoseTracker node(nh,argc,argv);
  
    ros::spin();

    return 0;
}
