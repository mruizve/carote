#include<tf/transform_datatypes.h>
#include<tf_conversions/tf_eigen.h>
#include<carote_msgs/OperatorStamped.h>
#include "carote/Operator.h"
#include "carote/Utils.h"

carote::Operator::Operator(const std::string& _name)
:
	node_("~"),
	name_(_name)
{
	std::string strpar;

	// prepare for advertise
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/operator' (must be defined at setup.launch)");
	}
	pub_command_=node_.advertise<carote_msgs::OperatorStamped>(strpar,1);

	// get frame id of the target and desired pose
	if( !node_.getParam("/carote/frames/goal",frame_id_goal_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/goal' (must be defined at setup.launch)");
	}

	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/target' (must be defined at setup.launch");
	}

	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::OperatorConfig>::CallbackType f;
	f=boost::bind(&carote::Operator::cbReconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Operator::~Operator(void)
{
}

void carote::Operator::cbReconfigure(carote::OperatorConfig& _config, uint32_t _level)
{
	// prepare operator message
	carote_msgs::OperatorStamped msg;
	msg.header.stamp=ros::Time::now();
	msg.data.lambda=_config.lambda;
	msg.data.phi=_config.phi;
	msg.data.rho=_config.rho;
	msg.data.z_lower=_config.z_lower;
	msg.data.z_upper=_config.z_upper;

	// send operator parameters to the controller
	pub_command_.publish(msg);

	// compute desired pose in target coordinates (as a tf::Transform)
	tf::Vector3 t;
	t[0]= _config.rho*cos(_config.phi*M_PI/180.0)*cos(M_PI_2+_config.lambda*M_PI/180.0);
	t[1]=-_config.rho*sin(_config.phi*M_PI/180.0);
	t[2]= _config.rho*cos(_config.phi*M_PI/180.0)*sin(M_PI_2+_config.lambda*M_PI/180.0);

	Eigen::Vector3d _t,x,y,z;
	_t << t[0],t[1],t[2];
	z=-_t/_t.norm();
	x=z.cross(-Eigen::Vector3d::UnitY());
	x=x/x.norm();
	y=z.cross(x);
	y=y/y.norm();

	Eigen::Matrix3d _R;
	_R << x,y,z;

	tf::Matrix3x3 R;
	tf::matrixEigenToTF(_R,R);

	tf_target_goal_=tf::Transform(R,t);

	// broadcast desired pose frame
	tf_broadcaster_.sendTransform(tf::StampedTransform(tf_target_goal_,msg.header.stamp,frame_id_target_,frame_id_goal_));
}
