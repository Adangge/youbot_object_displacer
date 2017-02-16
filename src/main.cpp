
//
// Simple demo program that calls the youBot ROS wrapper
//

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}


// move platform a little bit back- and forward and to the left and right
/* void movePlatform() {
	geometry_msgs::Twist twist;

	// forward
	twist.linear.x = 0.05;  // with 0.05 m per sec
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// backward
	twist.linear.x = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the left
	twist.linear.x = 0;
	twist.linear.y = 0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the right
	twist.linear.y = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(10).sleep();

	// stop
	twist.linear.y = 0;
	platformPublisher.publish(twist);
} */

// move arm once up and down
void moveArm() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	 // move arm straight up. values were determined empirically
    jointvalues[0] = 5.147;
    jointvalues[1] = 0.56874;
    jointvalues[2] = -4.695891;
    jointvalues[3] = 2.94531;
    jointvalues[4] = 1.3333;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

   /* // move arm back close to calibration position
    jointvalues[0] = 4.5555;
    jointvalues[1] = 2.4444;
    jointvalues[2] = -4.95143;
    jointvalues[3] = 2.29855;
    jointvalues[4] = 0.21111;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = 2.16421;
    jointvalues[1] = 1.37512 ;
    jointvalues[2] = -1.528414;
    jointvalues[3] = 3.166033;
    jointvalues[4] = 0.3222;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = 2.44251;
    jointvalues[1] = 2.52041 ;
    jointvalues[2] = -1.76995;
    jointvalues[3] = 0.97741;
    jointvalues[4] = 3.666;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = 2.94961;
    jointvalues[1] = 1.22819 ;
    jointvalues[2] = -1.06134;
    jointvalues[3] = 1.56102;
    jointvalues[4] = 4.555;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = 2.94961;
    jointvalues[1] = 0.84809 ;
    jointvalues[2] = -1.49785;
    jointvalues[3] = 2.34624;
    jointvalues[4] = 2.333;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep(); */

    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.5378;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 0.67777;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

}

// open and close gripper
void moveGripper() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();

	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

	// movePlatform();
	moveArm();
	moveGripper();

	sleep(1);
	ros::shutdown();

	return 0;
}

