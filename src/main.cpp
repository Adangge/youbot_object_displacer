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

// start position
void moveArmstart() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(3).sleep();
	
}
	
// open gripper
void moveGripperopen() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.0115);
	gripperPublisher.publish(msg);

	ros::Duration(2).sleep();
}

// move close to the cube
void moveArmcloseto() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
}

// close gripper
void moveGripperclose() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
	ros::Duration(3).sleep();
}

// move arm back to the Ausgangsposition
void moveArmback() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
	
}

// move arm straight
void moveArmfold() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 0.11;
    jointvalues[2] = -0.11;
    jointvalues[3] = 0.11;
    jointvalues[4] = 0.11;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// move arm right
void moveArmright() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 0.11;
    jointvalues[2] = -0.11;
    jointvalues[3] = 0.11;
    jointvalues[4] = 0.11;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// move arm unfold
void moveArmunfold() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// put down the cube
void moveArmdown() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
	jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();

}

// open the gripper

void moveGripperopenagain() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0.0115);
	gripperPublisher.publish(msg);

	ros::Duration(2).sleep();
}

// move the arm back 
void moveArmup() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
	
}

// move arm close to cube again
void moveArmclosetoagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
	jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(3).sleep();

}

// close the gripper
void moveGrippercloseagain() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
	ros::Duration(2).sleep();
}

// move arm up again
void moveArmupagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
	
}

// move arm fold again
void moveArmfoldagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 0.11;
    jointvalues[2] = -0.11;
    jointvalues[3] = 0.11;
    jointvalues[4] = 0.11;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// move arm left

void moveArmleft() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 0.11;
    jointvalues[2] = -0.11;
    jointvalues[3] = 0.11;
    jointvalues[4] = 0.11;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// move arm unfold again
void moveArmunfoldagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
	
}

// put down the cube

void moveArmdownagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(3).sleep();
}

// open the gripper
void moveGripperopenagainagain() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0.0115);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();
}

// move arm up again
void moveArmbackagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(1).sleep();
	
}



/* // move arm to right

void moveArmright() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// put down the cube
void moveArmdown() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
	jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();

}

// open the gripper

void moveGripperopenagain() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0.0115);
	gripperPublisher.publish(msg);

	ros::Duration(2).sleep();
}

// move the arm back 
void moveArmup() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
	
}

// move arm close to cube again
void moveArmclosetoagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
	jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(3).sleep();

}

// close the gripper
void moveGrippercloseagain() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
	ros::Duration(2).sleep();
}

// move arm up again
void moveArmupagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 0.1511;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
	
}

//move arm to left
void moveArmleft() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
	
}

// put down the cube

void moveArmdownagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	 
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735 ;
    jointvalues[2] = -1.6580;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(3).sleep();
}

// open the gripper
void moveGripperopenagainagain() {
	brics_actuator::JointPositions msg;
	
	msg = createGripperPositionCommand(0.0115);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();
}

// move arm up again
void moveArmbackagain() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
    jointvalues[0] = 5.84014;
    jointvalues[1] = 1.846735;
    jointvalues[2] = -1.8950;
    jointvalues[3] = 3.02356;
    jointvalues[4] = 2.95;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(1).sleep();
	
} */


int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

	// movePlatform();
	moveArmstart();
	moveGripperopen();
	moveArmcloseto();
	moveGripperclose();
	moveArmback();
	moveArmfold();
	moveArmright();
	moveArmunfold();
	moveArmdown();
	moveGripperopenagain();
	moveArmup();
	moveArmclosetoagain();
	moveGrippercloseagain();
	moveArmupagain();
	moveArmfoldagain();
	moveArmleft();
	moveArmunfoldagain();
	moveArmdownagain();
	moveGripperopenagainagain();
	moveArmbackagain();
	
	/* moveArmright();
	moveArmdown();
	moveGripperopenagain();
	moveArmup();
	moveArmclosetoagain();
	moveGrippercloseagain();
	moveArmupagain();
	moveArmleft();
	moveArmdownagain();
	moveGripperopenagainagain();
	moveArmbackagain();*/

	sleep(1);
	ros::shutdown();

	return 0;
	
}

