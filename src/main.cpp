/******************************************************************************
* Author:
* Julien Van Loo
******************************************************************************/

#include <vector>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

using namespace std;
using namespace youbot;

YouBotBase* createBase() {
    /* create a youbot base, return the base or NULL if could not create it */
    YouBotBase* base;
    try {
        base = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
        base->doJointCommutation();
    } catch (std::exception& e) {
        LOG(warning) << e.what();
        base = NULL;
    }
    return base;
}

YouBotManipulator* createArm() {
    /* create a youbot arm, return the arm or NULL if could not create it */
    YouBotManipulator* arm;
    try {
        arm = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
        arm->doJointCommutation();
        arm->calibrateManipulator();
    } catch (std::exception& e) {
        LOG(warning) << e.what();
        arm = NULL;
    }
    return arm;
}

void setBasePosition(YouBotBase* base, vector<int> position, int orientation, int velocity) {
    base->setBaseVelocity(longitudinalVelocity, transversalVelocity, orientation*radians_per_second);
}

int main() {
	/* define velocities */
	double translationalVelocity = 0.05; //meter_per_second
	double rotationalVelocity = 0.2; //radian_per_second

	/* create handles for youBot base and manipulator (if available) */
    YouBotBase* base = createBase();
    YouBotManipulator* arm = createArm();

	/*
	* Variable for the base.
	* Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
	*/
	quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

	/* Variable for the arm. */
	JointAngleSetpoint desiredJointAngle;

	try {
		/*
		 * Simple sequence of commands to the youBot:
		 */

        if (base) {

			/* forward */
			longitudinalVelocity = translationalVelocity * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
            base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive forward";
			SLEEP_MILLISEC(2000);

			/* backwards */
			longitudinalVelocity = -translationalVelocity * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
            base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive backwards";
			SLEEP_MILLISEC(2000);

			/* left */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = translationalVelocity * meter_per_second;
			angularVelocity = 0 * radian_per_second;
            base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive left";
			SLEEP_MILLISEC(2000);

			/* right */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = -translationalVelocity * meter_per_second;
			angularVelocity = 0 * radian_per_second;
            base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "drive right";
			SLEEP_MILLISEC(2000);

			/* stop base */
			longitudinalVelocity = 0 * meter_per_second;
			transversalVelocity = 0 * meter_per_second;
			angularVelocity = 0 * radian_per_second;
            base->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			LOG(info) << "stop base";
		}

        if (arm) {

			/* unfold arm 
			 * all of the following constants are empirically determined to move the arm into the desired position 
			 */
			desiredJointAngle.angle = 2.56244 * radian;
            arm->getArmJoint(1).setData(desiredJointAngle);

			desiredJointAngle.angle = 1.04883 * radian;
            arm->getArmJoint(2).setData(desiredJointAngle);

			desiredJointAngle.angle = -2.43523 * radian;
            arm->getArmJoint(3).setData(desiredJointAngle);

			desiredJointAngle.angle = 1.73184 * radian;
            arm->getArmJoint(4).setData(desiredJointAngle);
			LOG(info) << "unfold arm";
			SLEEP_MILLISEC(4000);

			/* fold arm (approx. home position) using empirically determined values for the positions */
			desiredJointAngle.angle = 0.11 * radian;
            arm->getArmJoint(1).setData(desiredJointAngle);

			desiredJointAngle.angle = 0.11 * radian;
            arm->getArmJoint(2).setData(desiredJointAngle);

			desiredJointAngle.angle = -0.11 * radian;
            arm->getArmJoint(3).setData(desiredJointAngle);
			desiredJointAngle.angle = 0.11 * radian;
            arm->getArmJoint(4).setData(desiredJointAngle);
			LOG(info) << "fold arm";
			SLEEP_MILLISEC(4000);
		}

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		std::cout << "unhandled exception" << std::endl;
	}

	/* clean up */
    if (base) {
        delete base;
        //myYouBotBase = 0;
	}
    if (arm) {
        delete arm;
        //myYouBotManipulator = 0;
	}

	LOG(info) << "Done.";

	return 0;
}

