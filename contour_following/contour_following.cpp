
#include <cmath>
#include <string>

#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>

/*
// for YARP port
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Network;
*/

#include <iostream>

#include </home/simon/Code/force_control/force_controller_core/include/force_controller_core/force_controller.h>
#include </home/simon/Code/icub_haptic_exploration_environment/plugins/skin/include/gazebo/Skin.hh> // TODO improve include in the future
#include <iCub/skinDynLib/skinContact.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class jointControl : public RFModule {

private:
    // gazebo::GazeboYarpSkin skin;

    // force controller core
    // std::shared_ptr<fcc::JointForceController> jfc_ptr;

    /////////////////////////////////////////

    BufferedPort<iCub::skinDynLib::skinContactList> port;
    //BufferedPort<Bottle> port;

    BufferedPort<Bottle> right_arm_outPort;
    PolyDriver right_arm_jointDriver;
    IControlLimits *right_arm_ilim;
    IEncoders *right_arm_ienc;
    IControlMode *right_arm_imod;
    IPositionControl *right_arm_iposd;


    //////////////////////////////////////////
    /// \brief shoulder
    ///
    int right_arm_0_joint;
    double right_arm_0_min_lim, right_arm_0_max_lim;
    double right_arm_0_pos, right_arm_0_init_pos;
    double right_arm_0_p0, right_arm_0_p1;
    double right_arm_0_speed;

    int right_arm_1_joint;
    double right_arm_1_min_lim, right_arm_1_max_lim;
    double right_arm_1_pos, right_arm_1_init_pos;
    double right_arm_1_p0, right_arm_1_p1;
    double right_arm_1_speed;

    int right_arm_2_joint;
    double right_arm_2_min_lim, right_arm_2_max_lim;
    double right_arm_2_pos, right_arm_2_init_pos;
    double right_arm_2_p0, right_arm_2_p1;
    double right_arm_2_speed;

    //////////////////////////////////////////
    /// \brief elbow
    ///
    int right_arm_3_joint;
    double right_arm_3_min_lim, right_arm_3_max_lim;
    double right_arm_3_pos, right_arm_3_init_pos;
    double right_arm_3_p0, right_arm_3_p1;
    double right_arm_3_speed;

    //////////////////////////////////////////
    /// \brief wrist
    ///
    int right_arm_4_joint;
    double right_arm_4_min_lim, right_arm_4_max_lim;
    double right_arm_4_pos, right_arm_4_init_pos;
    double right_arm_4_p0, right_arm_4_p1;
    double right_arm_4_speed;

    int right_arm_5_joint;
    double right_arm_5_min_lim, right_arm_5_max_lim;
    double right_arm_5_pos, right_arm_5_init_pos;
    double right_arm_5_p0, right_arm_5_p1;
    double right_arm_5_speed;

    int right_arm_6_joint;
    double right_arm_6_min_lim, right_arm_6_max_lim;
    double right_arm_6_pos, right_arm_6_init_pos;
    double right_arm_6_p0, right_arm_6_p1;
    double right_arm_6_speed;

    //////////////////////////////////////////
    /// \brief hand
    ///

    int right_arm_7_joint;
    double right_arm_7_min_lim, right_arm_7_max_lim;
    double right_arm_7_pos, right_arm_7_init_pos;
    double right_arm_7_p0, right_arm_7_p1;
    double right_arm_7_speed;

    int right_arm_8_joint;
    double right_arm_8_min_lim, right_arm_8_max_lim;
    double right_arm_8_pos, right_arm_8_init_pos;
    double right_arm_8_p0, right_arm_8_p1;
    double right_arm_8_speed;

    int right_arm_9_joint;
    double right_arm_9_min_lim, right_arm_9_max_lim;
    double right_arm_9_pos, right_arm_9_init_pos;
    double right_arm_9_p0, right_arm_9_p1;
    double right_arm_9_speed;

    int right_arm_10_joint;
    double right_arm_10_min_lim, right_arm_10_max_lim;
    double right_arm_10_pos, right_arm_10_init_pos;
    double right_arm_10_p0, right_arm_10_p1;
    double right_arm_10_speed;

    int right_arm_11_joint;
    double right_arm_11_min_lim, right_arm_11_max_lim;
    double right_arm_11_pos, right_arm_11_init_pos;
    double right_arm_11_p0, right_arm_11_p1;
    double right_arm_11_speed;

    int right_arm_12_joint;
    double right_arm_12_min_lim, right_arm_12_max_lim;
    double right_arm_12_pos, right_arm_12_init_pos;
    double right_arm_12_p0, right_arm_12_p1;
    double right_arm_12_speed;

    int right_arm_13_joint;
    double right_arm_13_min_lim, right_arm_13_max_lim;
    double right_arm_13_pos, right_arm_13_init_pos;
    double right_arm_13_p0, right_arm_13_p1;
    double right_arm_13_speed;

    int right_arm_14_joint;
    double right_arm_14_min_lim, right_arm_14_max_lim;
    double right_arm_14_pos, right_arm_14_init_pos;
    double right_arm_14_p0, right_arm_14_p1;
    double right_arm_14_speed;

    int right_arm_15_joint;
    double right_arm_15_min_lim, right_arm_15_max_lim;
    double right_arm_15_pos, right_arm_15_init_pos;
    double right_arm_15_p0, right_arm_15_p1;
    double right_arm_15_speed;

    /////////////////////////////////////////

    double period;

public:
    bool configure(ResourceFinder &rf) {
        // open and connect port here:
        port.open("/max_val");
        if (!port.open("/max_val"))
        {
            yError() << "Port could not open";
            return false;
        }
        yarp::os::Network::connect("/left_hand/skinManager/skin_events:o", "/max_val", "fast_tcp");

        // get simulation true or false
        int sim = rf.check("simulation", Value(1)).asInt();

        // get parameters
        setName((rf.check("name", Value("/hapticFeedBackMotion")).asString()).c_str());

        // force controller core
        // std::shared_ptr<double> fp = std::make_shared<double>(0.0);
        // jfc_ptr = std::make_shared<fcc::JointForceController>("", fp);

        std::string right_arm_body_part = "right_arm";

        /////////////////////////////////////////
        /// \brief shoulder
        ///

        right_arm_0_joint = 0;
        right_arm_0_p0 = rf.check("right_arm_0_p0", Value(-95.0)).asDouble();
        right_arm_0_p1 = rf.check("right_arm_0_p1", Value(10.0)).asDouble();
        right_arm_0_speed = rf.check("right_arm_0_speed", Value(100.0)).asDouble();

        right_arm_1_joint = 1;
        right_arm_1_p0 = rf.check("right_arm_1_p0", Value(0.0)).asDouble();
        right_arm_1_p1 = rf.check("right_arm_1_p1", Value(160.0)).asDouble();
        right_arm_1_speed = rf.check("right_arm_1_speed", Value(100.0)).asDouble();

        right_arm_2_joint = 2;
        right_arm_2_p0 = rf.check("right_arm_1_p0", Value(-37.0)).asDouble();
        right_arm_2_p1 = rf.check("right_arm_1_p1", Value(80.0)).asDouble();
        right_arm_2_speed = rf.check("right_arm_1_speed", Value(100.0)).asDouble();

        /////////////////////////////////////////
        /// \brief elbow
        ///

        right_arm_3_joint = 3;
        right_arm_3_p0 = rf.check("right_arm_3_p0", Value(15.0)).asDouble();
        right_arm_3_p1 = rf.check("right_arm_3_p1", Value(106.0)).asDouble();
        right_arm_3_speed = rf.check("right_arm_3_speed", Value(100.0)).asDouble();

        /////////////////////////////////////////
        /// \brief wrist
        ///

        right_arm_4_joint = 4;
        right_arm_4_p0 = rf.check("right_arm_4_p0", Value(-60.0)).asDouble();
        right_arm_4_p1 = rf.check("right_arm_4_p1", Value(60.0)).asDouble();
        right_arm_4_speed = rf.check("right_arm_4_speed", Value(100.0)).asDouble();

        right_arm_5_joint = 5;
        right_arm_5_p0 = rf.check("right_arm_5_p0", Value(-80.0)).asDouble();
        right_arm_5_p1 = rf.check("right_arm_5_p1", Value(25.0)).asDouble();
        right_arm_5_speed = rf.check("right_arm_5_speed", Value(100.0)).asDouble();

        right_arm_6_joint = 6;
        right_arm_6_p0 = rf.check("right_arm_6_p0", Value(-20.0)).asDouble();
        right_arm_6_p1 = rf.check("right_arm_6_p1", Value(25.0)).asDouble();
        right_arm_6_speed = rf.check("right_arm_6_speed", Value(100.0)).asDouble();

        /////////////////////////////////////////
        /// \brief hand
        ///

        right_arm_7_joint = 7;
        right_arm_7_p0 = rf.check("right_arm_7_p0", Value(-90.0)).asDouble();
        right_arm_7_p1 = rf.check("right_arm_7_p1", Value(90.0)).asDouble();
        right_arm_7_speed = rf.check("right_arm_7_speed", Value(100.0)).asDouble();

        right_arm_8_joint = 8;
        right_arm_8_p0 = rf.check("right_arm_8_p0", Value(-60.0)).asDouble();
        right_arm_8_p1 = rf.check("right_arm_8_p1", Value(60.0)).asDouble();
        right_arm_8_speed = rf.check("right_arm_8_speed", Value(100.0)).asDouble();

        right_arm_9_joint = 9;
        right_arm_9_p0 = rf.check("right_arm_9_p0", Value(0.0)).asDouble();
        right_arm_9_p1 = rf.check("right_arm_9_p1", Value(90.0)).asDouble();
        right_arm_9_speed = rf.check("right_arm_9_speed", Value(100.0)).asDouble();

        right_arm_10_joint = 10;
        right_arm_10_p0 = rf.check("right_arm_10_p0", Value(0.0)).asDouble();
        right_arm_10_p1 = rf.check("right_arm_10_p1", Value(90.0)).asDouble();
        right_arm_10_speed = rf.check("right_arm_10_speed", Value(100.0)).asDouble();

        right_arm_11_joint = 11;
        right_arm_11_p0 = rf.check("right_arm_11_p0", Value(0.0)).asDouble();
        right_arm_11_p1 = rf.check("right_arm_11_p1", Value(90.0)).asDouble();
        right_arm_11_speed = rf.check("right_arm_11_speed", Value(100.0)).asDouble();

        right_arm_12_joint = 12;
        right_arm_12_p0 = rf.check("right_arm_12_p0", Value(0.0)).asDouble();
        right_arm_12_p1 = rf.check("right_arm_12_p1", Value(90.0)).asDouble();
        right_arm_12_speed = rf.check("right_arm_12_speed", Value(100.0)).asDouble();

        right_arm_13_joint = 13;
        right_arm_13_p0 = rf.check("right_arm_13_p0", Value(0.0)).asDouble();
        right_arm_13_p1 = rf.check("right_arm_13_p1", Value(90.0)).asDouble();
        right_arm_13_speed = rf.check("right_arm_13_speed", Value(100.0)).asDouble();

        right_arm_14_joint = 14;
        right_arm_14_p0 = rf.check("right_arm_14_p0", Value(0.0)).asDouble();
        right_arm_14_p1 = rf.check("right_arm_14_p1", Value(90.0)).asDouble();
        right_arm_14_speed = rf.check("right_arm_14_speed", Value(100.0)).asDouble();

        right_arm_15_joint = 15;
        right_arm_15_p0 = rf.check("right_arm_15_p0", Value(0.0)).asDouble();
        right_arm_15_p1 = rf.check("right_arm_15_p1", Value(90.0)).asDouble();
        right_arm_15_speed = rf.check("right_arm_15_speed", Value(100.0)).asDouble();

        /////////////////////////////////////////
        period = rf.check("period", Value(0.01)).asDouble();

        // open communication with encoders
        Property optArm;
        optArm.put("device", "remote_controlboard");
        optArm.put("local", "/encReader/" + getName() + "/" + right_arm_body_part);

        if (sim) {
            optArm.put("remote", "/icubSim/" + right_arm_body_part);
            if (!right_arm_jointDriver.open(optArm)) {
                yError() << "Unable to connect to /icubSim/" << right_arm_body_part;
                return false;
            }
        } else {
            optArm.put("remote", "/icub/" + right_arm_body_part);
            if (!right_arm_jointDriver.open(optArm)) {
                yError() << "Unable to connect to /icub/" << right_arm_body_part;
                return false;
            }
        }

        /////////////////////////////////////////////////

        bool ok = true;
        ok = ok && right_arm_jointDriver.view(right_arm_ienc);
        ok = ok && right_arm_jointDriver.view(right_arm_ilim);
        ok = ok && right_arm_jointDriver.view(right_arm_imod);
        ok = ok && right_arm_jointDriver.view(right_arm_iposd);

        if (!ok) {
            yError() << "Unable to open views";
            return false;
        }

        // open output port
        if (!right_arm_outPort.open("/hapticFeedBackMotion/" + getName() + "/pos:o")) {
            yError() << "Unable to open hapticFeedBackMotion port";
            return false;
        }

        // compute joint limits shoulder
        right_arm_ilim->getLimits(right_arm_0_joint, &right_arm_0_min_lim, &right_arm_0_max_lim);
        yWarning() << "joint 0 min limit: " << right_arm_0_min_lim << " -   joint 0 max limit: " << right_arm_0_max_lim;

        right_arm_ilim->getLimits(right_arm_1_joint, &right_arm_1_min_lim, &right_arm_1_max_lim);
        yWarning() << "joint 1 min limit: " << right_arm_1_min_lim << " -   joint 1 max limit: " << right_arm_1_max_lim;

        right_arm_ilim->getLimits(right_arm_2_joint, &right_arm_2_min_lim, &right_arm_2_max_lim);
        yWarning() << "joint 2 min limit: " << right_arm_2_min_lim << " -   joint 2 max limit: " << right_arm_2_max_lim;

        // compute joint limits elbow
        right_arm_ilim->getLimits(right_arm_3_joint, &right_arm_3_min_lim, &right_arm_3_max_lim);
        yWarning() << "joint 3 min limit: " << right_arm_3_min_lim << " -   joint 3 max limit: " << right_arm_3_max_lim;

        // compute joint limits wrist
        right_arm_ilim->getLimits(right_arm_4_joint, &right_arm_4_min_lim, &right_arm_4_max_lim);
        yWarning() << "joint 4 min limit: " << right_arm_4_min_lim << " -   joint 4 max limit: " << right_arm_4_max_lim;

        right_arm_ilim->getLimits(right_arm_5_joint, &right_arm_5_min_lim, &right_arm_5_max_lim);
        yWarning() << "joint 5 min limit: " << right_arm_5_min_lim << " -   joint 5 max limit: " << right_arm_5_max_lim;

        right_arm_ilim->getLimits(right_arm_6_joint, &right_arm_6_min_lim, &right_arm_6_max_lim);
        yWarning() << "joint 6 min limit: " << right_arm_6_min_lim << " -   joint 6 max limit: " << right_arm_6_max_lim;

        // compute joint limits hand
        right_arm_ilim->getLimits(right_arm_7_joint, &right_arm_7_min_lim, &right_arm_7_max_lim);
        yWarning() << "joint 7 min limit: " << right_arm_7_min_lim << " -   joint 7 max limit: " << right_arm_7_max_lim;

        right_arm_ilim->getLimits(right_arm_8_joint, &right_arm_8_min_lim, &right_arm_8_max_lim);
        yWarning() << "joint 8 min limit: " << right_arm_8_min_lim << " -   joint 8 max limit: " << right_arm_8_max_lim;

        right_arm_ilim->getLimits(right_arm_9_joint, &right_arm_9_min_lim, &right_arm_9_max_lim);
        yWarning() << "joint 9 min limit: " << right_arm_9_min_lim << " -   joint 9 max limit: " << right_arm_9_max_lim;

        right_arm_ilim->getLimits(right_arm_10_joint, &right_arm_10_min_lim, &right_arm_10_max_lim);
        yWarning() << "joint 10 min limit: " << right_arm_10_min_lim << " -   joint 10 max limit: "
                   << right_arm_10_max_lim;

        right_arm_ilim->getLimits(right_arm_11_joint, &right_arm_11_min_lim, &right_arm_11_max_lim);
        yWarning() << "joint 11 min limit: " << right_arm_11_min_lim << " -   joint 11 max limit: "
                   << right_arm_11_max_lim;

        right_arm_ilim->getLimits(right_arm_12_joint, &right_arm_12_min_lim, &right_arm_12_max_lim);
        yWarning() << "joint 12 min limit: " << right_arm_12_min_lim << " -   joint 12 max limit: "
                   << right_arm_12_max_lim;

        right_arm_ilim->getLimits(right_arm_13_joint, &right_arm_13_min_lim, &right_arm_13_max_lim);
        yWarning() << "joint 13 min limit: " << right_arm_13_min_lim << " -   joint 13 max limit: "
                   << right_arm_13_max_lim;

        right_arm_ilim->getLimits(right_arm_14_joint, &right_arm_14_min_lim, &right_arm_14_max_lim);
        yWarning() << "joint 14 min limit: " << right_arm_14_min_lim << " -   joint 14 max limit: "
                   << right_arm_14_max_lim;

        right_arm_ilim->getLimits(right_arm_15_joint, &right_arm_15_min_lim, &right_arm_15_max_lim);
        yWarning() << "joint 15 min limit: " << right_arm_15_min_lim << " -   joint 15 max limit: "
                   << right_arm_15_max_lim;

        ////////////////////////////////

        // set control mode shoulder
        right_arm_imod->setControlMode(right_arm_0_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_1_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_2_joint, VOCAB_CM_POSITION);

        // set control mode elbow
        right_arm_imod->setControlMode(right_arm_3_joint, VOCAB_CM_POSITION);

        // set control mode wrist
        right_arm_imod->setControlMode(right_arm_4_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_5_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_6_joint, VOCAB_CM_POSITION);

        // set control mode hand
        right_arm_imod->setControlMode(right_arm_7_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_8_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_9_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_10_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_11_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_12_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_13_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_14_joint, VOCAB_CM_POSITION);

        right_arm_imod->setControlMode(right_arm_15_joint, VOCAB_CM_POSITION);

        // check that amplitude for shoulder !> motion allowed by the robot
        right_arm_ienc->getEncoder(right_arm_0_joint, &right_arm_0_pos);
        right_arm_iposd->setRefSpeed(right_arm_0_joint, right_arm_0_speed);

        right_arm_ienc->getEncoder(right_arm_1_joint, &right_arm_1_pos);
        right_arm_iposd->setRefSpeed(right_arm_1_joint, right_arm_1_speed);

        right_arm_ienc->getEncoder(right_arm_2_joint, &right_arm_2_pos);
        right_arm_iposd->setRefSpeed(right_arm_2_joint, right_arm_2_speed);

        // check that amplitude for shoulder !> motion allowed by the robot
        right_arm_ienc->getEncoder(right_arm_3_joint, &right_arm_3_pos);
        right_arm_iposd->setRefSpeed(right_arm_3_joint, right_arm_3_speed);

        // check that amplitude for wrist !> motion allowed by the robot
        right_arm_ienc->getEncoder(right_arm_4_joint, &right_arm_4_pos);
        right_arm_iposd->setRefSpeed(right_arm_4_joint, right_arm_4_speed);

        right_arm_ienc->getEncoder(right_arm_5_joint, &right_arm_5_pos);
        right_arm_iposd->setRefSpeed(right_arm_5_joint, right_arm_5_speed);

        right_arm_ienc->getEncoder(right_arm_6_joint, &right_arm_6_pos);
        right_arm_iposd->setRefSpeed(right_arm_6_joint, right_arm_6_speed);

        // check that amplitude for hand !> motion allowed by the robot
        right_arm_ienc->getEncoder(right_arm_7_joint, &right_arm_7_pos);
        right_arm_iposd->setRefSpeed(right_arm_7_joint, right_arm_7_speed);

        right_arm_ienc->getEncoder(right_arm_8_joint, &right_arm_8_pos);
        right_arm_iposd->setRefSpeed(right_arm_8_joint, right_arm_8_speed);

        right_arm_ienc->getEncoder(right_arm_9_joint, &right_arm_9_pos);
        right_arm_iposd->setRefSpeed(right_arm_9_joint, right_arm_9_speed);

        right_arm_ienc->getEncoder(right_arm_10_joint, &right_arm_10_pos);
        right_arm_iposd->setRefSpeed(right_arm_10_joint, right_arm_10_speed);

        right_arm_ienc->getEncoder(right_arm_11_joint, &right_arm_11_pos);
        right_arm_iposd->setRefSpeed(right_arm_11_joint, right_arm_11_speed);

        right_arm_ienc->getEncoder(right_arm_12_joint, &right_arm_12_pos);
        right_arm_iposd->setRefSpeed(right_arm_12_joint, right_arm_12_speed);

        right_arm_ienc->getEncoder(right_arm_13_joint, &right_arm_13_pos);
        right_arm_iposd->setRefSpeed(right_arm_13_joint, right_arm_13_speed);

        right_arm_ienc->getEncoder(right_arm_14_joint, &right_arm_14_pos);
        right_arm_iposd->setRefSpeed(right_arm_14_joint, right_arm_14_speed);

        right_arm_ienc->getEncoder(right_arm_15_joint, &right_arm_15_pos);
        right_arm_iposd->setRefSpeed(right_arm_15_joint, right_arm_15_speed);

        Time::delay(1);

        //////////////////////////////

        // move to initial position
        // lift hand to allow thumb to go in opposite
        /*right_arm_3_init_pos = 60.97;
        right_arm_iposd->setRefSpeed(right_arm_3_joint, 10);
        right_arm_iposd->positionMove(right_arm_3_joint, right_arm_3_init_pos);
        yInfo() << "Waiting 1 seconds to reach right_arm_3_init_pose";
        right_arm_4_init_pos = -14.5;
        right_arm_iposd->setRefSpeed(right_arm_4_joint, 100);
        right_arm_iposd->positionMove(right_arm_4_joint, right_arm_4_init_pos);
        yInfo() << "Waiting 1 seconds to reach right_arm_4_init_pose";
        Time::delay(5);
        // opposite thumb
        right_arm_8_init_pos = 90;
        right_arm_iposd->setRefSpeed(right_arm_8_joint, 100);
        right_arm_iposd->positionMove(right_arm_8_joint, right_arm_8_init_pos);
        yInfo() << "Waiting 1 seconds to reach right_arm_8_init_pose";
        Time::delay(5);
        // reach for object
        right_arm_0_init_pos = -59.85;
        right_arm_iposd->setRefSpeed(right_arm_0_joint, 10);
        right_arm_iposd->positionMove(right_arm_0_joint, right_arm_0_init_pos);
        yInfo() << "Waiting 1 seconds to reach right_arm_0_init_pose";
        right_arm_3_init_pos = 28.21;
        right_arm_iposd->setRefSpeed(right_arm_3_joint, 10);
        right_arm_iposd->positionMove(right_arm_3_joint, right_arm_3_init_pos);
        yInfo() << "Waiting 1 seconds to reach right_arm_3_init_pose";
        Time::delay(1);
        right_arm_2_init_pos = 11.7;
        right_arm_iposd->setRefSpeed(right_arm_2_joint, 10);
        right_arm_iposd->positionMove(right_arm_2_joint, right_arm_2_init_pos);
        yInfo() << "Waiting 1 seconds to reach right_arm_2_init_pose";
        Time::delay(5);*/

        return true;
    }

    bool updateModule() {

        yInfo() << "reading data";
        /*Bottle* input = port.read();
        if(input) {
            yInfo() << "I READ SOME DATA";
        }*/
        // skinContact = port.read(true);
        // std::cout << skinContact << std::endl;
        // port.read(true);


        iCub::skinDynLib::skinContactList *input = port.read(true);
        if(input) {
            //yInfo() << "I READ SOME DATA";
            std::cout << input->toString() << std::endl;
        }


        return true;
    }

    double getPeriod() {
        return period;
    }

    bool interruptModule() {
        yInfo() << "Interrupting module ...";
        right_arm_outPort.interrupt();

        return RFModule::interruptModule();
    }

    bool close() {
        yInfo() << "Closing the module...";
        right_arm_jointDriver.close();
        right_arm_outPort.close();

        yInfo() << "...done!";
        return RFModule::close();
    }
};

int main(int argc, char *argv[]) {
    Network yarp;
    if (!yarp.checkNetwork(2.0)) {
        yError() << "Network not found";
        return -1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    jointControl jCtrl;
    return jCtrl.runModule(rf);
}
