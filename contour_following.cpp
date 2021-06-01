// Standard libraries
#include <cmath>
#include <string>
#include <cstdlib>
#include <tuple>
#include <iterator>
#include <algorithm>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>

// Yarp libraries
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/CartesianControl.h>

// Skin simulation libraries
#include <gazebo/Skin.hh>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/iKin/iKinFwd.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class ContourFollowingModule : public RFModule, public yarp::os::Thread {

private:
    BufferedPort<iCub::skinDynLib::skinContactList> port;
    PolyDriver drv_arm;
    PolyDriver drv_finger;
    ICartesianControl *cartControl;

    yarp::sig::Matrix R;
    yarp::sig::Vector x0{-0.2, 0.0, 0.3}, o0; //TODO tune starting position
    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};
    double y_min, y_max, y_delta, y;
    std::string robot, which_arm;
    int startup_context_id_arm;
    double period;

    auto helperWaitDevice(yarp::dev::PolyDriver &driver,
                          const yarp::os::Property &options,
                          const std::string &device_name) {
        const auto t0 = yarp::os::Time::now();
        while (yarp::os::Time::now() - t0 < 10.) {
            if (driver.open(const_cast<yarp::os::Property &>(options))) {
                return true;
            }
            yarp::os::Time::delay(1.);
        }

        yError() << "Unable to open the Device Driver:" << device_name;
        return false;
    }

public:
    bool configure(ResourceFinder &rf) {

        // Reading command-line parameters
        setName((rf.check("name", yarp::os::Value("/contour_following")).asString()).c_str());
        robot = rf.check("robot", yarp::os::Value("icubSim")).asString();
        which_arm = rf.check("arm", yarp::os::Value("right_arm")).asString();
        period = rf.check("period", Value(0.01)).asDouble();

        // Setting up device for moving the arm in cartesian space
        yarp::os::Property options_arm;
        options_arm.put("device", "cartesiancontrollerclient");
        options_arm.put("remote", "/" + robot + "/cartesianController/" + which_arm);
        options_arm.put("local", getName() + "/" + which_arm);
        if (!helperWaitDevice(drv_arm, options_arm, "Cartesian Controller")) {
            return false;
        }

        drv_arm.view(cartControl);

        IEncoders *iencs;

        Property prop_encoders;
        prop_encoders.put("device", "remote_controlboard");
        prop_encoders.put("local", getName() + "/controlboard/" + which_arm);
        prop_encoders.put("remote", "/" + robot + "/" + which_arm);
        if (drv_finger.open(prop_encoders)) {
            /* Try to retrieve the view. */
            if (!drv_finger.view(iencs)) {
                std::cout << "Could not view driver" << std::endl;

                return false;
            }
            // /* Try to retrieve the control limits view. */
            // if (!(drv_finger.view(ilimits_)) || (ilimits_ == nullptr))
            //     throw std::runtime_error(log_name_ + "::ctor. Error: unable get view for finger control limits.");
        } else {
            std::cout << "Could not open remote controlboard" << std::endl;
            return false;
        }


        int nEncs;
        iencs->getAxes(&nEncs);
        Vector encs(nEncs);
        iencs->getEncoders(encs.data());
        std::cout << "ENCODERS = " << encs.toString().c_str() << std::endl;
        Vector joints;
        iCub::iKin::iCubFinger finger("right_index"); // relevant code to get the position of the finger tip
        finger.getChainJoints(encs, joints);          // wrt the end-effector frame
        Matrix tipFrame = finger.getH((M_PI / 180.0) * joints);

        Vector tip_x = tipFrame.getCol(3);
        Vector tip_o = yarp::math::dcm2axis(tipFrame);
        //cartControl->attachTipFrame(tip_x, tip_o); // establish the new controlled frame

        cartControl->storeContext(&startup_context_id_arm); //Storing initial context for restoring it later
        cartControl->setTrajTime(
                .6);                       // Each trajectory would take this much amount of time to perform

        // Enabling all degrees of freedom
        yarp::sig::Vector dof;
        cartControl->getDOF(dof);
        dof = 1.;
        cartControl->setDOF(dof, dof);


        // Moving to starting position
        R = yarp::math::zeros(3, 3);

        R(0, 0) = -1.;
        R(2, 1) = -1.;
        R(1, 2) = -1.;
        o0 = yarp::math::dcm2axis(R);

        cartControl->goToPoseSync(x0, o0);
        cartControl->waitMotionDone(wait_ping, wait_tmo);
        cartControl->setPosePriority("position");

        y = y_max;

        // open and connect port for reading skin events
        port.open("/skin_events");
        if (!port.open("/skin_events")) {
            yError() << "Port could not open";
            return false;
        }
        yarp::os::Network::connect("/left_hand/skinManager/skin_events:o", "/skin_events");

        return Thread::start();
    }

    // Synchronous update every getPeriod() seconds
    bool updateModule() {

        //Reading skin events
        iCub::skinDynLib::skinContactList *input = port.read(true);
        if (input) {
            std::cout << input->toString() << std::endl;
        }

        // PUT YOUR CODE HERE

        return true;
    }

    // Asynchronous update that runs in a separate thread as fast as it can
    void run() {
//        yarp::sig::Vector Xwrist, Owrist;
//        cartControl->getPose(8, Xwrist, Owrist); // get position and orientation of the last joint (wrist yaw)
//
//        yarp::sig::Vector Xelbow, Oelbow;
//        cartControl->getPose(6, Xelbow, Oelbow); // get position and orientation of the last joint (wrist yaw)
//
//        double theta = -atan2(Xelbow[1] - Xwrist[1], Xelbow[0] - Xwrist[0]);
//
//        yarp::sig::Matrix Rtheta_y;
//        Rtheta_y = yarp::math::zeros(3, 3); // rotation matrix around y
//
//        Rtheta_y(0, 0) = cos(theta);
//        Rtheta_y(0, 1) = 0;
//        Rtheta_y(0, 2) = sin(theta);
//        Rtheta_y(1, 0) = 0;
//        Rtheta_y(1, 1) = 1;
//        Rtheta_y(1, 2) = 0;
//        Rtheta_y(2, 0) = -sin(theta);
//        Rtheta_y(2, 1) = 0;
//        Rtheta_y(2, 2) = cos(theta);
//
//        cartControl->goToPoseSync(x0 + yarp::sig::Vector{0., y, 0.}, yarp::math::dcm2axis(R * Rtheta_y));
//        cartControl->waitMotionDone(wait_ping, wait_tmo);
//
//        std::cout << y << std::endl;
//
//        yarp::sig::Vector xdhat, odhat, q_arm;
//        cartControl->getDesired(xdhat, odhat, q_arm);
//
//        yarp::sig::Vector xarm, oarm;
//        cartControl->getPose(xarm, oarm);
//
//        y -= y_delta;

        // y >= -y_min - y_delta/2.;

        // PUT YOUR CODE HERE
    }

    double getPeriod() {
        return period;
    }

    bool interruptModule() {
        yInfo() << "Interrupting module ...";

        return RFModule::interruptModule();
    }

    bool close() {
        yInfo() << "Closing the module...";

        cartControl->stopControl();
        cartControl->restoreContext(startup_context_id_arm);
        drv_arm.close();

        yInfo() << "...done!";
        return RFModule::close();
    }
};

int main(int argc, char *argv[]) {
    // Connecting to the yarp server (must be running already)
    Network yarp;
    if (!yarp.checkNetwork(2.0)) {
        yError() << "Network not found";
        return -1;
    }

    // Configuring and running the module
    ContourFollowingModule jCtrl;
    ResourceFinder rf;
    rf.configure(argc, argv);

    return jCtrl.runModule(rf);
}
