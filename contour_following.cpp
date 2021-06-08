// Standard libraries
#ifdef _WIN32
#include <Windows.h>
#else

#include <unistd.h>

#endif

#include <cmath>
#include <string>
#include <cstdlib>
#include <vector>
#include <iostream>

// Yarp libraries
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/CartesianControl.h>

// Skin simulation libraries
#include <gazebo/Skin.hh>
#include <iCub/iKin/iKinFwd.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class ContourFollowingModule : public RFModule, public yarp::os::Thread {

private:
    BufferedPort<iCub::skinDynLib::skinContactList> skinEventsPort;
    PolyDriver drv_arm;
    PolyDriver drv_finger;
    ICartesianControl *cartControl;
    IPositionControl *posControl;
    IEncoders *iencs;

    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};
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

    void closeHand() {
        std::vector<int> fingerjoints = {7, 11, 12, 13, 14, 15};
        std::vector<double> fingerClosedPos = {60, 0, 2, 82, 140, 230};
        posControl->positionMove(fingerjoints.size(), fingerjoints.data(), fingerClosedPos.data());

        bool done = false;
        double now = Time::now();

        while (!done && Time::now() - now < wait_tmo) {
            sleep(1);
            posControl->checkMotionDone(&done);
        }

        std::vector<int> thumbjoints = {8, 9, 10};
        std::vector<double> thumbClosedPos = {37, 27, 103};
        posControl->positionMove(thumbjoints.size(), thumbjoints.data(), thumbClosedPos.data());

        now = Time::now();
        done = false;
        while (!done && Time::now() - now < wait_tmo) {
            sleep(1);
            posControl->checkMotionDone(&done);
        }
    }

    void parseParams(const ResourceFinder &rf) {
        setName((rf.check("name", Value("/contour_following")).asString()).c_str());
        robot = rf.check("robot", Value("icubSim")).asString();
        which_arm = rf.check("arm", Value("right_arm")).asString();
        period = rf.check("period", Value(0.01)).asDouble();
    }

    bool openDrivers() {
        // Setting up device for moving the arm in cartesian space
        yarp::os::Property options_arm;
        options_arm.put("device", "cartesiancontrollerclient");
        options_arm.put("remote", "/" + robot + "/cartesianController/" + which_arm);
        options_arm.put("local", getName() + "/" + which_arm);
        if (!helperWaitDevice(drv_arm, options_arm, "Cartesian Controller")) {
            return false;
        }

        drv_arm.view(cartControl);

        Property prop_encoders;
        prop_encoders.put("device", "remote_controlboard");
        prop_encoders.put("local", getName() + "/controlboard/" + which_arm);
        prop_encoders.put("remote", "/" + robot + "/" + which_arm);
        if (drv_finger.open(prop_encoders)) {
            if (!drv_finger.view(iencs) || !drv_finger.view(posControl)) {
                std::cout << "Could not view driver" << std::endl;
                return false;
            }

        } else {
            std::cout << "Could not open remote controlboard" << std::endl;
            return false;
        }
        return true;
    }

    bool moveEndEffectorToFingertip() {

        int nEncs;
        iencs->getAxes(&nEncs);
        yarp::sig::Vector encs(nEncs);
        // try to read the joint encoders
        std::size_t counter = 0;
        while (!iencs->getEncoders(encs.data())) {
            if (++counter == 50) {
                std::cout << "Error while reading the encoders.";
                return false;
            }
            usleep(100000);
        }
        Vector joints;
        iCub::iKin::iCubFinger finger("right_index"); // relevant code to get the position of the finger tip
        finger.getChainJoints(encs, joints);          // wrt the end-effector frame
        Matrix tipFrame = finger.getH((M_PI / 180.0) * joints);

        Vector tip_x = tipFrame.getCol(3);
        Vector tip_o = yarp::math::dcm2axis(tipFrame);
        cartControl->attachTipFrame(tip_x, tip_o); // establish the new controlled frame
        return true;
    }

    bool configure(ResourceFinder &rf) {

        parseParams(rf);
        if (!openDrivers()) return false;
        if (!moveEndEffectorToFingertip()) return false;

        // Set reference speed for all arm joints
        std::vector<double> speeds(15, 30);

        posControl->setRefSpeeds(speeds.data());
        closeHand();

        cartControl->storeContext(&startup_context_id_arm);    //Storing initial context for restoring it later
        cartControl->setTrajTime(
                .6);                       // Each trajectory would take this much amount of time to perform

        // Enabling all degrees of freedom
        yarp::sig::Vector dof;
        cartControl->getDOF(dof);
        dof = 1.;
        cartControl->setDOF(dof, dof);

        //--- Moving to starting pose ---//
        yarp::sig::Vector x0{-0.4, 0.1, 0.1}; // Starting end effector position

        //Rotation from root frame to end effector pointing straight ahead
        yarp::sig::Matrix R = yarp::math::zeros(3, 3);
        R(0, 0) = -1.;
        R(1, 1) = 1.;
        R(2, 2) = -1.;

        // Transformation defining the initial angle of the end effector wrt the table
        yarp::sig::Matrix impactAngle = yarp::math::euler2dcm(yarp::sig::Vector{0, -M_PI / 12, 0});

        yarp::sig::Vector o0 = yarp::math::dcm2axis(R * impactAngle);

        cartControl->setPosePriority("orientation");
        cartControl->goToPoseSync(x0, o0);
        cartControl->waitMotionDone(wait_ping, wait_tmo);

        // open and connect skinEventsPort for reading skin events
        const char *skinEventsPortName = "/skin_events:i";
        skinEventsPort.open(skinEventsPortName);
        if (!skinEventsPort.open(skinEventsPortName)) {
            yError() << "Port could not open";
            return false;
        }

        yarp::os::Network::connect("/left_hand/skinManager/skin_events:o", skinEventsPortName);

        return Thread::start();
    }

    // Synchronous update every getPeriod() seconds
    bool updateModule() {

        //Reading skin events
        iCub::skinDynLib::skinContactList *input = skinEventsPort.read(false);
        if (input) {
            std::cout << input->toString() << std::endl;
        }

        // PUT YOUR CODE HERE

        return true;
    }

    // Asynchronous update that runs in a separate thread as fast as it can
    void run() {

        // PUT YOUR CODE HERE
    }

    double getPeriod() {
        return period;
    }

    bool interruptModule() {
        yInfo() << "Interrupting module ...";
        skinEventsPort.interrupt();
        return RFModule::interruptModule();
    }

    bool close() {
        yInfo() << "Closing the module...";
        skinEventsPort.close();
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
