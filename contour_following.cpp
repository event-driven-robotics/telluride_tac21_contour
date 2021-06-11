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

class ContourFollowingModule : public RFModule, public Thread {

private:
    BufferedPort<iCub::skinDynLib::skinContactList> skinEventsPort;
    // Cartesian control interfaces
    PolyDriver cartDriver;
    ICartesianControl *cartControl;
    IControlMode *cartControlMode;

    // Position control interfaces
    PolyDriver drvTorso;
    PolyDriver drvRightArm;
    PolyDriver drvLeftArm;
    PolyDriver drvHead;
    IPositionControl *posControlRightArm;
    IPositionControl *posControlLeftArm;
    IPositionControl *posControlHead;
    IPositionControl *posControlTorso;
    IControlMode *controlModeRightArm;
    IControlMode *controlModeLeftArm;
    IControlMode *controlModeHead;
    IControlMode *controlModeTorso;
    IEncoders *iencs;
    RpcServer rpcServer;

    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};
    std::string robot, arm;
    int startup_context_id_arm;
    double period;

    static auto helperWaitDevice(PolyDriver &driver,
                          const Property &options,
                          const std::string &device_name) {
        const auto t0 = Time::now();
        while (Time::now() - t0 < 10.) {
            if (driver.open(const_cast<Property &>(options))) {
                return true;
            }
            Time::delay(1.);
        }

        yError() << "Unable to open the Device Driver:" << device_name;
        return false;
    }

public:

    void closeHand(IPositionControl *posControl) {
        posControl->setRefSpeeds(std::vector<double>(15, 30).data());
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
        arm = rf.check("arm", Value("right_arm")).asString();
        period = rf.check("period", Value(0.01)).asDouble();
    }

    bool openDrivers() {
        // Setting up device for moving the arm in cartesian space
        Property options_arm;
        options_arm.put("device", "cartesiancontrollerclient");
        options_arm.put("remote", "/" + robot + "/cartesianController/" + arm);
        options_arm.put("local", getName() + "/" + arm);
        if (!helperWaitDevice(cartDriver, options_arm, "Cartesian Controller")) {
            return false;
        }

        cartDriver.view(cartControl);

        Property optionsRightArm;
        optionsRightArm.put("device", "remote_controlboard");
        optionsRightArm.put("local", getName() + "/controlboard/right_arm");
        optionsRightArm.put("remote", "/" + robot + "/right_arm");
        if (drvRightArm.open(optionsRightArm)) {
            if (!drvRightArm.view(posControlRightArm) || !drvRightArm.view(controlModeRightArm)) {
                std::cout << "Could not view driver" << std::endl;
                return false;
            }

        } else {
            std::cout << "Could not open remote controlboard" << std::endl;
            return false;
        }

        Property optionsLeftArm;
        optionsLeftArm.put("device", "remote_controlboard");
        optionsLeftArm.put("local", getName() + "/controlboard/left_arm");
        optionsLeftArm.put("remote", "/" + robot + "/left_arm");
        if (drvLeftArm.open(optionsLeftArm)) {
            if (!drvLeftArm.view(posControlLeftArm) || !drvLeftArm.view(controlModeLeftArm)) {
                std::cout << "Could not view driver" << std::endl;
                return false;
            }

        } else {
            std::cout << "Could not open remote controlboard" << std::endl;
            return false;
        }

        Property optionsHead;
        optionsHead.put("device", "remote_controlboard");
        optionsHead.put("local", getName() + "/controlboard/head");
        optionsHead.put("remote", "/" + robot + "/head");
        if (drvHead.open(optionsHead)) {
            if (!drvHead.view(posControlHead) || !drvHead.view(controlModeHead)) {
                std::cout << "Could not view driver" << std::endl;
                return false;
            }

        } else {
            std::cout << "Could not open remote controlboard" << std::endl;
            return false;
        }

        Property optionsTorso;
        optionsTorso.put("device", "remote_controlboard");
        optionsTorso.put("local", getName() + "/controlboard/torso");
        optionsTorso.put("remote", "/" + robot + "/torso");
        if (drvTorso.open(optionsTorso)) {
            if (!drvTorso.view(posControlTorso) || !drvTorso.view(controlModeTorso)) {
                std::cout << "Could not view driver" << std::endl;
                return false;
            }

        } else {
            std::cout << "Could not open remote controlboard" << std::endl;
            return false;
        }

        if (arm == "right_arm") {
            drvRightArm.view(iencs);
            drvRightArm.view(cartControlMode);
        } else if (arm == "left_arm") {
            drvLeftArm.view(iencs);
            drvLeftArm.view(cartControlMode);
        } else {
            std::cout << "Sorry i only have a left and a right arm" << std::endl;
            return false;
        }
        return true;
    }

    bool moveEndEffectorToFingertip() {

        int nEncs;
        iencs->getAxes(&nEncs);
        Vector encs(nEncs);
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
        Vector tip_o = dcm2axis(tipFrame);
        cartControl->attachTipFrame(tip_x, tip_o); // establish the new controlled frame
        return true;
    }

    void home() {
        controlModeRightArm->setControlModes(std::vector<int>(15, VOCAB_CM_POSITION).data());
        controlModeLeftArm->setControlModes(std::vector<int>(15, VOCAB_CM_POSITION).data());
        controlModeHead->setControlModes(std::vector<int>(6, VOCAB_CM_POSITION).data());
        controlModeTorso->setControlModes(std::vector<int>(3, VOCAB_CM_POSITION).data());
        posControlRightArm->setRefSpeeds(std::vector<double>(15, 30).data());
        posControlLeftArm->setRefSpeeds(std::vector<double>(15, 30).data());
        posControlHead->setRefSpeeds(std::vector<double>(6, 30).data());
        posControlTorso->setRefSpeeds(std::vector<double>(3, 15).data());
        posControlRightArm->positionMove(std::vector<double>{-30, 30, 0, 45, 0, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0}.data());
        posControlLeftArm->positionMove(std::vector<double>{-30, 30, 0, 45, 0, 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0}.data());
        posControlHead->positionMove(std::vector<double>{0, 0, 0, 0, 0, 0}.data());
        posControlTorso->positionMove(std::vector<double>{0, 0, 0}.data());
        bool done = false;
        double now = Time::now();
        while (!done && Time::now() - now < wait_tmo) {
            done = true;
            bool check;
            posControlLeftArm->checkMotionDone(&check);
            done &= check;
            posControlRightArm->checkMotionDone(&check);
            done &= check;
            posControlHead->checkMotionDone(&check);
            done &= check;
            posControlTorso->checkMotionDone(&check);
            done &= check;
            sleep(1);
        }
    }

    bool configure(ResourceFinder &rf) {

        parseParams(rf);
        if (!openDrivers()) return false;
        if (!moveEndEffectorToFingertip()) return false;

        // Set reference speed for all arm joints
        home();
        if (arm == "right_arm") {
            controlModeRightArm->setControlModes(std::vector<int>(15, VOCAB_CM_POSITION).data());
            closeHand(posControlRightArm);
        } else {
            controlModeLeftArm->setControlModes(std::vector<int>(15, VOCAB_CM_POSITION).data());
            closeHand(posControlLeftArm);
        }

        cartControlMode->setControlModes(std::vector<int>(15, VOCAB_CM_POSITION).data());
        cartControl->setPosePriority("orientation");
        cartControl->storeContext(&startup_context_id_arm);    //Storing initial context for restoring it later
        cartControl->setTrajTime(.6);               // Each trajectory would take this much amount of time to perform

        // Enabling all degrees of freedom
        Vector dof;
        cartControl->getDOF(dof);
        dof = 1.;
        cartControl->setDOF(dof, dof);

        //--- Moving to starting pose ---//
        Vector x0{-0.4, 0.1, 0.1}; // Starting end effector position

        //Rotation from root frame to end effector pointing straight ahead
        Matrix R = zeros(3, 3);
        R(0, 0) = -1.;
        R(1, 1) = 1.;
        R(2, 2) = -1.;

        // Transformation defining the initial angle of the end effector wrt the table
        Matrix impactAngle = euler2dcm(Vector{0, -M_PI / 12, 0});

        Vector o0 = dcm2axis(R * impactAngle);

        cartControl->goToPoseSync(x0, o0);
        cartControl->waitMotionDone(wait_ping, wait_tmo);

        // open and connect skinEventsPort for reading skin events
        const char *skinEventsPortName = "/skin_events:i";
        skinEventsPort.open(skinEventsPortName);
        if (!skinEventsPort.open(skinEventsPortName)) {
            yError() << "Port could not open";
            return false;
        }
        if (!rpcServer.open(getName("/rpc"))) {
            yError() << "Could not open rpc port";
            return false;
        }

        attach(rpcServer);
        Network::connect("/left_hand/skinManager/skin_events:o", skinEventsPortName);

        return Thread::start();
    }

    bool respond(const Bottle &command, Bottle &reply) {
        if (command.get(0).asString() == "home") {
            reply.addString("Going back home");
            home();
        } else {
            reply.addString("Command not recognized. Available commands: home");
        }

        return true;
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
        cartDriver.close();

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
