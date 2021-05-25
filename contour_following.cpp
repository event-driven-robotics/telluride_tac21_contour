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

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class ContourFollowingModule : public RFModule, public yarp::os::Thread
{

private:
    BufferedPort<iCub::skinDynLib::skinContactList> port;
    PolyDriver drv_arm;
    ICartesianControl *arm;

    yarp::sig::Matrix R;
    yarp::sig::Vector x0{-.25, .0, -.05}, o0; //TODO tune starting position
    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};
    std::string robot, which_arm;
    int startup_context_id_arm;
    double period;

    auto helperWaitDevice(yarp::dev::PolyDriver &driver,
                          const yarp::os::Property &options,
                          const std::string &device_name)
    {
        const auto t0 = yarp::os::Time::now();
        while (yarp::os::Time::now() - t0 < 10.)
        {
            if (driver.open(const_cast<yarp::os::Property &>(options)))
            {
                return true;
            }
            yarp::os::Time::delay(1.);
        }

        yError() << "Unable to open the Device Driver:" << device_name;
        return false;
    }

public:
    bool configure(ResourceFinder &rf)
    {
        std::cout << "SIMON YOU ARE BEAUTIFUL!" << std::endl;

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
        if (!helperWaitDevice(drv_arm, options_arm, "Cartesian Controller"))
        {
            return false;
        }
        drv_arm.view(arm);

        arm->storeContext(&startup_context_id_arm); //Storing initial context for restoring it later
        arm->setTrajTime(.6);                       // Each trajectory would take this much amount of time to perform

        // Enabling all degrees of freedom
        yarp::sig::Vector dof;
        arm->getDOF(dof);
        dof = 1.;
        arm->setDOF(dof, dof);

        // Moving to starting position
        R = yarp::math::zeros(3, 3);

        R(0, 0) = -1.;
        R(2, 1) = -1.;
        R(1, 2) = -1.;
        o0 = yarp::math::dcm2axis(R);

        arm->goToPoseSync(x0, o0);
        arm->waitMotionDone(wait_ping, wait_tmo);
        arm->setPosePriority("position");

        // open and connect port for reading skin events
        port.open("/skin_events");
        if (!port.open("/skin_events"))
        {
            yError() << "Port could not open";
            return false;
        }
        yarp::os::Network::connect("/left_hand/skinManager/skin_events:o", "/skin_events");

        return Thread::start();
    }

    // Synchronous update every getPeriod() seconds
    bool updateModule()
    {

        //Reading skin events
        iCub::skinDynLib::skinContactList *input = port.read(true);
        if (input)
        {
            std::cout << input->toString() << std::endl;
        }

        // PUT YOUR CODE HERE

        return true;
    }

    // Asynchronous update that runs in a separate thread as fast as it can
    void run()
    {
        //TODO add example movement
        // PUT YOUR CODE HERE
    }

    double getPeriod()
    {
        return period;
    }

    bool interruptModule()
    {
        yInfo() << "Interrupting module ...";

        return RFModule::interruptModule();
    }

    bool close()
    {
        yInfo() << "Closing the module...";

        arm->stopControl();
        arm->restoreContext(startup_context_id_arm);
        drv_arm.close();

        yInfo() << "...done!";
        return RFModule::close();
    }
};

int main(int argc, char *argv[])
{
    // Connecting to the yarp server (must be running already)
    Network yarp;
    if (!yarp.checkNetwork(2.0))
    {
        yError() << "Network not found";
        return -1;
    }

    // Configuring and running the module
    ContourFollowingModule jCtrl;
    ResourceFinder rf;
    rf.configure(argc, argv);

    return jCtrl.runModule(rf);
}
