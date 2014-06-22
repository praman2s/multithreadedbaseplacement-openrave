#include <openrave-core.h>
#include "placementoptimizer/common.h"
#include "placementoptimizer/placementoptimizer.h"
#include <boost/program_options.hpp>
#include <boost/timer.hpp>

using namespace std;
using namespace OpenRAVE;
namespace po = boost::program_options;

string scene, robotname, manipname;

int processCommandLineParameters(int argc, char *argv[], EnvironmentBasePtr & env, boost::shared_ptr<PlacementOptimizerData> &data){
    RobotBasePtr probot;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
	("scene", po::value<string>(), "store scene file")
        ("robot", po::value<string>(), "Robot Name")
	("manip", po::value<string>(), "Manipulator Name")
        ("threads", po::value<unsigned int>(), "Number of threads")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);  

    if (vm.count("help")) {
       cout << "executable " << " " << " --scene=scene file " << " --robot=robotname "
	    << "--manip = manipulatorname \n";
       return 1;
    }  
    if(argc !=5){
	cout << "Command line options are \n" << "executable " << " " << " --scene=scene file " << " --robot=robotname "
	    << "--manip=manipulatorname "<< "--threads=number \n";
       return 1;
    }
    if (vm.count("scene")) {
	scene =  vm["scene"].as<string>();
	if (!env->Load( scene ))
		RAVELOG_ERROR("Please provide a correct scene file\n");
    }
    if (vm.count("threads")) {
	data->numThreads =  vm["threads"].as<unsigned int>();
    }
    if (vm.count("robot")) {
	robotname =  vm["robot"].as<string>();
	data->robotname = robotname; 
	RobotBasePtr probot = env->GetRobot(robotname);
	if(!probot){
			RAVELOG_ERROR("Incorrect robot name\n");
			return 1;
	}
	else {
		if (vm.count("manip")) {
			manipname =  vm["manip"].as<string>();
			probot->SetActiveManipulator(manipname);
			RobotBase::ManipulatorPtr _pmanip = probot->GetActiveManipulator();
			if (!!_pmanip){
				data->manipname = manipname;
			}
			else {
				RAVELOG_ERROR("Incorrect manipulator name\n");
				return 1;
			}
    		}

	}
    }
    
    return 0;
}




int main(int argc, char *argv[])
{
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    // loading optimizer data for placement optimizer
    boost::shared_ptr<PlacementOptimizerData> defaultOptimizerData(new PlacementOptimizerData());
    if (!processCommandLineParameters( argc, argv, penv, defaultOptimizerData)){  //process command line parameters
	    double _time;    // to store the time
	    //initiate the placement optimizer
	    boost::shared_ptr<PlacementOptimizerBase> optimizer( new DiscretizedPlacementOptimizer(penv,defaultOptimizerData ));
	    if (!!optimizer->OptimizeBase()) {
		_time = optimizer->GetOptimizedTime();
		Transform t = optimizer->GetOptimizedPose();
		std::cout << "Optimized Base Pose is : " << t << std::endl;
		RAVELOG_INFO("The trajectory duration is %f . The trajectory file is saved as traj.xml\n",_time);

	    } // should be called only after optimizebase
    }
    RaveDestroy();
    return 0;
}
