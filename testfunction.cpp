#include <openrave-core.h>
#include "placementoptimizer/common.h"
#include "placementoptimizer/placementoptimizer.h"
#include <boost/program_options.hpp>
#include <boost/timer.hpp>

using namespace std;
using namespace OpenRAVE;
namespace po = boost::program_options;

int processCommandLineParameters(int argc, char *argv[]){

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
    ;

    return 0;
}




int main(int argc, char *argv[])
{
    boost::timer t;  // to measure optimization time
    double _time;    // to store the time
    // starting the normal OpenRAVE core
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    penv->Load( "scenes/testscene.env.xml" );

    // loading optimizer data for placement optimizer
    boost::shared_ptr<PlacementOptimizerData> defaultOptimizerData(new PlacementOptimizerData());


    //initiate the placement optimizer
    boost::shared_ptr<PlacementOptimizerBase> optimizer( new DiscretizedPlacementOptimizer(penv,defaultOptimizerData ));
    if (!!optimizer->OptimizeBase()) {
        _time = optimizer->GetOptimizedTime();
        RAVELOG_INFO("The optimized time is %f \n",_time);

    } // should be called only after optimizebase

    //resultant trajectory is always stored as traj.xml
    double elapsed = t.elapsed();
    RAVELOG_INFO("%f elapsed to optimize\n",elapsed); //display the elapsed time
    std::cout << defaultOptimizerData->_allPoses.size() << std::endl;
    RaveDestroy();
    return 0;
}
