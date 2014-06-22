#include <openrave-core.h>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <iostream>

using namespace OpenRAVE;
using namespace std;

float time_ms = 0;
int numThreads = 12;
float a=0.3, b=0.3, c= -0.3, d= -0.3, z=0.1;
Transform Ta, Tb;
string robotname = "RV-4F";
std::ofstream traj;
std::ofstream *_traj;
RaveVector< dReal > transA(0.4, 0, 0.2), transB(0, 0.4, 0.3),quatA(0, 1, 0, 0),quatB(0, 0.7071067657322365, 0.707106796640857, 0);
static size_t k_threads = 1;

const float discretization_x = 0.1;
const float discretization_y = 0.1;
const float discretization_z = 0.1;
boost::mutex mtx_ ,mtx__;
static dReal timer = 10;
boost::thread::id completed_thread;

void SetViewer(EnvironmentBasePtr penv, const std::string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AddViewer(viewer);

    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}


bool GetIKSolutions(EnvironmentBasePtr _penv, Transform Pose, std::vector< std::vector< dReal > > &vsolution){

	ModuleBasePtr _pikfast = RaveCreateModule(_penv,"ikfast");
	RobotBasePtr _probot = _penv->GetRobot(robotname);
	_probot->SetActiveManipulator("1"); 
	RobotBase::ManipulatorPtr _pmanip = _probot->GetActiveManipulator(); 
	_penv->Add(_pikfast,true,"");
	std::vector< std::vector< dReal > > solns;

	solns.resize(6);
	stringstream ssin,ssout;
	string iktype = "Transform6D";
	ssin << "LoadIKFastSolver " << _probot->GetName() << " " << iktype;
	if( !_pikfast->SendCommand(ssout,ssin) ) {
		RAVELOG_ERROR("failed to load iksolver\n");
		_penv->Destroy();
		return false;
	}

	if(_pmanip->FindIKSolutions(IkParameterization(Pose),vsolution,IKFO_CheckEnvCollisions) ) {
		stringstream ss; ss << "solution is: ";
			for(size_t i = 0; i < vsolution.size(); ++i) {
				for(size_t j = 0; j < vsolution[i].size(); ++j){
					ss << vsolution[i][j] << " " ;
				} ss << endl;
			}
		ss << endl;
		////RAVELOG_INFO(ss.str());
	 }
	else {
		// could fail due to collisions, etc
		return false;
	}
	return true ;
}

void writeTrajectory(TrajectoryBasePtr ptraj){

	mtx_.lock();
	stringstream ss;
	if (!!(ptraj)){
	
		if ((ptraj->GetDuration() < timer)){
			timer = ptraj->GetDuration();
			//std::cout << "Storing Trajectory with following info" << std::endl;
			//std::cout << timer << std::endl;
			ptraj->serialize(ss);
			traj.open("traj.xml");
			traj.clear();
			traj << ss.str();
			traj.close();
		}
		else {
			//std::cout << "Ignoring Trajectory Files" << std::endl;
		}

	}
	else {
		//std::cout << "Planning Failed : trajectory empty" << std::endl;
	}
	mtx_.unlock();

}

void GetTaskTime(EnvironmentBasePtr _penv, std::vector<dReal> &vsolutionA, std::vector<dReal> &vsolutionB, TrajectoryBasePtr ptraj){

	//thread::id get_id();
	std::cout << "this thread ::" << boost::this_thread::get_id() << std::endl;
	PlannerBasePtr planner = RaveCreatePlanner(_penv,"birrt");
	//std::vector<dReal> vinitialconfig,vgoalconfig;
	ptraj = RaveCreateTrajectory(_penv,"");
	PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
	RobotBasePtr probot = _penv->GetRobot(robotname);
	params->_nMaxIterations = 4000; // max iterations before failure

	GraphHandlePtr pgraph;
	{
		EnvironmentMutex::scoped_lock lock(_penv->GetMutex()); // lock environment
		params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs

		//initial config
		probot->SetActiveDOFValues(vsolutionA);
		params->vinitialconfig.resize(probot->GetActiveDOF());
		probot->GetActiveDOFValues(params->vinitialconfig);

		//goal config	
		probot->SetActiveDOFValues(vsolutionB);
		params->vgoalconfig.resize(probot->GetActiveDOF());
		probot->GetActiveDOFValues(params->vgoalconfig);

		//setting limits
		vector<dReal> vlower,vupper;
		probot->GetActiveDOFLimits(vlower,vupper);

		//RAVELOG_INFO("starting to plan\n");
		
		if( !planner->InitPlan(probot,params) ) {
			//return ptraj;
		}
		// create a new output trajectory
		
		if( !planner->PlanPath(ptraj) ) {
			RAVELOG_WARN("plan failed \n");
			//return NULL;
			
		}

		
	}
	
	
	////RAVELOG_INFO(ss.str());
	if (!! ptraj){
		//std::cout << ptraj->GetDuration() << std::endl;	
		writeTrajectory(ptraj);
	}
	std::cout << "this thread ::" << boost::this_thread::get_id() << " has completed its work" << std::endl;
	mtx__.lock();
	completed_thread = boost::this_thread::get_id();
	mtx__.unlock();
	//return  ptraj;


}

void do_task(Transform Ta, Transform Tb, EnvironmentBasePtr _penv, unsigned int thread_count_left)
{
	std::vector< std::vector< dReal > > vsolutionsA, vsolutionsB;
	dReal time_seconds;
	
	TrajectoryBasePtr ptraj;
	EnvironmentBasePtr pEnv = _penv->CloneSelf(Clone_Bodies);
	
	if (!!(GetIKSolutions(_penv, Tb, vsolutionsB) && GetIKSolutions(_penv, Ta,vsolutionsA))){
		//RAVELOG_INFO("Solution found for both source and goal\n");
		
	} 
	else {

		////RAVELOG_INFO("No solution found either for source or goal");
	}
	k_threads = 1;
	unsigned int _combinations = vsolutionsA.size() * vsolutionsB.size() , combinations = 0;
	std::cout << "Found " << _combinations << " combinations" << std::endl;
	if (!! _combinations){
	vector<boost::shared_ptr<boost::thread> > vthreads(_combinations);
	for(size_t i = 0; i < vsolutionsA.size(); ++i) {
		//RAVELOG_INFO("Planning for every Pose A \n");
			for(size_t j = 0; j < vsolutionsB.size(); ++j) {
				//RAVELOG_INFO("to every Pose B \n");
				std::cout << "i " << i << "j " <<j << std::endl;
				combinations +=1;
				//check if all threads are already running
				 if (k_threads == thread_count_left || combinations == _combinations){
					 for(size_t k = 0; k < k_threads-1; ++k) {
					        std::cout << "Retrieved thread to be complete " << completed_thread << std::endl;
						vthreads[k]->join();
						//k_threads -= 1;
						RAVELOG_INFO("joined ........................\n");
					  }


					 /*for(size_t k = 0; k < k_threads-1; ++k) {
					        std::cout << "Retrieved thread to be complete " << completed_thread << std::endl;
						if (vthreads[k]->get_id() == completed_thread){
								vthreads[k]->join();
								k_threads -= 1;
								RAVELOG_INFO("joined \n");

						}
						
					  }*/
				 }
	
				 else if(k_threads <= thread_count_left){
					//std::cout << k_threads-1 << std::endl;
					
				 	vthreads[k_threads-1].reset(new boost::thread(boost::bind(&GetTaskTime, _penv, vsolutionsA[i], vsolutionsB[j],ptraj)));
					//vthreads[k_threads].get_id();
					k_threads += 1;
					continue;
					
				 }
				  
			          //RAVELOG_INFO("wait for threads to finish\n");
				  //std::cout << vthreads.size() << std::endl;
				  //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
				 
				 
				   
				   ////RAVELOG_INFO("threads finished\n");
				   //vthreads.resize(0);
			}
	}
	
 	
	
	}

}



void track_threads(){
	
	//std::cout << "THREADS LEFT" << (numThreads-k_threads +1) << std::endl;	
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		
	
}

int main()
{
   
    traj.clear();
    unsigned int mainthreadsleft = numThreads;
    
   // boost::shared_ptr<boost::thread> ( new boost::thread(boost::bind( &track_threads )));
    Ta.trans = transA;
    Ta.rot = quatA;
    Tb.trans = transB;
    Tb.rot = quatB;
    std::string scenefilename = "scenes/test6dof.mujin.zae";
    std::string viewername = "qtcoin";
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    Transform robot_t;
    RaveVector< dReal > transR(c, d, 0);
    robot_t.trans = transR;

    //boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
    penv->Load(scenefilename);
    RobotBasePtr probot = penv->GetRobot("RV-4F");

    //removing floor for collision checking
    EnvironmentBasePtr pclondedenv = penv->CloneSelf(Clone_Bodies);
    pclondedenv->Remove( pclondedenv->GetKinBody("floor"));
    RobotBasePtr probot_clone = pclondedenv->GetRobot("RV-4F");

    unsigned int tot = ((( abs(a) + abs(c) )/discretization_x )+1) * (((( abs(b) + abs(d) )/discretization_y )+1) * (( abs( z )/discretization_z )+1));
    unsigned int tot_o = tot;
   
    for (unsigned int i = 0 ; i <= (( abs(a) + abs(c) )/discretization_x );i++) {
	for (unsigned int j = 0 ; j <= (( abs(b) + abs(d) )/discretization_y ); j++) {
		for (unsigned int k = 0 ; k <= ( abs( z )/discretization_z ) ; k++) {
			////std::cout << transR[0] << ";" << transR[1] << ";" << transR[2] << std::endl;
			
			//boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			robot_t.trans = transR;
			probot->SetTransform(robot_t);
                        probot_clone->SetTransform(robot_t);
			if( pclondedenv->CheckCollision(RobotBaseConstPtr(probot_clone)) ){
				//std::cout << "Robot in collision with the environment" << std::endl;
			}
			else {	
				
				do_task(Ta, Tb, penv,3);
			}
			tot -= 1;
			std::cout << tot << "/" << tot_o << std::endl;
			transR[2] = transR[2]+ discretization_z;
		}
		transR[2] = 0;
		transR[1] = transR[1] + discretization_y;
		robot_t.trans = transR;
	}
	transR[2] = 0;
	transR[1] = c;
	transR[0] = transR[0] + discretization_x;
	robot_t.trans = transR;
    }

 
    //thviewer.join(); // wait for the viewer thread to exit	
    RaveDestroy(); // make sure to destroy the OpenRAVE runtime
    penv->Destroy(); // destroy
    return 0;
}
