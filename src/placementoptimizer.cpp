// -*- coding: utf-8 -*-
// Praveen Ramanujam <pramanujam86@gmail.com>
//
// This file is part of multithreaded base placement optimizatin function.
// Implementation for the placement optimizer.


#include <placementoptimizer/placementoptimizer.h>
#include <boost/thread/mutex.hpp>

void PlacementOptimizerData :: writePoseData(Transform T) {
        
	//boost::lock<boost::mutex> scoped_lock(_posemutex);
	if ( !!_posemutex.try_lock()){
		std::cout << T.trans << std::endl;
		_allPoses.push_back(T);
		//std::cout << boost::this_thread::get_id() << " got mutex" << std::endl;
		_posemutex.unlock();
	}
	else{
		//tested. condition never occurs
	     std::cout << boost::this_thread::get_id() << " did not get mutex" << std::endl;
	}

}

void PlacementOptimizerData::writeData(TrajectoryBasePtr ptraj, double time){

    __mutex.lock();                     //could be replaced by scoped lock
    stringstream ss;
    if (!!(ptraj)) {
        //check if the trajectory is less than the stored trajectory
        if (( ptraj->GetDuration() < time )) {
            time = ptraj->GetDuration();
            ptraj->serialize(ss);
            traj.open("traj.xml");
            traj.clear();
            traj << ss.str();
            traj.close();
        }
        else {
            RAVELOG_INFO( "Ignoring Trajectory Files\n" );
        }

    }
    else {
        RAVELOG_INFO(  "Planning Failed : trajectory empty\n" );
    }
    __mutex.unlock();              //unlock happens automatically in case of scoped lock



}

void PlacementOptimizerData :: CheckNoCollisions(EnvironmentBasePtr pclondedenv, string ignoreBody){

    
    
    //EnvironmentMutex::scoped_lock lock(env->GetMutex());
    pclondedenv->Remove( pclondedenv->GetKinBody(ignoreBody));      // remove floor
    RobotBasePtr probot_clone = pclondedenv->GetRobot(robotname);
    Transform T = probot_clone->GetTransform();
    //std::cout << T.trans << std::endl;
    //check for environment collision
    if ( !pclondedenv->CheckCollision(RobotBaseConstPtr(probot_clone)) ){ 
	writePoseData(probot_clone->GetTransform());
	
    }
    
	
    

}

void PlacementOptimizerData:: GetTrajectoryTime(EnvironmentBasePtr _penv, boost::shared_ptr<PlacementOptimizerData> _data, std::vector<dReal> &vsolutionA, std::vector<dReal> &vsolutionB, TrajectoryBasePtr &ptraj, double  &time){


    //std::cout << "This thread currently in operation" <<  boost::this_thread::get_id() << std::endl;
    //EnvironmentBasePtr _penv = penv->CloneSelf(Clone_Bodies);
    PlannerBasePtr planner = RaveCreatePlanner(_penv,"birrt");
    ptraj = RaveCreateTrajectory(_penv,"");
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    RobotBasePtr probot = _penv->GetRobot(_data->robotname);
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
            //do nothing
        }
        // create a new output trajectory

        if( !planner->PlanPath(ptraj) ) {
            RAVELOG_WARN("plan failed \n");
            // ptraj will be any ways empty

        }

    }

    if (!!ptraj) {
        writeData(ptraj,time);
    }
    //_penv->Destroy();


}


DiscretizedPlacementOptimizer:: DiscretizedPlacementOptimizer
    (EnvironmentBasePtr penv,boost::shared_ptr<PlacementOptimizerData> data) :
    _penv(penv->CloneSelf(Clone_Bodies)), // operates on cloned environment. Does not disturb the main environment
    _data(data){
    _timemilliseconds = 100.0; // higher enough to let the first trajectory store into file
    k_threads = 1;
    _probot = _penv->GetRobot(_data->robotname);
    _iktype = "Transform6D";

}

DiscretizedPlacementOptimizer :: ~DiscretizedPlacementOptimizer(){


}

unsigned int DiscretizedPlacementOptimizer :: UpdateGrid(){

    //Set robot to the extreme end
    Transform robot_t;
   
    RaveVector< dReal > transR(_data->points[2], _data->points[3], 0);
    robot_t.trans = transR;

     for (unsigned int i = 0; i <= (( abs(_data->points[0]) + abs(_data->points[2]) )/_data->discretization_x )+1; ){
        for (unsigned int j = 0; j <=  (( abs(_data->points[1]) + abs(_data->points[3]) )/_data->discretization_y )+1;) {
            for (unsigned int k = 0; k <= (abs(_data->z) /_data->discretization_z)+1;k++) {
		robot_t.trans = transR;
                gridMap.push_back(robot_t); 
		transR[2] = transR[2]+ (_data->discretization_z);

	}
	           transR[2] = 0;
		    transR[1] = transR[1] + _data->discretization_y;
		    robot_t.trans = transR;
		    j++;
        }
	transR[2] = 0;
	transR[1] = _data->points[2];
	transR[0] = transR[0] + _data->discretization_x;
	robot_t.trans = transR;
	i++;
    }
    RAVELOG_INFO("Found %d grids to be evaluated for planning \n", gridMap.size());
    return gridMap.size();


}
bool DiscretizedPlacementOptimizer :: OptimizeBase(){

    unsigned int cnt = 0, thread_cnt = _data->numThreads,  _thread_cnt = 0; //local count for threads
    std::vector< EnvironmentBasePtr > pclondedenv ( thread_cnt );
    RobotBasePtr probot_clone;
    unsigned int gridSize = UpdateGrid();
    RobotBasePtr robot;
   
    vector<boost::shared_ptr<boost::thread> > _threadscollision(thread_cnt);

    for (unsigned int i = 0; i < gridMap.size() ; ){
	
	 //clone body
      
	
	//_data->CheckNoCollisions(_penv,_data->ignorebody);
	if(cnt < thread_cnt){
		pclondedenv[cnt] = _penv->CloneSelf(Clone_Bodies); 
		probot_clone = pclondedenv[cnt]->GetRobot(_data->robotname);
		probot_clone->SetTransform(gridMap[i]);
		i = i+1;
		//boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		_threadscollision[cnt].reset(new boost::thread(boost::bind(&PlacementOptimizerData:: CheckNoCollisions,_data, pclondedenv[cnt] , _data->ignorebody)));
		
		cnt++;
		_thread_cnt++;
		
	}
	else{
		//RAVELOG_INFO("Found %d threads to join \n",cnt);
		for (unsigned int m=0; m < cnt; m++) {
		            _threadscollision[m]->join();
			    pclondedenv[m]->Destroy();
			     
        	}
		//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		 // destroy the clone environment
		cnt = 0;
		
	   //i = i -1;

	}
	
      }
      for (unsigned int m=0; m < cnt; m++) {
		_threadscollision[m]->join();
        }
    
    RAVELOG_INFO("GLOBAL COUNT %d  \n",_thread_cnt);
    return true; // always returns true if all the threads are complete

}




bool DiscretizedPlacementOptimizer :: PlanningLoop (EnvironmentBasePtr env){

    std::vector< std::vector< dReal > > vsolutionsA, vsolutionsB;
    TrajectoryBasePtr ptraj;

    if (!!(GetIKSolutions(env, _data->Tb, vsolutionsB) && GetIKSolutions(env, _data->Ta,vsolutionsA))) {
        MulithreadedPlanning(env, vsolutionsA, vsolutionsB ); // dedicate threads and hold the control over threads

    }
    else {

        //RAVELOG_INFO("No IK solution found either for source or goal");
    }
    return true;     // returns true after completion of all threads
}

bool DiscretizedPlacementOptimizer :: MulithreadedPlanning(EnvironmentBasePtr env, std::vector< std::vector< dReal > > vsolutionsA, std::vector< std::vector< dReal > > vsolutionsB){

    unsigned int cnt = 0, thread_cnt = _data->numThreads,  _thread_cnt = 0; //local count for threads
    TrajectoryBasePtr ptraj;
    vector<boost::shared_ptr<boost::thread> > vthreads(thread_cnt); //initiate threads for mulithreaded planning
    for (unsigned int i = 0; i < vsolutionsA.size(); i++) {
        for (unsigned int j = 0; j < vsolutionsB.size(); j++) {

            if( cnt < thread_cnt ) {
                vthreads[cnt].reset(new boost::thread(boost::bind(&PlacementOptimizerData:: GetTrajectoryTime,_data, _penv, _data, vsolutionsA[i], vsolutionsB[j],ptraj, _timemilliseconds)));
                cnt++;
                _thread_cnt++;
                continue;
            }
            else {
                for (unsigned int k=0; k < cnt; k++) {
                    if (!!vthreads[k]->joinable())
                    {
                        _thread_cnt--;
                        vthreads[k]->join();
                        continue;
                    }

                }
                j--;
                cnt = 0;
            }
        }


    }

    for (unsigned int k=0; k < cnt; k++) {
        vthreads[k]->join();
        _thread_cnt--;

    }

    return true;  // returns true only after synchronization



}


double DiscretizedPlacementOptimizer :: GetOptimizedTime () const {

    return _timemilliseconds;


}
Transform DiscretizedPlacementOptimizer :: GetOptimizedPose () const {

    return _robotPose;

}

bool DiscretizedPlacementOptimizer :: GetIKSolutions(EnvironmentBasePtr _penv, Transform Pose, std::vector< std::vector< dReal > > &vsolution){

    ModuleBasePtr _pikfast = RaveCreateModule(_penv,"ikfast");
    RobotBasePtr _probot = _penv->GetRobot(_data->robotname);
    _probot->SetActiveManipulator("1");
    RobotBase::ManipulatorPtr _pmanip = _probot->GetActiveManipulator();
    _penv->Add(_pikfast,true,"");
    std::vector< std::vector< dReal > > solns;

    solns.resize(6);
    stringstream ssin,ssout;

    ssin << "LoadIKFastSolver " << _probot->GetName() << " " << _iktype;
    if( !_pikfast->SendCommand(ssout,ssin) ) {
        RAVELOG_ERROR("failed to load iksolver\n");
        _penv->Destroy();
        return false;
    }

    if(_pmanip->FindIKSolutions(IkParameterization(Pose),vsolution,IKFO_CheckEnvCollisions) ) {

        //continue. the function will return true
    }
    else {

        return false;
    }
    return true;
}




