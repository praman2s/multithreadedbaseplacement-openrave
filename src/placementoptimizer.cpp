// -*- coding: utf-8 -*-
// Praveen Ramanujam <pramanujam86@gmail.com>
//
// This file is part of multithreaded base placement optimizatin function.
// Implementation for the placement optimizer.


#include <placementoptimizer/placementoptimizer.h>


void DiscretizedPlacementOptimizer :: writePoseData(Transform T, std::vector< std::vector< dReal > > solnsA, std::vector< std::vector< dReal > > solnsB) {
	_dataReady = false;        
	struct PoseMap Map;
	Map.t = T;
	Map.solnsA = solnsA;
	Map.solnsB = solnsB;
	{
		boost::unique_lock<boost::mutex> lock(_posemutex);
		_data->_ikPoses.push_back(Map);
		RAVELOG_INFO("Notify a waiting thread \n");
		_dataReady = true;
	}
	
	_poseCondition.notify_one();
	
	
}

void DiscretizedPlacementOptimizer :: writeData(TrajectoryBasePtr ptraj, EnvironmentBasePtr env){

    __mutex.lock();                     //could be replaced by scoped lock
    stringstream ss;
    if (!!(ptraj)) {
        //check if the trajectory is less than the stored trajectory
        if (( ptraj->GetDuration() < _timeseconds )) {
            _timeseconds = ptraj->GetDuration();
            ptraj->serialize(ss);
            traj.open("traj.xml");
            traj.clear();
            traj << ss.str();
            traj.close();
	    _robotPose = env->GetRobot(_data->robotname)->GetTransform();
	    RAVELOG_INFO( "Replacing Trajectory Files\n" );
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

void DiscretizedPlacementOptimizer :: CheckNoCollisions(EnvironmentBasePtr pclondedenv){

    
    std::vector< std::vector< dReal > > vsolutionsA, vsolutionsB;
    //EnvironmentMutex::scoped_lock lock(env->GetMutex());
    pclondedenv->Remove( pclondedenv->GetKinBody(_data->ignorebody));      // remove floor
    RobotBasePtr probot_clone = pclondedenv->GetRobot(_data->robotname);
  
    //check for environment collision
    if ( !pclondedenv->CheckCollision(RobotBaseConstPtr(probot_clone)) ){ 
	if (!!(GetIKSolutions(pclondedenv, _data->Tb, vsolutionsB) && GetIKSolutions(pclondedenv, _data->Ta,vsolutionsA))){

		RAVELOG_INFO("Found solution for both base and goal\n");
		writePoseData(probot_clone->GetTransform(), vsolutionsA, vsolutionsB);
	}
	
    }
    
	
    

}

bool DiscretizedPlacementOptimizer :: GetIKSolutions(EnvironmentBasePtr _penv, Transform Pose, std::vector< std::vector< dReal > > &vsolution){

    ModuleBasePtr _pikfast = RaveCreateModule(_penv,"ikfast");
    RobotBasePtr _probot = _penv->GetRobot(_data->robotname);
    _probot->SetActiveManipulator(_data->manipname);
    RobotBase::ManipulatorPtr _pmanip = _probot->GetActiveManipulator();
    _penv->Add(_pikfast,true,"");
    std::vector< std::vector< dReal > > solns;

    solns.resize(6);
    stringstream ssin,ssout;

    ssin << "LoadIKFastSolver " << _probot->GetName() << " " << _data->_iktype;
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

/// Taken from OpenRAVE example
void DiscretizedPlacementOptimizer:: GetTrajectoryTime(EnvironmentBasePtr env, std::vector<dReal> &vsolutionA, std::vector<dReal> &vsolutionB, TrajectoryBasePtr ptraj){


        //thread::id get_id();
	//std::cout << "this thread ::" << boost::this_thread::get_id() << std::endl;
	PlannerBasePtr planner = RaveCreatePlanner(env,"birrt");
	//std::vector<dReal> vinitialconfig,vgoalconfig;
	ptraj = RaveCreateTrajectory(env,"");
	PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
	RobotBasePtr probot = env->GetRobot(_data->robotname);
	params->_nMaxIterations = 4000; // max iterations before failure

	GraphHandlePtr pgraph;
	{
		EnvironmentMutex::scoped_lock lock(env->GetMutex()); // lock environment
		params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs

		//initial config
		probot->SetActiveDOFValues(vsolutionA);
		params->vinitialconfig.resize(probot->GetActiveDOF());
		probot->GetActiveDOFValues( params->vinitialconfig );
                probot->GetActiveDOFMaxVel( params->_vConfigVelocityLimit );

		//goal config	
		probot->SetActiveDOFValues( vsolutionB );
		params->vgoalconfig.resize( probot->GetActiveDOF() );
		probot->GetActiveDOFValues( params->vgoalconfig );

		//setting limits
		vector<dReal> vlower,vupper;
		probot->GetActiveDOFLimits(params->_vConfigLowerLimit,params->_vConfigUpperLimit);
		

		//RAVELOG_INFO("starting to plan\n");
		
		if( !planner->InitPlan(probot,params) ) {
			return;
		}
		// create a new output trajectory
		
		if( !planner->PlanPath(ptraj) ) {
			RAVELOG_WARN("plan failed \n");
			//return NULL;
			
		}

		
	}
        if (!! ptraj){
		writeData(ptraj,env);
	}

}


DiscretizedPlacementOptimizer:: DiscretizedPlacementOptimizer
    (EnvironmentBasePtr penv,boost::shared_ptr<PlacementOptimizerData> data) :
    _penv(penv->CloneSelf(Clone_Bodies)), // operates on cloned environment. Does not disturb the main environment
    _data(data){
    _timeseconds = 100.0; // higher enough to let the first trajectory store into file
    k_threads = 1;
    _threads.resize(_data->numThreads);
    _probot = _penv->GetRobot(_data->robotname);
    _dataReady = false; 
    _cnt = 0;
    

}

DiscretizedPlacementOptimizer :: ~DiscretizedPlacementOptimizer(){

   //check finally if any threads are active and join. Should not raise this condition.
   for (unsigned int m=0; m < _cnt; m++) {
		 _threads[m]->join();
   }

}

void DiscretizedPlacementOptimizer :: UpdateGrid(){

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
    //return gridMap.size();


}

/// minimum two threads will be created even if user initiates one threads
bool DiscretizedPlacementOptimizer :: OptimizeBase(){
   
    std::vector < boost::shared_ptr < boost::thread > > mainthread(2);
    mainthread[0].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: MultithreadedPlanning,this)));
    mainthread[1].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: PlanningLoop,this)));
    mainthread[0]->join();
    mainthread[1]->join();
    return true; // always returns true if all the threads are complete

}



/// Instantiate a thread waiting for a valid base pose
bool DiscretizedPlacementOptimizer :: PlanningLoop (){
    
    boost::unique_lock<boost::mutex> lock(_posemutex);
	
    while(!_dataReady)
	_poseCondition.wait(lock);	
   
    unsigned int  localcnt = 0, threadCnt =  _data->numThreads,  _threadCnt = 0; //local count for threads
    vector<boost::shared_ptr<boost::thread> >  _threadloop( threadCnt ); //initiate threads for mulithreaded planning
    std::vector< EnvironmentBasePtr > pclondedenv ( threadCnt ); 
    RobotBasePtr probot_clone;
    Transform robot_t;
    std::vector< std::vector< dReal > > vsolutionA, vsolutionB ;
    TrajectoryBasePtr ptraj;
    
   for (unsigned int i = 0; i < _data->_ikPoses.size(); i++){
	    robot_t  = _data->_ikPoses[i].t;
	    vsolutionA = _data->_ikPoses[i].solnsA;
            vsolutionB = _data->_ikPoses[i].solnsB;
	    //std::cout << vsolutionA.size() << "," << vsolutionB.size() << std::endl;
	for (unsigned int j = 0; j < vsolutionA.size(); j++) {
		for (unsigned int k = 0; k < vsolutionB.size(); ) {
			if(localcnt < threadCnt){
				//std::cout << j << "," << k << std::endl;
				pclondedenv[localcnt] = _penv->CloneSelf(Clone_Bodies); 
				probot_clone = pclondedenv[localcnt]->GetRobot(_data->robotname);
				probot_clone->SetTransform(robot_t);
				 _threadloop[localcnt].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: GetTrajectoryTime, this, pclondedenv[localcnt], vsolutionA[j], vsolutionB[k], ptraj)));
		                k++;
				localcnt++;
				_threadCnt++;
		
			}
			else{
				//polling rather than to wait
				for (unsigned int m=0; m < localcnt; m++) {
					//if(  _threads[m]->joinable()){
					     _threadloop[m]->join();
					    pclondedenv[m]->Destroy();
					    //break;
					 //}
				}
		
				localcnt = 0;
		
			  

			}
	
		}
	 }
    // join all the active threads
    for (unsigned int m=0; m < localcnt; m++) {
		 _threadloop[m]->join();
    }


  }
  
    return true;     // returns true after completion of all threads
}



bool DiscretizedPlacementOptimizer :: MultithreadedPlanning(){

    unsigned int  threadCnt = _data->numThreads,  _threadCnt = 0; //local count for threads
    std::vector< EnvironmentBasePtr > pclondedenv ( threadCnt );
    RobotBasePtr probot_clone;
    UpdateGrid();
    RobotBasePtr robot;
    
    //PlanningLoop ();
    //vector<boost::shared_ptr<boost::thread> >  _threads(threadCnt);

    for (unsigned int i = 0; i < gridMap.size() ; ){
	
	if(_cnt < threadCnt){
		pclondedenv[_cnt] = _penv->CloneSelf(Clone_Bodies); 
		probot_clone = pclondedenv[_cnt]->GetRobot(_data->robotname);
		probot_clone->SetTransform(gridMap[i]);
		i = i+1;
		_threads[_cnt].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: CheckNoCollisions, this, pclondedenv[_cnt])));
		// _threads[_cnt]->detach();
		RAVELOG_INFO("%d / %d \n",i,gridMap.size());
		_cnt++;
		_threadCnt++;
		
	}
	else{
		//polling rather than to wait
		for (unsigned int m=0; m < _cnt; m++) {
			//if( _threads[m]->joinable()){
		             _threads[m]->join();
			    pclondedenv[m]->Destroy();
			    //break;
			//}
			     
        	}
		
		_cnt = 0;
		
	  

	}
	
      }
      for (unsigned int m=0; m < _cnt; m++) {
		 _threads[m]->join();
        }
    _cnt = 0;
    
    RAVELOG_VERBOSE("Found %d base poses with no collision and corresponding IK solutions\n", _data->_ikPoses.size());
    return true;  // returns true only after synchronization



}


double DiscretizedPlacementOptimizer :: GetOptimizedTime () const {

    return _timeseconds;

}

Transform DiscretizedPlacementOptimizer :: GetOptimizedPose () const {

    return _robotPose;
}
