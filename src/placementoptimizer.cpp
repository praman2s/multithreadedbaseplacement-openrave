// -*- coding: utf-8 -*-
// Praveen Ramanujam <pramanujam86@gmail.com>
//
// This file is part of multithreaded base placement optimizatin function.
// Implementation for the placement optimizer.


#include <placementoptimizer/placementoptimizer.h>
#include <boost/thread/mutex.hpp>

void DiscretizedPlacementOptimizer :: writePoseData(Transform T, std::vector< std::vector< dReal > > solnsA, std::vector< std::vector< dReal > > solnsB) {
        struct PoseMap Map;
	Map.t = T;
	Map.solnsA = solnsA;
	Map.solnsB = solnsB;
	//boost::lock<boost::mutex> scoped_lock(_posemutex);
	if ( !!_posemutex.try_lock()){
		
		_data->_ikPoses.push_back(Map);
		_posemutex.unlock();
	}
	else{
		//tested. condition never occurs
	     std::cout << boost::this_thread::get_id() << " did not get mutex" << std::endl;
	}

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
			//return ptraj;
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
   //_threads(_data->numThreads);
    _probot = _penv->GetRobot(_data->robotname);
    

}

DiscretizedPlacementOptimizer :: ~DiscretizedPlacementOptimizer(){


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


bool DiscretizedPlacementOptimizer :: OptimizeBase(){

    unsigned int cnt = 0, thread_cnt = _data->numThreads,  _thread_cnt = 0; //local count for threads
    std::vector< EnvironmentBasePtr > pclondedenv ( thread_cnt );
    RobotBasePtr probot_clone;
    UpdateGrid();
    RobotBasePtr robot;
   
    vector<boost::shared_ptr<boost::thread> > _threadscollision(thread_cnt);

    for (unsigned int i = 0; i < gridMap.size() ; ){
	
	if(cnt < thread_cnt){
		pclondedenv[cnt] = _penv->CloneSelf(Clone_Bodies); 
		probot_clone = pclondedenv[cnt]->GetRobot(_data->robotname);
		probot_clone->SetTransform(gridMap[i]);
		i = i+1;
		//_threadscollision[cnt].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer:: CheckNoCollisions,_data, pclondedenv[cnt] , _data->ignorebody))); 
		 _threadscollision[cnt].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: CheckNoCollisions, this, pclondedenv[cnt])));
		RAVELOG_INFO("%d / %d \n",i,gridMap.size());
		cnt++;
		_thread_cnt++;
		
	}
	else{
		//polling rather than to wait
		for (unsigned int m=0; m < cnt; m++) {
			//if(_threadscollision[m]->joinable()){
		            _threadscollision[m]->join();
			    pclondedenv[m]->Destroy();
			    //break;
			//}
			     
        	}
		
		cnt = 0;
		
	  

	}
	
      }
      for (unsigned int m=0; m < cnt; m++) {
		_threadscollision[m]->join();
        }
    
    RAVELOG_INFO("Found %d base poses with no collision and corresponding IK solutions\n", _data->_ikPoses.size());
    PlanningLoop ();
    return true; // always returns true if all the threads are complete

}




bool DiscretizedPlacementOptimizer :: PlanningLoop (){

    unsigned int cnt = 0, thread_cnt = _data->numThreads,  _thread_cnt = 0; //local count for threads
    vector<boost::shared_ptr<boost::thread> > vthreads( thread_cnt ); //initiate threads for mulithreaded planning
    std::vector< EnvironmentBasePtr > pclondedenv ( thread_cnt ); 
    RobotBasePtr probot_clone;
    Transform robot_t;
    std::vector< std::vector< dReal > > vsolutionA, vsolutionB ;
    TrajectoryBasePtr ptraj;
    
   for (unsigned int i = 0; i < _data->_ikPoses.size(); i++){
	    robot_t  = _data->_ikPoses[i].t;
	    vsolutionA = _data->_ikPoses[i].solnsA;
            vsolutionB = _data->_ikPoses[i].solnsB;
	    std::cout << vsolutionA.size() << "," << vsolutionB.size() << std::endl;
	for (unsigned int j = 0; j < vsolutionA.size(); j++) {
		for (unsigned int k = 0; k < vsolutionB.size(); ) {
			if(cnt < thread_cnt){
				std::cout << j << "," << k << std::endl;
				pclondedenv[cnt] = _penv->CloneSelf(Clone_Bodies); 
				probot_clone = pclondedenv[cnt]->GetRobot(_data->robotname);
				probot_clone->SetTransform(robot_t);
				vthreads[cnt].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: GetTrajectoryTime, this, pclondedenv[cnt], vsolutionA[j], vsolutionB[k], ptraj)));
		                k++;
				cnt++;
				_thread_cnt++;
		
			}
			else{
				//polling rather than to wait
				for (unsigned int m=0; m < cnt; m++) {
					//if( vthreads[m]->joinable()){
					    vthreads[m]->join();
					    pclondedenv[m]->Destroy();
					    //break;
					 //}
				}
		
				cnt = 0;
		
			  

			}
	
		}
	 }
      	for (unsigned int m=0; m < cnt; m++) {
		vthreads[m]->join();
	}


  }
    return true;     // returns true after completion of all threads
}



bool DiscretizedPlacementOptimizer :: MulithreadedPlanning(EnvironmentBasePtr env, std::vector< std::vector< dReal > > vsolutionsA, std::vector< std::vector< dReal > > vsolutionsB, unsigned int subThreads){

    std::vector< EnvironmentBasePtr > pclondedenv ( subThreads );
    
    unsigned int cnt = 0, thread_cnt = subThreads,  _thread_cnt = 0; //local count for threads
    TrajectoryBasePtr ptraj;
    vector<boost::shared_ptr<boost::thread> > vthreads(thread_cnt); //initiate threads for mulithreaded planning
    for (unsigned int i = 0; i < vsolutionsA.size(); i++) {
        for (unsigned int j = 0; j < vsolutionsB.size(); ) {

            if( cnt < thread_cnt ) {
		pclondedenv[cnt] = _penv->CloneSelf(Clone_Bodies); 
                vthreads[cnt].reset(new boost::thread(boost::bind(&DiscretizedPlacementOptimizer :: GetTrajectoryTime, this, pclondedenv[cnt], vsolutionsA[i], vsolutionsB[j], ptraj)));
		j = j+1;
                cnt++;
                _thread_cnt++;
                
            }
            else {
                for (unsigned int k=0; k < cnt; k++) {
                     vthreads[k]->join();
		     pclondedenv[k]->Destroy();

                }
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

    return _timeseconds;

}

Transform DiscretizedPlacementOptimizer :: GetOptimizedPose () const {

    return _robotPose;
}
