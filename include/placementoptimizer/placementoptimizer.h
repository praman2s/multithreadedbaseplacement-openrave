// -*- coding: utf-8 -*-
// Praveen Ramanujam <praveen.ramanujam@gmail.com>
//
// This file is part of multithreaded base placement optimizatin function.
// Includes common includes and inline functions for placementoptimizer.

#ifndef _PLACEMENTOPTIMIZER_H
#define _PLACEMENTOPTIMIZER_H

#include "common.h"


/////////////////////////////////////////////////////////////////////////////////
/// Data Storage for PlacementOptimizerBase . Contains Default Values
/////////////////////////////////////////////////////////////////////////////////

class PlacementOptimizerData {

public:
    /// Constructor with default values for initiailization
    PlacementOptimizerData(){
        numThreads = 675;
        points[0] = 0.3;
        points[1] = 0.3;
        points[2] = -0.3;
        points[3] = -0.3;
        z = 0.3;
        robotname = "RV-4F";
        manipname = "1";
        ignorebody = "floor";
        OpenRAVE::RaveVector< OpenRAVE::dReal > transA(0.4, 0, 0.2), transB(0, 0.4, 0.3),quatA(0, 1, 0, 0),quatB(0, 0.7071067657322365, 0.707106796640857, 0);
        Ta.trans = transA;
        Ta.rot = quatA;
        Tb.trans = transB;
        Tb.rot = quatB;
        discretization_x = 0.05;
        discretization_y = 0.05;
        discretization_z = 0.1;

    };

    /// Default destructor
    virtual ~PlacementOptimizerData(){
    };
    void writeData(TrajectoryBasePtr ptraj, double time);
    
    

    /// \brief Performs the actual planning on the cloned environment.
    ///        No loop should be performed. Since multithreaded, each time this function is called, environment
    ///        will be cloned internally to the member function
    /// \param Environment and vector of solutions for both Pose A and B
    /// \return If called after optimizebase, returns the optimized time
    void GetTrajectoryTime(EnvironmentBasePtr _penv, boost::shared_ptr<PlacementOptimizerData> _data, std::vector<dReal> &vsolutionA, std::vector<dReal> &vsolutionB, TrajectoryBasePtr &ptraj, double  &time);

    /// \brief Performs if the robot collides with any object in the internally cloned environment.
    /// \param EnvironmentBasePtr Environment that needs to be checked
    /// \param string Bodies like floor can be ignored
    /// \return bool returns true if no collision occured
    void CheckNoCollisions(EnvironmentBasePtr env, string ignoreBody, bool &collision);
	
    /// number of threads
    unsigned int numThreads;
    /// Boubdaries in 3D. Imagine robot in the center and the boundaries in (x,y,z)
    OpenRAVE::dReal points[4],z;
    /// Optimization from Ta in configuration space to Tb in configuration space
    OpenRAVE::Transform Ta, Tb;
    /// Name of the robot under which optimization needs to be performed
    std::string robotname;
    /// Manipulator name for which the value needs to be set ["1"]
    std::string manipname;
    /// Body to be ignored. Default is set to floor
    std::string ignorebody;
    /// Discretization for x axis default = 0.1
    float discretization_x;
    /// Discretization for y axis default = 0.1
    float discretization_y;
    /// Discretization for z axis default = 0.1
    float discretization_z;
    /// All robot poses where robot is not suffering from collision
    std::vector < Transform > _allPoses;

private:
    /// mutex for shared resource
    boost::mutex __mutex, _posemutex;
    /// internal memeber function to write pose data
    void writePoseData(Transform T) const;
	
};



/////////////////////////////////////////////////////////////////////////////////
/// Optimizes the position of the base to move manipulator from A to B w.r.t time
/////////////////////////////////////////////////////////////////////////////////

class PlacementOptimizerBase {

public:
    virtual ~PlacementOptimizerBase(){
    };
    virtual bool OptimizeBase() = 0;
    virtual double GetOptimizedTime () const = 0;
    virtual Transform GetOptimizedPose () const = 0;


};


/////////////////////////////////////////////////////////////////////////////////
/// Concrete Implementation fo PlacementOptimizerBase class
/////////////////////////////////////////////////////////////////////////////////
class DiscretizedPlacementOptimizer : public PlacementOptimizerBase {

public:
    DiscretizedPlacementOptimizer(EnvironmentBasePtr penv,boost::shared_ptr<PlacementOptimizerData> data);
    virtual ~DiscretizedPlacementOptimizer();

    /// \brief The main function which needs to be called for base optimization
    ///
    /// \return In case of clashes, will output text and return false
    bool OptimizeBase();

    /// \brief Function returns the optimized time w.r.t time
    ///
    /// \return If called after optimizebase, returns the optimized time
    double GetOptimizedTime () const;

    /// \brief Function returns the optimized time w.r.t pose
    ///
    /// \return If called after optimizebase, returns the optimized pose
    Transform GetOptimizedPose () const;

private:


    /// \brief Performs the actual planning on the cloned environment.
    ///        No loop should be performed. Since multithreaded, each time this function is called, environment
    ///        will be cloned internally to the member function
    /// \param EnvironmentBasePtr and vector of solutions for both Pose A and B
    /// \return If called after optimizebase, returns the optimized time
    bool GetIKSolutions(EnvironmentBasePtr _penv, Transform Pose, std::vector< std::vector< dReal > > &vsolution);

    

    /// \brief Multithreaded Trajectory generation from Pt A to Pt B with differnt IK solutions.
    /// \param EnvironmentBasePtr Environment that needs to be checked
    /// \param std::vector< std::vector< dReal > > Pt A solutions
    /// \param std::vector< std::vector< dReal > > Pt B solutions
    /// \return bool returns true if mulithreading is complete i.e all threads are joined
    bool MulithreadedPlanning(EnvironmentBasePtr env, std::vector< std::vector< dReal > > vsolutionsA, std::vector< std::vector< dReal > > vsolutionsB);

    /// \brief Multithreaded Trajectory generation from Pt A to Pt B with differnt IK solutions.
    /// \param EnvironmentBasePtr environment under which planning loop is performed
    /// \return bool returns true if planning loop is performed
    bool PlanningLoop (EnvironmentBasePtr env);

    /// Cloned Environment
    EnvironmentBasePtr _penv;

    /// Robot from cloned environment
    RobotBasePtr _probot;

    /// User set optimizer data
    boost::shared_ptr<PlacementOptimizerData> _data;

    /// output robot pose. stored as result
    Transform _robotPose;

    // trajectory in seconds
    double _timemilliseconds;

    // mutable data
    void writeToResource(TrajectoryBasePtr ptraj);

    // writable trajectory
    std::ofstream traj;

    // threads left
    size_t k_threads;
    /// Currently supports only 6D IK
    string _iktype;
    boost::mutex _mutex;


};



#endif
