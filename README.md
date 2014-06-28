multithreadedbaseplacement-openrave
==================

Bug fixes after delivery

Tested in :

	      Ubuntu 12.04 ( gcc 4.5.3 )
	      Boost 1.43.0
	      OpenRAVE 0.9 (master) -- shows error with latest_stable
         
              	      

Directly want to play the optimized base pose and trajectory , execute :

              python test.py


Compiling the code :

              git clone https://github.com/crazy-robot/multithreadedbaseplacement-openrave
              cd multithreadedbaseplacement-openrave
              make
              
To run the optimization :

              ./build/testfunction --scene=scenes/testscene.env.xml --robot=RV-4F --manip=1 --threads=4
              
              
Results :
    
              progress and results are displayed
              trajectory result are stored in traj.xml in the same folder


Code documentation (gh-pages) :

               http://crazy-robot.github.io/multithreadedbaseplacement-openrave/
               
