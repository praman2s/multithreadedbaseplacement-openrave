#!/usr/bin/python

from openravepy import *

def main():
    env = Environment()
    env.Load('scenes/testscene.env.xml')
    env.SetViewer('qtcoin')
    robot = env.GetRobots()[0]
    T = robot.GetTransform()
    T[1,3] = -0.1
    robot.SetTransform(T)
    traj = RaveCreateTrajectory(env,'').deserialize(open('traj.xml','r').read())
    raw_input("Press any key to execute trajectory") 
    robot.GetController().SetPath(traj)
    raw_input("Press any key to exit") 



if __name__ == "__main__":
    main()
