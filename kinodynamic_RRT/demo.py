#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy as np
#### YOUR IMPORTS GO HERE ####
import RRT_1
import RRT_2
import RRT_3
import matplotlib.pyplot as plt
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


def ConvertPathToTrajectory(robot,path=[]):
    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    a = np.array([0., 0., 10.])
    planningutils.RetimeAffineTrajectory(traj, maxvelocities=a, maxaccelerations=20*ones(3))
    return traj


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('final.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        #### YOUR CODE HERE ####

    h = []
    h.append(env.drawlinestrip(
        points=array(((-8, 14.5, 0), (-8, 17.5, 0), (-14, 17.5, 0), (-14, 14.5, 0), (-8, 14.5, 0))), linewidth=20.0,
        colors=array((1, 0, 0))))
    h.append(env.drawlinestrip(
        points=array(((1, 8, 0), (1, 24, 0), (-23, 24, 0), (-23, 8, 0), (1, 8, 0))), linewidth=20.0,
        colors=array((1, 1, 0))))

    startconfig=[-29., -29., 0.05]

    print 'This expectation runtime of this program is about 2~5 minutes\n'

    for i in range(3):
        if i == 0:
            print 'test case 1 (least number of motion primitives):\n'
            raw_input('press enter to start\n\n')
            with env:
                path, timeset, totalvelocity, xvset, yvset, thetavset = RRT_1.RRT_connect(startconfig, env, robot)
                for j in range(len(path)):
                    if j > 0:
                        h.append(env.drawlinestrip(points=np.array(((path[j][0], path[j][1], 0.05),
                                                                    (path[j - 1][0], path[j - 1][1], 0.05))),
                                                   linewidth=5.0, colors=np.array((0, 0, 0))))
                raw_input("It may takes a few seconds to show the path, please wait for it to show up, then press enter"
                          " to execute path , velocity graphs will be generated after that\n")
                # Now that you have computed a path, convert it to an openrave trajectory
                traj = ConvertPathToTrajectory(robot, path)

                # Execute the trajectory on the robot.
            if traj != None:
                robot.GetController().SetPath(traj)

            waitrobot(robot)

        elif i == 1:
            env.Reset()
            # load a scene from ProjectRoom environment XML file
            env.Load('final.env.xml')
            time.sleep(0.1)

            # 1) get the 1st robot that is inside the loaded scene
            # 2) assign it to the variable named 'robot'
            robot = env.GetRobots()[0]

            # tuck in the PR2's arms for driving
            tuckarms(env, robot);

            with env:
                # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
                robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])

                #### YOUR CODE HERE ####

            h = []
            h.append(env.drawlinestrip(
                points=array(((-8, 14.5, 0), (-8, 17.5, 0), (-14, 17.5, 0), (-14, 14.5, 0), (-8, 14.5, 0))),
                linewidth=20.0,
                colors=array((1, 0, 0))))
            h.append(env.drawlinestrip(
                points=array(((1, 8, 0), (1, 24, 0), (-23, 24, 0), (-23, 8, 0), (1, 8, 0))), linewidth=20.0,
                colors=array((1, 1, 0))))

            print 'test case 2 (median number of motion primitives):\n'
            raw_input('press enter to start\n')

            with env:
                path, timeset, totalvelocity, xvset, yvset, thetavset = RRT_2.RRT_connect(startconfig, env, robot)
                for j in range(len(path)):
                    if j > 0:
                        h.append(env.drawlinestrip(points=np.array(((path[j][0], path[j][1], 0.05),
                                                                    (path[j - 1][0], path[j - 1][1], 0.05))),
                                                   linewidth=5.0, colors=np.array((0, 0, 0))))
                raw_input("It may takes a few seconds to show the path, please wait for it to show up, then press enter"
                          " to execute path , velocity graphs will be generated after that\n")

                # Now that you have computed a path, convert it to an openrave trajectory
                traj = ConvertPathToTrajectory(robot, path)

                # Execute the trajectory on the robot.
            if traj != None:
                robot.GetController().SetPath(traj)

            waitrobot(robot)

        else:
            env.Reset()
            # load a scene from ProjectRoom environment XML file
            env.Load('final.env.xml')
            time.sleep(0.1)

            # 1) get the 1st robot that is inside the loaded scene
            # 2) assign it to the variable named 'robot'
            robot = env.GetRobots()[0]

            # tuck in the PR2's arms for driving
            tuckarms(env, robot);

            with env:
                # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
                robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])

                #### YOUR CODE HERE ####

            h = []
            h.append(env.drawlinestrip(
                points=array(((-8, 14.5, 0), (-8, 17.5, 0), (-14, 17.5, 0), (-14, 14.5, 0), (-8, 14.5, 0))),
                linewidth=20.0,
                colors=array((1, 0, 0))))
            h.append(env.drawlinestrip(
                points=array(((1, 8, 0), (1, 24, 0), (-23, 24, 0), (-23, 8, 0), (1, 8, 0))), linewidth=20.0,
                colors=array((1, 1, 0))))

            print 'test case 3 (largest number of motion primitives):\n'
            raw_input('press enter to start\n')

            with env:
                path, timeset, totalvelocity, xvset, yvset, thetavset = RRT_3.RRT_connect(startconfig, env, robot)
                for j in range(len(path)):
                    if j > 0:
                        h.append(env.drawlinestrip(points=np.array(((path[j][0], path[j][1], 0.05),
                                                                    (path[j - 1][0], path[j - 1][1], 0.05))),
                                                   linewidth=5.0, colors=np.array((0, 0, 0))))
                raw_input("It may takes a few seconds to show the path, please wait for it to show up, then press enter"
                          " to execute path , velocity graphs will be generated after that\n")

                # Now that you have computed a path, convert it to an openrave trajectory
                traj = ConvertPathToTrajectory(robot, path)

                # Execute the trajectory on the robot.
            if traj != None:
                robot.GetController().SetPath(traj)

            waitrobot(robot)
            #### END OF YOUR CODE ###

        plt.figure(1)
        plt.title('velocity vs time', fontsize=30)
        plt.plot(timeset, totalvelocity, 'r-', label='total velocity magnitude', markersize=5, linewidth=3.0)
        plt.plot(timeset, xvset, 'b-', label='x velocity', markersize=5, linewidth=2.0)
        plt.plot(timeset, yvset, 'g-', label='y velocity', markersize=5, linewidth=2.0)
        plt.legend(fontsize=20)
        plt.xlabel('time/s', fontsize=30)
        plt.ylabel('velocity/(m/s)', fontsize=30)
        plt.xlim((0, timeset[0]))
        plt.yticks(np.arange(-3.5, 4, 0.5), fontsize=20)
        plt.xticks(np.arange(0, timeset[0], 5), fontsize=20)
        plt.ylim((-3.5, 3.5))

        plt.figure(2)
        plt.title('angular velocity vs time', fontsize=30)
        plt.plot(timeset, thetavset, 'r-', markersize=5, linewidth=3.0)
        plt.xlabel('time/s', fontsize=30)
        plt.ylabel('angular velocity/(rad/s)', fontsize=30)
        plt.xlim((0, timeset[0]))
        plt.yticks(np.arange(-3, 3.5, 0.5), fontsize=20)
        plt.xticks(np.arange(0, timeset[0], 5), fontsize=20)
        plt.ylim((-3, 3))
        plt.show(block=False)

        if i != 2:
            raw_input("Press enter to the next, reconstructing environment takes several seconds\n")
            plt.close()
            plt.close()

    #raw_input("Press enter to the next, reconstructing environment takes several seconds\n")
    raw_input("\nPress enter to end the program\n")
    plt.close()
    plt.close()
    env.Destroy()
