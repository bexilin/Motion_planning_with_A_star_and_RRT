import random
from math import *
import openravepy
import numpy as np
import time


class Node:
    def __init__(self, x_in, y_in, theta_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.xdot = 0
        self.ydot = 0
        self.thetadot = 0
        self.tip = 0
        self.parent = []
        self.t = 0


def d(c1, c2):
    return sqrt(pow(c2.x - c1.x, 2) + pow(c2.y - c1.y, 2))


def d2(c1, c2):
    return sqrt(pow(c2.x - c1.x, 2) + pow(c2.y - c1.y, 2) + 2*pow(c2.xdot - c1.xdot, 2) + 2*pow(c2.ydot - c1.ydot, 2))


def randomconfig():
    randomcon = []
    if random.randint(1, 10) == 1:
        randomcon.append(random.uniform(-8, -14))
        randomcon.append(random.uniform(14.5, 17.5))
        randomcon.append(random.uniform(-pi, pi))
    else:
        randomcon.append(random.uniform(-29, 29))
        randomcon.append(random.uniform(-29, 29))
        randomcon.append(random.uniform(-pi, pi))

    randomnode = Node(randomcon[0], randomcon[1], randomcon[2])

    if (randomnode.x-(-14.))*(randomnode.x-(-8.)) < 0 and (randomnode.y-14.5)*(randomnode.y-17.5) < 0:
        randomnode.xdot = 0
        randomnode.ydot = 0
    elif (randomnode.x-(-23.))*(randomnode.x-1) < 0 and (randomnode.y-8)*(randomnode.y-24.) < 0:
        randomnode.xdot = random.uniform(0, 0.3)
        randomnode.ydot = random.uniform(0, 0.3)

    return randomnode


def RRT_connect(start, env, robot):
    starttime = time.clock()

    # initialize tree set and input
    tree = []
    startconfig = Node(start[0], start[1], start[2])
    tree.append(startconfig)
    p = []
    li = []
    dt = 0.1
    L = 0.45
    end = 0

    # start to generate tree until reaching goal region
    while 1:
        for i in tree:
            if (i.x-(-8))*(i.x-(-14)) < 0 and (i.y-14.5)*(i.y-17.5) < 0 and i.tip == 1 and i.xdot**2+i.ydot**2 <= 0.01:
                current = i
                end = 1
                break

        if end == 1:
            break

        # generate a random point if it does not collide
        randomcon = randomconfig()
        with env:
            robot.SetTransform([[cos(randomcon.theta),  -sin(randomcon.theta),  0.,  randomcon.x],
                            [sin(randomcon.theta),  cos(randomcon.theta),  0.,  randomcon.y],
                            [0.,  0.,  1.,   0.05],
                            [0.,  0.,  0.,   1.]])
            if env.CheckCollision(robot):
                continue
        p.append(env.plot3(points=np.array((randomcon.x, randomcon.y, 0.05)), pointsize=5.0,
                           colors=np.array((0, 0, 0))))

        # determine closest point in tree
        closest = tree[0]
        distance = d(closest, randomcon)
        for i in tree:
            if d(i, randomcon) < distance:
                closest = i
                distance = d(i, randomcon)

        # if distance is too small, directly go to next iteration
        if 0 <= distance <= 0.05:
            continue
        else:
            # compute input set
            phidotset = [-pi / 10, -pi / 20, 0., pi / 20, pi / 10]
            udotset = [-0.5, -0.25, 0., 0.25, 0.5]
            inputset = [(A, B) for A in phidotset for B in udotset]

            num = 0
            best_dist_last = 1000
            while 1:
                #print 'no'
                # compute the results of all motion primitives, add to tree if no collide, and choose the one that minimize distance
                best_dist = 1000
                primitive = []
                for i in inputset:
                    if (closest.x-(-8.))*(closest.x-(-14)) < 0 and (closest.y-14.5)*(closest.y-17.5) < 0 and i[1] != -0.5 :
                        continue
                    elif (closest.x-(-23.))*(closest.x-1) < 0 and (closest.y-8.)*(closest.y-24.) < 0 and i[1] > 0 :
                        continue

                    oneway = []
                    abandon = 0
                    for j in range(10):
                        if j == 0:
                            formers = closest
                        u = sqrt(formers.xdot**2+formers.ydot**2) + dt*i[1]
                        #else:
                        #    u = u + dt*i[1]
                        if u > 2.5:
                            u = 2.5
                        if u < 0.0:
                            u = 0.0
                        omega = u / L * tan(i[0])
                        #if formers.theta + dt * omega > closest.theta + pi/2 or formers.theta + dt * omega < closest.theta - pi/2:
                        #    omega = 0
                        nexttheta = formers.theta + dt * omega
                        nexts = Node(formers.x + dt * u * cos(nexttheta),
                                     formers.y + dt * u * sin(nexttheta),
                                     nexttheta)
                        nexts.xdot = u * cos(nexttheta)
                        nexts.ydot = u * sin(nexttheta)
                        nexts.thetadot = omega
                        nexts.t = formers.t + dt

                        if (nexts.x - (-29.0)) * (nexts.x - 29.0) > 0 or (nexts.y - (-29.0)) * (nexts.y - 29.0) > 0:
                            abandon = 1
                            break

                        with env:
                            robot.SetTransform([[cos(nexts.theta), -sin(nexts.theta), 0., nexts.x],
                                                [sin(nexts.theta), cos(nexts.theta), 0., nexts.y],
                                                [0., 0., 1., 0.05],
                                                [0., 0., 0., 1.]])
                            if env.CheckCollision(robot):
                                abandon = 1
                                break

                        if j == 9:
                            nexts.tip = 1

                        nexts.parent = formers
                        oneway.append(nexts)
                        formers = nexts

                    if abandon == 1:
                        continue

                    for k in range(len(oneway)):
                        tree.append(oneway[k])
                        primitive.append(oneway[k])
                        if k == 0:
                            li.append(env.drawlinestrip(points=np.array(((oneway[k].x, oneway[k].y, 0.05),
                                                                         (closest.x, closest.y, 0.05))), linewidth=1.0,
                                                        colors=np.array((1, 0, 0))))
                        else:
                            li.append(env.drawlinestrip(points=np.array(((oneway[k].x, oneway[k].y, 0.05),
                                                                         (oneway[k-1].x, oneway[k-1].y,
                                                                          0.05))),
                                                        linewidth=1.0, colors=np.array((1, 0, 0))))
                    p.append(env.plot3(points=np.array((oneway[9].x, oneway[9].y, 0.05)), pointsize=5.0,
                                       colors=np.array((0, 0, 1))))

                    if (closest.x-(-8.))*(closest.x-(-14)) < 0 and (closest.y-14.5)*(closest.y-17.5) < 0:
                        current_dist = sqrt(oneway[9].xdot**2 + oneway[9].ydot**2)
                    elif (randomcon.x-(-23.))*(randomcon.x-1) < 0 and (randomcon.y-8.)*(randomcon.y-24.) < 0 and (closest.x-(-23.))*(closest.x-1) < 0 and (closest.y-8.)*(closest.y-24.) < 0:
                        current_dist = d2(oneway[9], randomcon)
                    else:
                        current_dist = d(oneway[9], randomcon)

                    if current_dist < best_dist:
                        best_dist = current_dist
                        best_node = oneway[9]

                if best_dist <= 0.5 or best_dist_last - best_dist <= 0.5 or len(primitive) == 0 or num == 25:
                    #print best_dist, len(primitive)
                    #raw_input('press and continue')
                    break

                best_dist_last = best_dist
                closest = best_node
                num = num + 1
                #raw_input('press and continue')

    # generate path and plot sets
    path = []
    timeset = []
    totalvelocity = []
    xvset = []
    yvset = []
    thetavset = []
    while current is not startconfig:
        path.append((current.x, current.y, current.theta))
        timeset.append(current.t)
        totalvelocity.append(sqrt(current.xdot ** 2 + current.ydot ** 2))
        xvset.append(current.xdot)
        yvset.append(current.ydot)
        thetavset.append(current.thetadot)
        current = current.parent
    path.append((startconfig.x, startconfig.y, startconfig.theta))
    timeset.append(startconfig.t)
    totalvelocity.append(sqrt(startconfig.xdot**2 + startconfig.ydot**2))
    xvset.append(startconfig.xdot)
    yvset.append(startconfig.ydot)
    thetavset.append(startconfig.thetadot)
    path.reverse()

    endtime = time.clock()
    print "Time: ", endtime - starttime, '\n'
    raw_input('reach goal region, press enter to get computed path\n')

    # output path
    return path, timeset, totalvelocity, xvset, yvset, thetavset
