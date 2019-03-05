import random
from math import *
import openravepy
import numpy as np


class Node:
    def __init__(self, q1_in, q2_in, q3_in, q4_in, q5_in, q6_in):
        self.q1 = q1_in
        self.q2 = q2_in
        self.q3 = q3_in
        self.q4 = q4_in
        self.q5 = q5_in
        self.q6 = q6_in
        self.parent = []


def d(c1, c2):
    return sqrt(pow(c2.q1 - c1.q1, 2) + pow(c2.q2 - c1.q2, 2) + pow(c2.q3 - c1.q3, 2) + pow(c2.q4 - c1.q4, 2) + pow(
        c2.q5 - c1.q5, 2) + pow(c2.q6 - c1.q6, 2))


def randomconfig(goalconfig, d):
    if random.randint(1, 100) == 1:
        return goalconfig
    else:
        randomcon = []
        randomcon.append(random.uniform(-0.56, 2.13))
        randomcon.append(random.uniform(-0.35, 1.29))
        randomcon.append(random.uniform(-2.12, -0.15))
        randomcon.append(random.uniform(-0.65, 3.74))
        randomcon.append(random.uniform(-3.14, 3.14))
        randomcon.append(random.uniform(-2.0, -0.1))
        if random.randint(1, 9) == 1:
            randomcon = Node(randomcon[0], randomcon[1], randomcon[2], randomcon[3], randomcon[4], randomcon[5])
            dist = d(randomcon, goalconfig)
            vec = [randomcon.q1 - goalconfig.q1, randomcon.q2 - goalconfig.q2, randomcon.q3 - goalconfig.q3,
                   randomcon.q4 - goalconfig.q4, randomcon.q5 - goalconfig.q5, randomcon.q6 - goalconfig.q6]
            dire = [i * 0.2 / dist for i in vec]
            return Node(goalconfig.q1+dire[0], goalconfig.q2+dire[1], goalconfig.q3+dire[2], goalconfig.q4+dire[3],
                        goalconfig.q5+dire[4], goalconfig.q6+dire[5])
        else:
            return Node(randomcon[0], randomcon[1], randomcon[2], randomcon[3], randomcon[4], randomcon[5])


def RRT_connect(start, goal, env, robot, GetEETransform):
    # initialize tree set and goalconfig
    tree = []
    startconfig = Node(start[0], start[1], start[2], start[3], start[4], start[5])
    tree.append(startconfig)
    goalconfig = Node(goal[0], goal[1], goal[2], goal[3], goal[4], goal[5])
    x = []

    # start to generate tree until reaching goal
    while 1:
        if goalconfig in tree:
            break

        # generate random point if the latest extend point[2] is not in [0.1,0.2], otherwise try to connect it to goal
        randomcon = randomconfig(goalconfig, d)
        transform = GetEETransform(robot, [randomcon.q1, randomcon.q2, randomcon.q3, randomcon.q4, randomcon.q5,
                                           randomcon.q6])
        x.append(env.plot3(points=np.array((transform[0][3], transform[1][3], transform[2][3])), pointsize=5.0,
                           colors=np.array((0, 0, 0))))

        # determine closest point in tree if the latest extend point[2] is not in [0.1,0.2], otherwise is that point
        closest = tree[0]
        distance = d(closest, randomcon)
        for i in tree:
            if d(i, randomcon) < distance:
                closest = i
                distance = d(i, randomcon)

        # if distance too small, directly expand and go to next iteration
        if 0 <= distance <= 0.05:
            if not env.CheckCollision(robot):
                tree.append(randomcon)
                continue
        else:
            # find the expand direction
            vector = [randomcon.q1 - closest.q1, randomcon.q2 - closest.q2, randomcon.q3 - closest.q3,
                      randomcon.q4 - closest.q4, randomcon.q5 - closest.q5, randomcon.q6 - closest.q6]
            direction = [i * 0.05 / distance for i in vector]

            # try to connect random point
            formerextend = closest
            while 1:
                if d(formerextend, randomcon) <= 0.05:
                    randomcon.parent = formerextend
                    break
                nextextend = Node(formerextend.q1 + direction[0], formerextend.q2 + direction[1],
                                  formerextend.q3 + direction[2], formerextend.q4 + direction[3],
                                  formerextend.q5 + direction[4], formerextend.q6 + direction[5])
                with env:
                    robot.SetActiveDOFValues(
                        [nextextend.q1, nextextend.q2, nextextend.q3, nextextend.q4, nextextend.q5, nextextend.q6])
                    if env.CheckCollision(robot):
                        break

                nextextend.parent = formerextend
                tree.append(nextextend)
                formerextend = nextextend

    # generate path
    path = []
    current = goalconfig
    while current is not startconfig:
        path.append((current.q1, current.q2, current.q3, current.q4, current.q5, current.q6))
        current = current.parent
    path.append((startconfig.q1, startconfig.q2, startconfig.q3, startconfig.q4, startconfig.q5, startconfig.q6))
    path.reverse()

    # shortcut smoothing
    smoothpath = path[:]
    for i in range(150):
        randconfig1 = random.choice(smoothpath)
        randconfig2 = random.choice(smoothpath)
        if randconfig1 == randconfig2:
            continue
        distance = sqrt(pow(randconfig2[0] - randconfig1[0], 2) + pow(randconfig2[1] - randconfig1[1], 2) + pow(
            randconfig2[2] - randconfig1[2], 2) + pow(randconfig2[3] - randconfig1[3], 2) + pow(
            randconfig2[4] - randconfig1[4], 2) + pow(randconfig2[5] - randconfig1[5], 2))
        vector = [randconfig2[0] - randconfig1[0], randconfig2[1] - randconfig1[1], randconfig2[2] - randconfig1[2],
                  randconfig2[3] - randconfig1[3], randconfig2[4] - randconfig1[4], randconfig2[5] - randconfig1[5]]
        direction = [i * 0.05 / distance for i in vector]
        checknum = int(distance / 0.05)
        if checknum == 0:
            continue

        # begin check, if pass, return substitute set
        result = 0
        substitute = []
        formerextend = randconfig1
        for j in range(checknum-1):
            nextextend = [formerextend[0] + direction[0], formerextend[1] + direction[1],
                          formerextend[2] + direction[2], formerextend[3] + direction[3],
                          formerextend[4] + direction[4], formerextend[5] + direction[5]]
            with env:
                robot.SetActiveDOFValues(nextextend)
                if env.CheckCollision(robot):
                    result = 1
                    break

            substitute.append(nextextend)
            formerextend = nextextend

        # connect two random points by line if applicable and generate smooth path
        if result == 0:
            a = smoothpath.index(randconfig1)
            b = smoothpath.index(randconfig2)
            inv = 0
            if a > b:
                t = a
                a = b
                b = t
                inv = 1
            smoothpath[a+1:b]=[]
            if inv == 0:
                substitute.reverse()
            for k in substitute:
                smoothpath.insert(a + 1, k)

    # output path
    return path, smoothpath
