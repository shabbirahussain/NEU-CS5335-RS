from openravepy import *
from numpy import *
from velctl import *
import time

def run():
    
    # Set up environment
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('../../src/data/cs5335.env.xml')
    #~ env.Load('../../src/data/puma_rob.env.xml')

	# Set up robot
    robot = env.GetRobots()[0] # get the first robot
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
    basemanip=interfaces.BaseManipulation(robot)
    robot.SetActiveDOFs([0, 1, 2, 3, 4, 5])

	# Get coordinates of pick location
    cyl1 = RaveGetEnvironment(1).GetKinBody('cylinder_green_3')
    Tcyl1 = cyl1.GetTransform()
    Tpick = eye(4)
    Tpick[0:2,3] = Tcyl1[0:2,3]
    Tpick[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
    Tpick[2,3] = 0.1
    solutions = ikmodel.manip.FindIKSolutions(Tpick,True)

	# Move to pick location
    traj = basemanip.MoveActiveJoints(goal=solutions[0])
    robot.WaitForController(0)

    # Grab cylinder
    with env:
        robot.Grab(env.GetKinBody('cylinder_green_3'))

	# Move to place location
	Tplace = Tpick.copy()
    Tplace[2,3] = Tplace[2,3] + 0.01
    Tplace[0,3] = Tplace[0,3] - 0.4
    solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    traj = basemanip.MoveActiveJoints(goal=solutions[0])
    robot.WaitForController(0)

	# Drop object
    robot.ReleaseAllGrabbed()

	# Lift arm
    Tplace = Tpick.copy()
    Tplace[2,3] = Tplace[2,3] + 0.2
    solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    traj = basemanip.MoveActiveJoints(goal=solutions[0])
    robot.WaitForController(0)

    raw_input("Press Enter to continue...")



if __name__ == "__main__":
    run()
