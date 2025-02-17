import numpy as np
from math import pi
from RobotSerial import RobotSerial
from Frame import Frame
from Robot import Robot
from RobotTrajectory import RobotTrajectory


dh_params = np.array([[0.163, 0., 0.5 * pi, 0.],
                      [0., 0.632, pi, 0.5 * pi],
                      [0., 0.6005, pi, 0.],
                      [0.2013, 0., -0.5 * pi, -0.5 * pi],
                      [0.1025, 0., 0.5 * pi, 0.],
                      [0.094, 0., 0., 0.]])
robot = RobotSerial(dh_params)
theta = np.array([0., 0., -0.25 * pi, 0., 0., 0.])
f = robot.forward(theta)
robot.end_frame
xyz = np.array([[0.28127], [0.], [1.13182]])
abc = np.array([0.5 * pi, 0., pi])
end = Frame.from_euler_3(abc, xyz)
robot.inverse(end)
print("inverse is successful: {0}".format(robot.is_reachable_inverse))
print("axis values: \n{0}".format(robot.axis_values))
# robot.show()

frames = [Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.28127], [0.], [1.13182]])),
          Frame.from_euler_3(np.array([0.25 * pi, 0., 0.75 * pi]), np.array([[0.48127], [0.], [1.13182]])),
          Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.48127], [0.], [0.63182]]))]

trajectory = RobotTrajectory(robot, frames)
trajectory.show(motion="p2p")