import numpy as np
from RobotSerial import RobotSerial
from math import pi
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button
from loguru import logger


class RobotJointControl():
    def __init__(self, robot, max_interpolation_steps):
        self.robot = robot
        self.goal_joint_angles = np.zeros((1, len(robot.axis_values)))
        self.max_interpolation_steps = max_interpolation_steps
        # self.check_limits()
    
    def compute_interpolation(self):
        print("Calculating interpolation values")
        current_joint_angles = self.robot.axis_values
        print(f"Current Joint angles are: {current_joint_angles}")
        print(f"Goal Joint angles are: {self.goal_joint_angles}")
        joint_angles_error = np.subtract(self.goal_joint_angles, current_joint_angles)
        print(f"Joint angle error: {joint_angles_error}")
        interpolation_step_per_joint = np.divide(joint_angles_error, self.max_interpolation_steps)
        print(f"interpolation_step_per_joint: {interpolation_step_per_joint}")
        interpolated_joint_angles = np.zeros((self.max_interpolation_steps, len(current_joint_angles)))
        print(interpolated_joint_angles.shape)
        increments_per_joint = current_joint_angles
        for interpolation_step in range(self.max_interpolation_steps):
            for axis_idx in range(len(current_joint_angles)):
                increments_per_joint[axis_idx] += interpolation_step_per_joint[axis_idx]
                interpolated_joint_angles[interpolation_step] = increments_per_joint
        print(interpolated_joint_angles.round(5))
        return interpolated_joint_angles.round(5)
    
    def check_limits(self):
        print("Checking Joints limits")
        joints_limits = self.robot.joints_limits["joints_limits"]
        print(joints_limits)
        for (idx, goal_angle) in enumerate(self.goal_joint_angles):
            joint_entry_limits = joints_limits["joint"+str(idx+1)]
            print(joint_entry_limits)
            print(goal_angle)
            if goal_angle < joint_entry_limits["min_angle"] or goal_angle > joint_entry_limits["max_angle"]:
                logger.error(f"Goal joint value exceeds limits for {"joint"+str(idx+1)} - Aborting execution!")
                return False
        logger.success("Goal joints values are within joints limits")
        return True
            
    def execute_motion(self, visualize):
        print("Executing motion")
        res = self.check_limits()
        if res:
            interp_angles = self.compute_interpolation()
            if visualize:
                self.visualize_motion(interp_angles)

    def visualize_motion(self, interp_angles, time_step=0.1):
        plt.ion()
        for angle in interp_angles:
            self.robot.forward(angle)
            self.robot.show()
            plt.pause(time_step)
        plt.show(block=True)
        
    def get_user_input(self):
        fig = plt.figure()
        ax_join1 = fig.add_axes([0.15, 0.3, 0.75, 0.02])
        ax_join2 = fig.add_axes([0.15, 0.4, 0.75, 0.02])
        ax_join3 = fig.add_axes([0.15, 0.5, 0.75, 0.02])
        ax_join4 = fig.add_axes([0.15, 0.6, 0.75, 0.02])
        ax_join5 = fig.add_axes([0.15, 0.7, 0.75, 0.02])
        ax_join6 = fig.add_axes([0.15, 0.8, 0.75, 0.02])
        joint1_slider = Slider(ax_join1, "Joint 1", -7.0, 7.0, valinit=0.0)
        joint2_slider = Slider(ax_join2, "Joint 2", -7.0, 7.0, valinit=0.0)
        joint3_slider = Slider(ax_join3, "Joint 3", -4.0, 4.0, valinit=0.0)
        joint4_slider = Slider(ax_join4, "Joint 4", -7.0, 7.0, valinit=0.0)
        joint5_slider = Slider(ax_join5, "Joint 5", -7.0, 7.0, valinit=0.0)
        joint6_slider = Slider(ax_join6, "Joint 6", -7.0, 7.0, valinit=0.0)
                
        # Sliders' callback
        def update(val):
            self.goal_joint_angles = np.array([
                joint1_slider.val, 
                joint2_slider.val, 
                joint3_slider.val,
                joint4_slider.val,
                joint5_slider.val,
                joint6_slider.val
                ])
            print(f"Joint angles: {self.goal_joint_angles}")
            fig.canvas.draw_idle()
        joint1_slider.on_changed(update)
        joint2_slider.on_changed(update)
        joint3_slider.on_changed(update)
        joint4_slider.on_changed(update)
        joint5_slider.on_changed(update)
        joint6_slider.on_changed(update)
        ax_execution = fig.add_axes([0.65, 0.025, 0.1, 0.04])
        execution_button = Button(ax_execution, 'Execute', color='lightblue', hovercolor='0.975')
        def execution_button_on_clicked(mouse_event):
            print("Execution button clicked")
            self.execute_motion(visualize=True)
        execution_button.on_clicked(execution_button_on_clicked)
        ax_reset = fig.add_axes([0.8, 0.025, 0.1, 0.04])
        reset_button = Button(ax_reset, 'Reset', color='red', hovercolor='0.975')
        def reset_button_on_clicked(mouse_event):
            print("Reset button clicked")
            joint1_slider.reset()
            joint2_slider.reset()
            joint3_slider.reset()
            joint4_slider.reset()
            joint5_slider.reset()
            joint6_slider.reset()
        reset_button.on_clicked(reset_button_on_clicked)
        
        # Set goal to initial sliders' values
        self.goal_joint_angles = np.array([
            joint1_slider.valinit,
            joint2_slider.valinit,
            joint3_slider.valinit,
            joint4_slider.valinit,
            joint5_slider.valinit,
            joint6_slider.valinit,
            ])
        self.robot.forward(self.goal_joint_angles)
        self.robot.show()
        
        plt.show()

        
        
        
if __name__ == "__main__":
    dh_params = np.array([[0.163, 0., 0.5 * pi, 0.],
                      [0., 0.632, pi, 0.5 * pi],
                      [0., 0.6005, pi, 0.],
                      [0.2013, 0., -0.5 * pi, -0.5 * pi],
                      [0.1025, 0., 0.5 * pi, 0.],
                      [0.094, 0., 0., 0.]])
    robot = RobotSerial(dh_params, plot_xlim=[-1.3, 1.3], plot_ylim=[-1.3, 1.3], plot_zlim=[0.0, 1.0])
    goal_joint_angles = np.array([0.5 * pi,0.25 * pi, -0.25 * pi, -0.25 * pi, 0.5 * pi, 0.25 * pi])
    robot_control = RobotJointControl(robot, max_interpolation_steps=15)
    robot_control.get_user_input()
    # robot_control.execute_motion(goal_joint_angles, visualize=True)

        