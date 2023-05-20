"""scara_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import numpy as np
from controller import Supervisor  # type: ignore

np.set_printoptions(precision=4, suppress=True)


def c(x):
    return np.cos(x)


def s(x):
    return np.sin(x)


def fkine(q):
    """
    # TODO
    Implement the forward kinematics of the SCARA robot.
    Input: q - joint angles (list of 4 floats)
    Output: T - transformation matrix (4x4 numpy array)
    """
    raise NotImplementedError("FKINE not implemented")


def invkine(x, y, z, phi, l1=0.5, l2=0.5, offset=0.525):
    """
    # DONE
    Implement the inverse kinematics of the SCARA robot.
    Input: x, y, z, phi - desired end-effector pose
    Output: q - joint angles (list of 4 floats)
    """
    c2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if np.abs(c2) > 1:
        print("Position out of reach!")
        return [0.0, 0.0, 0.0, 0.0]

    s2 = np.sqrt(1 - c2 ** 2)
    q2 = np.arctan2(s2, c2)

    k1 = l1 + l2 * c2
    k2 = l2 * s2

    q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    q3 = phi - q1 - q2
    q4 = offset - z 

    return [q1, q2, q3, q4]


class Scara:
    def __init__(self):
        # create the Robot instance.
        self.robot = Supervisor()

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())

        # get joint motor devices
        self.joints = [self.robot.getDevice("joint%d" % i) for i in range(1, 5)]

        # get duck reference
        self.duck = self.robot.getFromDef("DUCK")

        # get gripper reference
        self.gripper = self.robot.getFromDef("GRIPPER")

        self.grasp = False
        self.grasp_prev = False

    def set_position(self, q):
        """
        Set the joint positions of the SCARA robot.
        Input: q - joint angles (list of 4 floats)
        """
        for joint, value in zip(self.joints, q):
            joint.setPosition(value)

    def is_colliding(self, ds=0.15):
        """
        Check if the gripper is colliding with the duck.
        Input: ds - safety distance (float)
        Output: new_pos - new gripper position (list of 3 floats)
                new_yaw - new gripper yaw (float)
                colliding - True if colliding, False otherwise (bool)
        """
        dp = np.array(self.duck.getPose()).reshape(4, 4)[:-1, -1]
        gt = np.array(self.gripper.getPose()).reshape(4, 4)
        gp = gt[:-1, -1]
        gy = np.arctan2(gt[1, 0], gt[0, 0])
        return (
            (gp + np.array([0.0, 0.0, -0.5 * ds])).tolist(),
            gy,
            (np.linalg.norm(dp - gp) < ds),
        )

    def step(self):
        """
        Perform one simulation step.
        Output: -1 if Webots is stopping the controller, 0 otherwise (int)
        """
        if self.grasp is True and self.grasp_prev is False:
            print("GRASP STARTED")
            self.grasp_prev = True
        elif self.grasp is False and self.grasp_prev is True:
            print("GRASP ENDED")
            self.grasp_prev = False

        if self.grasp:
            new_pos, new_yaw, colliding = self.is_colliding()
            self.duck.resetPhysics()
            if colliding:
                self.duck.getField("translation").setSFVec3f(new_pos)
                self.duck.getField("rotation").setSFRotation([0.0, 0.0, 1.0, new_yaw])

        return self.robot.step(self.timestep)

    def delay(self, ms):
        """
        Delay the simulation for a given time.
        Input: ms - delay time in milliseconds (int)
        """
        counter = ms / self.timestep
        while (counter > 0) and (self.step() != -1):
            counter -= 1

    def hold(self):
        """
        Hold the duck.
        """
        self.grasp = True

    def release(self):
        """
        Release the duck.
        """
        self.grasp = False

    def getDuckPose(self):
        """
        Get the duck pose.
        Output: position - duck position (list of 3 floats)
                yaw - duck yaw (float)
        """
        pose = np.array(self.duck.getPose()).reshape(4, 4)
        position = pose[:-1, -1].tolist()
        yaw = np.arctan2(pose[1, 0], pose[0, 0])
        return position + [yaw]


if __name__ == "__main__":
    scara = Scara()

    # Main loop:
    # Perform simulation steps until Webots is stopping the controller
    while scara.step() != -1:
        # TODO
        # Implement your code here
        raise NotImplementedError("TASK is not implemented")

    # Enter here exit cleanup code.
