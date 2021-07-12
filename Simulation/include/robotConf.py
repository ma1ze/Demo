import gym
import math
import numpy
import numpy as np
import pybullet
import pybullet_data
import cv2


def rotationVectorToEulerAngles(rvecs):
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2])
    sz = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return [-x, -y, z]


def setSimEnv(timestep, gravity):
    pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP, 1)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.setTimeStep(timestep)
    pybullet.setGravity(0, 0, -gravity)
    pybullet.loadURDF('plane.urdf')
    # pybullet.loadURDF('table/table.urdf', [0, -0.5, 0], pybullet.getQuaternionFromEuler([0, 0, -math.pi / 2]))


def setCameraPicAndGetPic(robot, width: int = 224, height: int = 224, physicsClientId: int = 0):
    # 摄像头的位置
    cameraPos = [1.5, -0.7, 0.9]
    targetPos = [0, -0.7, 0.7]
    viewMatrix = pybullet.computeViewMatrix(
        cameraEyePosition=cameraPos,
        cameraTargetPosition=targetPos,
        cameraUpVector=[0, 0, 1],
        physicsClientId=physicsClientId
    )
    projectionMatrix = pybullet.computeProjectionMatrixFOV(
        fov=60.0,  # 摄像头的视线夹角
        aspect=1.0,
        nearVal=0.01,  # 摄像头焦距下限
        farVal=20,  # 摄像头能看上限
        physicsClientId=physicsClientId
    )

    width, height, rgbImg, depthImg, segImg = pybullet.getCameraImage(
        width=width, height=height,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix,
        physicsClientId=physicsClientId
    )

    return width, height, rgbImg, depthImg, segImg


pandaDOF = 7
ur10eDOF = 6
motominiDOF = 6


def chooseRobot(index):
    if index == 1:
        panda = Panda(pybullet)
        initialJointPose = [0, math.pi / 4, 0, -math.pi / 4, 0, math.pi / 2, 0, 0, 0]
        for i in range(pandaDOF):
            panda.bullet_client.setJointMotorControl2(
                panda.robot, i, panda.bullet_client.POSITION_CONTROL, initialJointPose[i], force=5 * 240.)
        return panda
    elif index == 2:
        ur10e = UR10e(pybullet)
        initialJointPose = [0, 0, -math.pi / 4, math.pi / 4, -math.pi / 2, -math.pi / 2, 0, 0]
        for i in range(ur10eDOF):
            ur10e.bullet_client.setJointMotorControl2(
                ur10e.robot, i, ur10e.bullet_client.POSITION_CONTROL, initialJointPose[i], force=5 * 240.)
        return ur10e
    elif index == 3:
        motomini = Motomini(pybullet)
        initialJointPose = [0, 0, math.pi / 4, math.pi / 4, 0, 0, math.pi / 2, 0]
        for i in range(ur10eDOF):
            motomini.bullet_client.setJointMotorControl2(
                motomini.robot, i, motomini.bullet_client.POSITION_CONTROL, initialJointPose[i], force=5 * 240.)
        return motomini
    else:
        return


ll = [-7] * pandaDOF
# upper limits for null space (todo: set them to proper range)
ul = [7] * 1
# joint ranges for null space (todo: set them to proper range)
jr = [7] * 1
# restposes for null space
rp = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]


class Panda(object):
    observation_space = gym.spaces.Dict(dict(
        observation=gym.spaces.Box(-np.inf, np.inf, shape=(12,), dtype='float32'),
    ))

    action_space = gym.spaces.Box(low=-1, high=1, shape=(4,), dtype=float)
    position_bounds = [(0.5, 1.0), (-0.25, 0.25), (0.7, 1)]

    def __init__(self, bullet_client):
        self.bullet_client = bullet_client
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.offset = [0, -0.6, 1.4]
        robot_position = [0, 0, 0.62]
        robot_orientation = pybullet.getQuaternionFromEuler([0, 0, -math.pi / 2])
        self.robot = pybullet.loadURDF('franka_panda/panda.urdf', robot_position, robot_orientation, useFixedBase=True,
                                       flags=pybullet.URDF_USE_SELF_COLLISION)
        self.cube = pybullet.loadURDF('cube-06/urdf/cube-06.urdf', [0, -0.6, 0.70],
                                      pybullet.getQuaternionFromEuler([0, 0, 0]))

        pybullet.loadURDF('base/urdf/base.urdf', [0.02, -0.6, 0.65],
                          pybullet.getQuaternionFromEuler([0, 0, math.pi / 2]),
                          useFixedBase=True)

        self.joints = []
        self.links = {}
        joint_type = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']
        for joint_id in range(pybullet.getNumJoints(self.robot)):
            info = pybullet.getJointInfo(self.robot, joint_id)
            data = {
                'jointID': info[0],
                'jointName': info[1].decode('utf-8'),
                'jointType': joint_type[info[2]],
                'jointLowerLimit': info[8],
                'jointUpperLimit': info[9],
                'jointMaxForce': info[10],
                'jointMaxVelocity': info[11]
            }
            self.joints.append(data)
            self.links[data['jointName']] = joint_id
        self.t = 0.
        '''
        Here is the joints information of Franka Emika Panda
        [joint id, joint name, joint type, lower limit, upper limit, max force, max velocity]
        0, 'panda_joint1', 'REVOLUTE', -2.9671, 2.9671, 87.0, 2.175
        1, 'panda_joint2', 'REVOLUTE', -1.8326, 1.8326, 87.0, 2.175
        2, 'panda_joint3', 'REVOLUTE', -2.9671, 2.9671, 87.0, 2.175
        3, 'panda_joint4', 'REVOLUTE', -3.1416, 0.0, 87.0, 2.175
        4, 'panda_joint5', 'REVOLUTE', -2.9671, 2.9671, 12.0, 2.61
        5, 'panda_joint6', 'REVOLUTE', -0.0873, 3.8223, 12.0, 2.61
        6, 'panda_joint7', 'REVOLUTE', -2.9671, 2.9671, 12.0, 2.61
        7, 'panda_joint8', 'FIXED', 0.0, -1.0, 0.0, 0.0
        8, 'panda_hand_joint', 'FIXED', 0.0, -1.0, 0.0, 0.0
        9, 'panda_finger_joint1', 'PRISMATIC', 0.0, 0.04, 20.0, 0.2
        10, 'panda_finger_joint2', 'PRISMATIC', 0.0, 0.04, 20.0, 0.2
        11, 'panda_grasptarget_hand', 'FIXED', 0.0, -1.0, 0.0, 0.0
        '''

    def step(self, action):
        pass

    def reset(self):
        pass

    def observeState(self, prestate):
        state = np.zeros(6)
        state1 = np.zeros(6)
        gripper_position, gripper_orientation, _, _, _, _, gripper_velocity, gripper_angular_velocity = pybullet.getLinkState(
            self.robot, linkIndex=self.links['panda_grasptarget_hand'], computeLinkVelocity=True)
        state[:3] = gripper_position
        state[3:6] = pybullet.getEulerFromQuaternion(gripper_orientation)
        rotationMatrix = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
        state1[:3] = numpy.dot(state[:3] - self.offset, numpy.linalg.inv(rotationMatrix))
        state1[3:6] = [-state[4], state[3], -state[5] + math.pi / 2]
        if state1[5] >= math.pi:
            state1[5] = state1[5] - math.pi
        return state, state1

    def arucoTracker(self, pos, rot):
        self.t += 1. / 60.
        orn = self.bullet_client.getQuaternionFromEuler(rot)
        jointPoses = self.bullet_client.calculateInverseKinematics(
            self.robot, 11, pos, orn, ll, ul, jr, rp, maxNumIterations=5)
        # print(jointPoses)
        for i in range(pandaDOF):
            self.bullet_client.setJointMotorControl2(
                self.robot, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240.)
        pass

    def clawGrasp(self):
        pybullet.setJointMotorControl2(self.robot, 9, pybullet.POSITION_CONTROL, 0)
        pybullet.setJointMotorControl2(self.robot, 10, pybullet.POSITION_CONTROL, 0)
        pass

    def clawOpen(self):
        pybullet.setJointMotorControl2(self.robot, 9, pybullet.POSITION_CONTROL, 0.08)
        pybullet.setJointMotorControl2(self.robot, 10, pybullet.POSITION_CONTROL, 0.08)
        pass


class UR10e(object):
    observation_space = gym.spaces.Dict(dict(
        observation=gym.spaces.Box(-np.inf, np.inf, shape=(12,), dtype='float32'),
    ))

    action_space = gym.spaces.Box(low=-1, high=1, shape=(4,), dtype=float)
    position_bounds = [(0.5, 1.0), (-0.25, 0.25), (0.7, 1)]

    def __init__(self, bullet_client):
        self.bullet_client = bullet_client
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.offset = [0, -0.8, 1.4]
        robot_position = [0, 0, 0.62]
        robot_orientation = pybullet.getQuaternionFromEuler([0, 0, -math.pi / 2])
        self.robot = pybullet.loadURDF('ur10_robot.urdf', robot_position, robot_orientation, useFixedBase=True,
                                       flags=pybullet.URDF_USE_SELF_COLLISION)
        self.cube = pybullet.loadURDF('cube-06/urdf/cube-06.urdf', [0, -0.8, 0.70],
                                      pybullet.getQuaternionFromEuler([0, 0, 0]))

        pybullet.loadURDF('base/urdf/base.urdf', [0.02, -0.8, 0.65],
                          pybullet.getQuaternionFromEuler([0, 0, math.pi / 2]),
                          useFixedBase=True)
        self.joints = []
        self.links = {}
        joint_type = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']
        for joint_id in range(pybullet.getNumJoints(self.robot)):
            info = pybullet.getJointInfo(self.robot, joint_id)
            data = {
                'jointID': info[0],
                'jointName': info[1].decode('utf-8'),
                'jointType': joint_type[info[2]],
                'jointLowerLimit': info[8],
                'jointUpperLimit': info[9],
                'jointMaxForce': info[10],
                'jointMaxVelocity': info[11]
            }
            if data['jointType'] != 'FIXED':
                self.joints.append(data)
                self.links[data['jointName']] = joint_id
            print(self.joints)
        '''
        {'jointID': 1, 'jointName': 'shoulder_pan_joint', 'jointType': 'REVOLUTE', 'jointLowerLimit': -6.28318530718, 'jointUpperLimit': 6.28318530718, 'jointMaxForce': 330.0, 'jointMaxVelocity': 3.14}, 
        {'jointID': 2, 'jointName': 'shoulder_lift_joint', 'jointType': 'REVOLUTE', 'jointLowerLimit': -6.28318530718, 'jointUpperLimit': 6.28318530718, 'jointMaxForce': 330.0, 'jointMaxVelocity': 3.14}, 
        {'jointID': 3, 'jointName': 'elbow_joint', 'jointType': 'REVOLUTE', 'jointLowerLimit': -3.14159265359, 'jointUpperLimit': 3.14159265359, 'jointMaxForce': 150.0, 'jointMaxVelocity': 3.14}, 
        {'jointID': 4, 'jointName': 'wrist_1_joint', 'jointType': 'REVOLUTE', 'jointLowerLimit': -6.28318530718, 'jointUpperLimit': 6.28318530718, 'jointMaxForce': 54.0, 'jointMaxVelocity': 6.28}, 
        {'jointID': 5, 'jointName': 'wrist_2_joint', 'jointType': 'REVOLUTE', 'jointLowerLimit': -6.28318530718, 'jointUpperLimit': 6.28318530718, 'jointMaxForce': 54.0, 'jointMaxVelocity': 6.28}, 
        {'jointID': 6, 'jointName': 'wrist_3_joint', 'jointType': 'REVOLUTE', 'jointLowerLimit': -6.28318530718, 'jointUpperLimit': 6.28318530718, 'jointMaxForce': 54.0, 'jointMaxVelocity': 6.28}
        '''

        self.t = 0.

    def step(self, action):
        pass

    def reset(self):
        pass

    def observeState(self, prestate):
        state = np.zeros(6)
        state1 = np.zeros(6)
        gripper_position, gripper_orientation, _, _, _, _, gripper_velocity, gripper_angular_velocity = pybullet.getLinkState(
            self.robot, linkIndex=self.links['gripper_finger_joint'], computeLinkVelocity=True)
        state[:3] = gripper_position
        state[3:6] = pybullet.getEulerFromQuaternion(gripper_orientation)
        rotationMatrix = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
        state1[:3] = numpy.dot(state[:3] - self.offset, numpy.linalg.inv(rotationMatrix))
        state1[3:6] = [-state[4], state[3], -state[5] + math.pi / 2]
        if state1[5] >= math.pi:
            state1[5] = state1[5] - math.pi
        return state, state1

    def arucoTracker(self, pos, rot):
        self.t += 1. / 60.
        orn = self.bullet_client.getQuaternionFromEuler(rot)
        jointPoses = self.bullet_client.calculateInverseKinematics(
            self.robot, 10, pos, orn, rp, maxNumIterations=5)
        # for i in range(ur10eDOF):
        #     self.bullet_client.setJointMotorControl2(
        #         self.robot, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=240.)

        for joint_id in range(6):
            pybullet.setJointMotorControl2(
                self.robot, self.joints[joint_id]['jointID'],
                pybullet.POSITION_CONTROL,
                targetPosition=jointPoses[joint_id],
            )

        pass

    def clawGrasp(self):
        for joint_id in range(6, 12):
            value = 0.3
            pybullet.setJointMotorControl2(
                self.robot,
                self.joints[joint_id]['jointID'],
                pybullet.POSITION_CONTROL,
                targetPosition=value,
            )
        pass

    def clawOpen(self):
        for joint_id in range(6, 12):
            value = 0
            pybullet.setJointMotorControl2(
                self.robot,
                self.joints[joint_id]['jointID'],
                pybullet.POSITION_CONTROL,
                targetPosition=value,
            )
        pass


class Motomini(object):
    observation_space = gym.spaces.Dict(dict(
        observation=gym.spaces.Box(-np.inf, np.inf, shape=(12,), dtype='float32'),
    ))

    action_space = gym.spaces.Box(low=-1, high=1, shape=(4,), dtype=float)
    position_bounds = [(0.5, 1.0), (-0.25, 0.25), (0.7, 1)]

    def __init__(self, bullet_client):
        self.bullet_client = bullet_client
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.offset = [0, -0.4, 1]
        robot_position = [0, 0, 0.62]
        robot_orientation = pybullet.getQuaternionFromEuler([0, 0, -math.pi / 2])
        self.robot = pybullet.loadURDF('motomini.urdf', robot_position, robot_orientation, useFixedBase=True,
                                       flags=pybullet.URDF_USE_SELF_COLLISION)
        self.cube = pybullet.loadURDF('cube-06/urdf/cube-06.urdf', [0, -0.4, 0.70],
                                      pybullet.getQuaternionFromEuler([0, 0, 0]))
        pybullet.loadURDF('base/urdf/base.urdf', [0.02, -0.4, 0.65],
                          pybullet.getQuaternionFromEuler([0, 0, math.pi / 2]),
                          useFixedBase=True)
        self.joints = []
        self.links = {}
        joint_type = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']
        for joint_id in range(pybullet.getNumJoints(self.robot)):
            info = pybullet.getJointInfo(self.robot, joint_id)
            data = {
                'jointID': info[0],
                'jointName': info[1].decode('utf-8'),
                'jointType': joint_type[info[2]],
                'jointLowerLimit': info[8],
                'jointUpperLimit': info[9],
                'jointMaxForce': info[10],
                'jointMaxVelocity': info[11]
            }
            self.joints.append(data)
            self.links[data['jointName']] = joint_id
        self.t = 0.
        '''
        Here is the joints information of Yaskawa Motoman motomini
        [joint id, joint name, joint type, lower limit, upper limit, max force, max velocity]
        0, 'world', 'FIXED', 0.0, -1.0, 0.0, 0.0
        1, 'joint_1_s', 'REVOLUTE', -2.967, 2.967, 0.12, 5.4977
        2, 'joint_2_l', 'REVOLUTE', -1.4835, 1.5707, 0.12, 5.4977
        3, 'joint_3_u', 'REVOLUTE', -0.8726, 1.5707, 0.12, 7.3304
        4, 'joint_4_r', 'REVOLUTE', -2.4434, 2.4434, 0.07, 10.4719
        5, 'joint_5_b', 'REVOLUTE', -0.5235, 3.6651, 0.07, 10.4719
        6, 'joint_6_t', 'REVOLUTE', -6.2831, 6.2831, 0.07, 10.4719
        7, 'joint_6_t-tool0', 'FIXED', 0.0, -1.0, 0.0, 0.0
        8, 'gripper_palm_joint', 'FIXED', 0.0, -1.0, 0.0, 0.0
        9, 'gripper_finger_joint_l', 'PRISMATIC', 0.0, 0.015, 2.0, 0.1
        10, 'gripper_finger_joint_r', 'PRISMATIC', 0.0, 0.015, 2.0, 0.1
        11, 'base_link-base', 'FIXED', 0.0, -1.0, 0.0, 0.0
        '''

    def step(self, action):
        pass

    def reset(self):
        pass

    def observeState(self, prestate):
        state = np.zeros(6)
        state1 = np.zeros(6)
        gripper_position, gripper_orientation, _, _, _, _, gripper_velocity, gripper_angular_velocity = pybullet.getLinkState(
            self.robot, linkIndex=self.links['base_link-base'], computeLinkVelocity=True)
        state[:3] = gripper_position
        state[3:6] = pybullet.getEulerFromQuaternion(gripper_orientation)
        rotationMatrix = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
        state1[:3] = numpy.dot(state[:3] - self.offset, numpy.linalg.inv(rotationMatrix))
        state1[3:6] = [-state[4], state[3], -state[5] + math.pi / 2]
        if state1[5] >= math.pi:
            state1[5] = state1[5] - math.pi
        return state, state1

    def arucoTracker(self, pos, rot):
        self.t += 1. / 60.
        orn = self.bullet_client.getQuaternionFromEuler(rot)
        jointPoses = self.bullet_client.calculateInverseKinematics(
            self.robot, 9, pos, orn, ll, ul, jr, rp, maxNumIterations=5)
        for i in range(motominiDOF):
            self.bullet_client.setJointMotorControl2(
                self.robot, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240.)
        pass

    def clawGrasp(self):
        pybullet.setJointMotorControl2(self.robot, 9, pybullet.POSITION_CONTROL, 0)
        pybullet.setJointMotorControl2(self.robot, 10, pybullet.POSITION_CONTROL, 0)
        pass

    def clawOpen(self):
        pybullet.setJointMotorControl2(self.robot, 9, pybullet.POSITION_CONTROL, 0.015)
        pybullet.setJointMotorControl2(self.robot, 10, pybullet.POSITION_CONTROL, 0.015)
        pass
