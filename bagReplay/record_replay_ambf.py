import time
import os
import sys
from glob import glob

import geometry_msgs
import rosbag
import gc
import numpy as np
# from surgical_robotics_challenge.utils.task3_init import NeedleInitialization
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.utils.utilities import get_input_in_range

from surgical_robotics_challenge.utils import coordinate_frames
dynamic_path = os.path.abspath(__file__ + "/../../")
# data_path = os.path.abspath(__file__+"/../../../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame
from surgical_robotics_challenge.utils.utilities import convert_frame_to_mat
from scipy.spatial.transform import Rotation as R
import pickle
# import dtw


def needle_msg_to_mtx(msg):
    pose_info = msg.pose
    pos = np.array([pose_info.position.x, pose_info.position.y, pose_info.position.z])
    ori_q = np.array([pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z, pose_info.orientation.w])
    mtx = np.eye(4)
    rot = R.from_quat(ori_q)
    mtx[0:3, 3] = pos * 0.1
    mtx[0:3, 0:3] = rot.as_matrix()
    return mtx


def gripper_msg_to_jaw(msg):
    '''
    Map the MTM input to the gripper
    '''
    min = -0.698  # ~ -40 deg
    max = 1.047  # ~60 deg
    input_val = get_input_in_range(msg.position[0], min, max)
    jaw_angle = (input_val - min) / (max - min)
    return jaw_angle


if __name__ == '__main__':
    exp_name = 'simple'
    # exp_name = '3d_complex'
    # exp_name = '3d_straight'
    rosbag_folder = os.path.join(dynamic_path, 'record_bags')
    rosbag_name = os.path.join(rosbag_folder, f'test_{exp_name}.bag')

    print(rosbag_name)

    bag = rosbag.Bag(rosbag_name)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    types = [val[0] for val in bag.get_type_and_topic_info()[1].values()]

    count = 0
    topics_name = []
    psm1_pos = []
    psm2_pos = []
    psm1_pos_ambf = []
    psm2_pos_ambf = []
    t_psm1 = []
    t_psm2 = []
    psm1_jaw = []
    psm2_jaw = []
    psm1_jaw_ambf = []
    psm2_jaw_ambf = []
    ecm_pos = []
    needle_pos = []

    ## ambf raw replay
    for topic, msg, t in bag.read_messages(topics='/ambf/env/psm1/baselink/State'):
        psm1_pos_temp = [msg.joint_positions[0],
                         msg.joint_positions[1],
                         msg.joint_positions[2],
                         msg.joint_positions[3],
                         msg.joint_positions[4],
                         msg.joint_positions[5]]
        psm1_pos_ambf.append(psm1_pos_temp)
        t_psm1.append(t)
        count += 1
    print('psm1 ambf record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics='/ambf/env/psm2/baselink/State'):
        psm2_pos_temp = [msg.joint_positions[0],
                         msg.joint_positions[1],
                         msg.joint_positions[2],
                         msg.joint_positions[3],
                         msg.joint_positions[4],
                         msg.joint_positions[5]]
        psm2_pos_ambf.append(psm2_pos_temp)
        t_psm2.append(t)
        count += 1
    print('psm2 ambf record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics='/MTML/gripper/measured_js'):
        psm1_jaw_ambf_temp = gripper_msg_to_jaw(msg)
        psm1_jaw_ambf.append(psm1_jaw_ambf_temp)
        count += 1
    print('MTML gripper record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics='/MTMR/gripper/measured_js'):
        psm2_jaw_ambf_temp = gripper_msg_to_jaw(msg)
        psm2_jaw_ambf.append(psm2_jaw_ambf_temp)
        count += 1
    print('MTMR gripper record count: ', count)
    count = 0
    gc.collect()

    simulation_manager = SimulationManager('record_test')
    time.sleep(0.5)
    w = simulation_manager.get_world_handle()
    time.sleep(0.2)
    w.reset_bodies()
    time.sleep(0.2)
    cam = ECM(simulation_manager, 'CameraFrame')
    # cam.servo_jp([0.0, 0.05, -0.01, 0.0])
    time.sleep(0.5)
    psm1 = PSM(simulation_manager, 'psm1', add_joint_errors=False)
    time.sleep(0.5)
    psm2 = PSM(simulation_manager, 'psm2', add_joint_errors=False)
    time.sleep(0.5)
    needle = simulation_manager.get_obj_handle('Needle')
    time.sleep(0.2)

    # atn = AttachNeedle(needle, link1, link2)

    needle_pose_list = []

    total_num = min(len(psm1_pos_ambf), len(psm2_pos_ambf), len(psm1_jaw_ambf), len(psm2_jaw_ambf))
    # total_num = min(len(psm1_pos), len(psm2_pos), len(psm1_jaw), len(psm2_jaw))
    print(total_num)
    for i in range(total_num):
        # cam.servo_jp(ecm_pos[i])
        psm1.servo_jp(psm1_pos_ambf[i])
        psm1.set_jaw_angle(psm1_jaw_ambf[i])
        psm2.servo_jp(psm2_pos_ambf[i])
        psm2.set_jaw_angle(psm2_jaw_ambf[i])

        # psm1.servo_jp(psm1_pos[i])
        # psm1.set_jaw_angle(psm1_jaw[i])
        # psm2.servo_jp(psm2_pos[i])
        # psm2.set_jaw_angle(psm2_jaw[i])

        # needle_pose_item = needle.get_pose()
        # needle_pose_list.append(needle_pose_item)

        time.sleep(0.01)
        count += 1
        sys.stdout.write(f'\r Running progress {count}/{total_num}')
        sys.stdout.flush()

    print('Done')

