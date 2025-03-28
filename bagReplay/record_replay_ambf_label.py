import time
import os
import sys
from glob import glob
import rosbag
import gc
import numpy as np
# from surgical_robotics_challenge.utils.task3_init import NeedleInitialization
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.utils import coordinate_frames
dynamic_path = os.path.abspath(__file__ + "/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame
from surgical_robotics_challenge.utils.utilities import convert_frame_to_mat
from surgical_robotics_challenge.utils.utilities import get_input_in_range
from scipy.spatial.transform import Rotation as R
import pickle
# import dtw
from pynput.keyboard import Listener, Key, KeyCode
from PyKDL import Frame, Rotation, Vector
import json

def needle_msg_to_mtx(msg):
    pose_info = msg.pose
    pos = np.array([pose_info.position.x, pose_info.position.y, pose_info.position.z])
    ori_q = np.array([pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z, pose_info.orientation.w])
    mtx = np.eye(4)
    rot = R.from_quat(ori_q)
    mtx[0:3, 3] = pos
    mtx[0:3, 0:3] = rot.as_matrix()
    return mtx


def needle_msg_to_frame(msg):
    pose_info = msg.pose
    frame = Frame(Rotation.Quaternion(pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z,
                                      pose_info.orientation.w),
                  Vector(pose_info.position.x, pose_info.position.y, pose_info.position.z))
    return frame


def gripper_msg_to_jaw(msg):
    '''
    Map the MTM input to the gripper
    '''
    min = -0.698  # ~ -40 deg
    max = 1.047  # ~60 deg
    input_val = get_input_in_range(msg.position[0], min, max)
    jaw_angle = (input_val - min) / (max - min)
    return jaw_angle

def get_pressed(event):
    global key_d
    if event == KeyCode.from_char('d'):
        key_d = True

def get_released(key):
    if key == Key.esc:
        print('stop listener')
        return False

if __name__ == '__main__':
    key_d = False
    pre_fix = 'test'
    # pre_fix = 'alternative'
    exp_name = '3d_complex'
    # exp_name = '3d_straight'
    # exp_name = '3d_complex_gravity'
    rosbag_folder = os.path.join(dynamic_path, 'record_bags')
    rosbag_name = os.path.join(rosbag_folder, f'{pre_fix}_{exp_name}.bag')

    # if not os.path.exists(output_folder):
    #     print('Create Output Folder')
    #     os.makedirs(output_folder)

    bag = rosbag.Bag(rosbag_name)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    types = [val[0] for val in bag.get_type_and_topic_info()[1].values()]

    count = 0
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
    needle_pose = []
    index_list = []

    needle_v = []
    needle_w = []
    ### needle test replay
    prev_pos = None
    prev_ori = None
    prev_time = None
    for topic, msg, t in bag.read_messages(topics='/ambf/env/phantom/Needle/State'):
        # test_msg = msg
        needle_pose_temp = needle_msg_to_frame(msg)
        needle_pose.append(needle_pose_temp)
        count += 1
        cur_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        cur_ori = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                            msg.pose.orientation.w])
        cur_time = t.secs + 1e-9 * t.nsecs
        if prev_time is not None:
            v = np.linalg.norm(cur_pos - prev_pos) / (cur_time - prev_time)
            # calculate angular velocity from two quaternions
            # https://math.stackexchange.com/questions/90081/quaternion-distance
            w = np.arccos(2 * np.dot(cur_ori, prev_ori) ** 2 - 1) / (cur_time - prev_time)
            w = w * 180 / np.pi
            needle_v.append(v)
            needle_w.append(w)
        prev_pos = cur_pos
        prev_ori = cur_ori
        prev_time = cur_time
    print('needle record count: ', count)
    count = 0

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
    needle = simulation_manager.get_obj_handle('/phantom/Needle')
    time.sleep(1.0)

    needle_pose_list = []

    total_num = min(len(psm1_pos_ambf), len(psm2_pos_ambf), len(psm1_jaw_ambf), len(psm2_jaw_ambf), len(needle_pose))
    # total_num = min(len(psm1_pos), len(psm2_pos), len(psm1_jaw), len(psm2_jaw))
    print('Total number of elements : ', total_num)
    listener = Listener(on_press=get_pressed)
    listener.start()

    for i in range(total_num):
        # needle.set_force(Vector(0, 0, 0))
        # needle.set_torque(Vector(0, 0, 0))
        needle.set_pose(needle_pose[i])

        time.sleep(0.01)

        if key_d:
            index_list.append(count)
            print('\n The index is', count)
        count += 1
        sys.stdout.write(f'\r Running progress {count}/{total_num}')
        sys.stdout.flush()
        key_d = False
    listener.stop()
    listener.join()
    print('Done')

    # save_action = True
    save_action = False

    if save_action:
        save_folder = os.path.join(dynamic_path, 'output', 'data_replay', f'test_{exp_name}')

        if not os.path.exists(save_folder):
            print('Create Save Folder')
            os.makedirs(save_folder)

        save_file = os.path.join(save_folder, f'test_needle_{exp_name}.pkl') # pickle file name

        with open(save_file, 'wb') as f:
            pickle.dump(needle_pose[0:total_num], f)

        print('Needle list saved')

        save_list_file = os.path.join(save_folder, f'test_needle_{exp_name}.json')  # json file name

        data = {'index_list': index_list,
                'needle_linearv': needle_v[0:total_num],
                'needle_angularv': needle_w[0:total_num]}
        json_w = json.dumps(data)
        f_l = open(save_list_file, 'w')
        f_l.write(json_w)
        f_l.close()
        print('Index list saved')
