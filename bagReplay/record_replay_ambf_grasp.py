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
from surgical_robotics_challenge.kinematics.psmFK import *
from surgical_robotics_challenge.kinematics.psmIK import *
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.utils import coordinate_frames
dynamic_path = os.path.abspath(__file__ + "/../../")
# data_path = os.path.abspath(__file__+"/../../../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame
from surgical_robotics_challenge.utils.utilities import convert_frame_to_mat
from scipy.spatial.transform import Rotation as R
# from sklearn import preprocessing
# from ros_numpy import numpify
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

def needle_msg_to_frame(msg):
    pose_info = msg.pose
    frame = Frame(Rotation.Quaternion(pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z,
                                      pose_info.orientation.w),
                  Vector(pose_info.position.x * 0.1, pose_info.position.y * 0.1, pose_info.position.z * 0.1))
    return frame


def gripper_msg_to_jaw(msg):
    min = -0.1
    max = 0.51
    jaw_angle = msg.position[0] + min / (max - min)
    return jaw_angle

if __name__ == '__main__':
    data_folder = os.path.join(dynamic_path, "Data", 'grasping')
    # exp_name = '3dmed'  # 3dmed
    exp_name = 'oldphantom' # 2021 phantom

    rosbag_name = os.path.join(data_folder, f'grasp_{exp_name}.bag')
    print(rosbag_name)
    # rosbag_name = file_list[3]  ## phantom_old_shang_01.bag
    # if not os.path.exists(output_folder):
    #     print('Create Output Folder')
    #     os.makedirs(output_folder)

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
    needle_pose = []

    for topic, msg, t in bag.read_messages(topics=topics[11]):
        assert topic == '/ambf/env/Needle/State', 'load incorrect topics for needle'
        # test_msg = msg
        needle_pose_temp = needle_msg_to_frame(msg)
        needle_pose.append(needle_pose_temp)
        count += 1
    print('needle record count: ', count)
    count = 0

    ## ambf raw replay
    for topic, msg, t in bag.read_messages(topics=topics[14]):
        assert topic == '/ambf/env/psm1/baselink/State', 'load incorrect topics'
        psm1_pos_temp = [msg.joint_positions[0],
                         msg.joint_positions[1],
                         msg.joint_positions[2] / 10.,
                         msg.joint_positions[3],
                         msg.joint_positions[4],
                         msg.joint_positions[5]]
        psm1_pos_ambf.append(psm1_pos_temp)
        t_psm1.append(t)
        count += 1
    print('psm1 ambf record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics=topics[16]):
        assert topic == '/ambf/env/psm2/baselink/State', 'load incorrect topics'
        psm2_pos_temp = [msg.joint_positions[0],
                         msg.joint_positions[1],
                         msg.joint_positions[2] / 10.,
                         msg.joint_positions[3],
                         msg.joint_positions[4],
                         msg.joint_positions[5]]
        psm2_pos_ambf.append(psm2_pos_temp)
        t_psm2.append(t)
        count += 1
    print('psm2 ambf record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics=topics[0]):
        assert topic == '/MTML/gripper/measured_js', 'load incorrect topics'
        psm1_jaw_ambf_temp = gripper_msg_to_jaw(msg)
        psm1_jaw_ambf.append(psm1_jaw_ambf_temp)
        count += 1
    print('MTML gripper record count: ', count)
    count = 0

    for topic, msg, t in bag.read_messages(topics=topics[1]):
        assert topic == '/MTMR/gripper/measured_js', 'load incorrect topics'
        psm2_jaw_ambf_temp = gripper_msg_to_jaw(msg)
        psm2_jaw_ambf.append(psm2_jaw_ambf_temp)
        count += 1
    print('MTMR gripper record count: ', count)
    count = 0
    gc.collect()

    # dtw_align = dtw.dtw(psm1_pos_ambf,psm1_pos) ### (query, reference)

    # test_path_q = dtw.warp(dtw_align, index_reference=False)
    # test_path_r = dtw.warp(dtw_align, index_reference=True)

    simulation_manager = SimulationManager('record_test')
    time.sleep(0.5)
    w = simulation_manager.get_world_handle()
    time.sleep(0.2)
    w.reset_bodies()
    time.sleep(0.2)
    cam = ECM(simulation_manager, 'CameraFrame')
    cam.servo_jp([0.0, 0.05, -0.01, 0.0])
    time.sleep(0.5)
    psm1 = PSM(simulation_manager, 'psm1', add_joint_errors=False)
    time.sleep(0.5)
    # if psm1.is_present():
    #     print('psm1 run')
    #     T_psmtip_c = coordinate_frames.PSM1.T_tip_cam
    #     T_psmtip_b = psm1.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
    #     psm1.set_home_pose(T_psmtip_b)
    #     time.sleep(1.0)
    psm2 = PSM(simulation_manager, 'psm2', add_joint_errors=False)
    time.sleep(0.5)
    # if psm2.is_present():
    #     print('psm2 run')
    #     T_psmtip_c = coordinate_frames.PSM2.T_tip_cam
    #     T_psmtip_b = psm2.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
    #     psm2.set_home_pose(T_psmtip_b)
    #     time.sleep(1.0)
    needle = simulation_manager.get_obj_handle('Needle')
    time.sleep(0.2)

    # assert len(psm1_pos) == len(psm2_pos), 'psm1 and psm2 run time not equal'
    #
    # cam.servo_jp(ecm_pos[0])


    ### Add needle attachment code
    # needle = simulation_manager.get_obj_handle('Needle')
    # link1 = simulation_manager.get_obj_handle('psm1' + '/toolyawlink')
    # link2 = simulation_manager.get_obj_handle('psm2' + '/toolyawlink')

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

        time.sleep(0.01)
        count += 1

        # ### for psm 1 pose
        # ### 3d med
        # if count == 1892:
        #     T_psm1_b_w = psm1.get_T_b_w()  # from world to PSM base
        #     T_psm1_w_b = psm1.get_T_w_b()  # from PSM base to world
        #
        #     psm1_jp_init = psm1.measured_jp()
        #     mtx_grasp = compute_FK(psm1_jp_init, 7)
        #     frame_psm1_old = convert_mat_to_frame(mtx_grasp)
        #
        #     needle_init = needle.get_pose()
        #     T_needle_psmtip = needle_init.Inverse() * T_psm1_b_w * frame_psm1_old
        #     print(T_needle_psmtip)
        # ### old phantom
        # if count == 2579:
        #     T_psm1_b_w = psm1.get_T_b_w()  # from world to PSM base
        #     T_psm1_w_b = psm1.get_T_w_b()  # from PSM base to world
        #
        #     psm1_jp_init = psm1.measured_jp()
        #     mtx_grasp = compute_FK(psm1_jp_init, 7)
        #     frame_psm1_old = convert_mat_to_frame(mtx_grasp)
        #
        #     needle_init = needle.get_pose()
        #     T_needle_psmtip = needle_init.Inverse() * T_psm1_b_w * frame_psm1_old
        #     print(T_needle_psmtip)

        # ### for needle
        # ### 3d med
        # if count == 2525:
        #     needle_current = needle_pose[i]
        #     print(needle_current)
        # ### old phantom
        if count == 3335:
            needle_current = needle_pose[i]
            print(needle_current)

        sys.stdout.write(f'\r Running progress {count}/{total_num}')
        sys.stdout.flush()

    print('Done')

    ### psm 1 pose
    # ### 3d med
    # [[0.94974, -0.177397, -0.257921;
    # -0.243021, -0.937175, -0.250287;
    # -0.197317, 0.300388, -0.933185]
    # [0.00477713, 0.0116078, 0.0071877]]

    # ### old phantom
    # [[0.916646, -0.380538, -0.122273;
    # -0.399522, -0.863202, -0.308651;
    # 0.0119074, 0.331774, -0.943284]
    # [0.00302862, 0.0115958, 0.00599355]]

    ### needle pose

    # ### 3d med
    # [[0.919976, 0.39151, -0.0190948;
    # -0.192544, 0.4938, 0.847991;
    # 0.341426, -0.776454, 0.529667]
    # [0.00266491, 0.0351907, 0.0918371]]
    # ### old phantom
    # [[0.935925, 0.351379, 0.0240132;
    # -0.0902596, 0.173392, 0.980708;
    # 0.340436, -0.920037, 0.193998]
    # [0.000114786, 0.00639507, 0.106231]]
