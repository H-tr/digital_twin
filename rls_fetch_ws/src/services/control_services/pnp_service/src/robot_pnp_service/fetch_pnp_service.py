#!/usr/bin/env python

import rospy
import math
import moveit_commander
from moveit_msgs.msg import Grasp
import tf.transformations as T
import tf2_ros
from geometry_msgs.msg import TransformStamped

from fetch_robot.fetch_robot import FetchRobot
from rls_perception_msgs.srv import *
from rls_control_msgs.srv import *
from fetch_robot.fetch_joint_position_config import *
from robot_pnp_service import PnpService

''' --------- Settings -----------'''
ASK_BEFORE_EXECUTE = False
MOVE_CLOSER_TO_TABLE = True
VIS_GRASP = True

''' --------- Constant ----------'''
BASE_FRAME_NAME = 'base_link'
GRIPPER_LENGTH = 0.2  # metres
GRIPPER_CLOSE_OPENING = 0.05  # between 0 and 0.16
PLANNING_SCENE_SYNC_WAIT_TIME = 1  # second
BUFFER_HEIGHT = 0.008
PLACE_BUFFER_HEIGHT = 0.2  # meter
APPROACH_HEIGTH = 0.1
SIDE_APPROACH_DIST = 0.1
DEPART_HEIGHT = 0.1
TOP_PLACE_APPROACH_DIST = 0.1
# READY_POSE_HEAD_TILT = 42
READY_POSE_HEAD_TILT = 55
READY_POSE_FINAL_HEAD_TILT = 42
CLOSE_GRIPPER_EFFORT = 70
SAFE_TABLE_DISTANCE = 0.35
MOTION_PLANNING_BUFFER_HEIGHT = 0.08

DIST_TO_MOVE_AWAY_FROM_TABLE = 0.4  # meter

''' ---------- Defines ----------- '''
FAIL_TO_MOVE_TO_READY_POSE = 1

''' ---------- Code -------------- '''

class FetchPnpService(PnpService):
    def __init__(self):
        PnpService.__init__(self)

        # # torso controller
        # self._torso = fetch_api.Torso()

        # # arm controller
        # self._arm = fetch_api.ArmV2()
        # self._gripper = fetch_api.Gripper()
        # self._head = fetch_api.Head()
        # self._base = fetch_api.Base()

        # # arm, torso, _gripper are all necessary, raise error if initialization fails
        # if self._torso is None or self._arm is None or self._gripper is None:
        #     raise RuntimeError("PNPServer: Failed to init PNPServer")

        self.robot = FetchRobot()
        if self.robot is None:
            raise RuntimeError("PNPServer: Failed to init PNPServer")

        # Table Segmentor
        rospy.loginfo("Segmentor: Waiting for table pointcloud segmentation service /table_segmentation_service ...")
        rospy.wait_for_service('rls_perception_services/fetch/table_segmentation_service')
        self._table_segmentor_srv = rospy.ServiceProxy('rls_perception_services/fetch/table_segmentation_service', TableSegmentation)
        rospy.loginfo("Service found!")

        # planning scene
        self._planning_frame = 'base_link'
        self._scene = moveit_commander.PlanningSceneInterface()

        # config
        self._depart_height = DEPART_HEIGHT
        self._depart_height_min = 0.15
        self._depart_height_max = 0.20
        self._top_grasp_approach_height = APPROACH_HEIGTH
        self._side_grasp_approach_height = SIDE_APPROACH_DIST
        self._plan_attempt = 10
        self._sync_wait_time = PLANNING_SCENE_SYNC_WAIT_TIME
        self._top_place_approach_dist = TOP_PLACE_APPROACH_DIST

        # advertise service
        rospy.Service('rls_control_services/fetch/pnp', PickPlace, self._handle_pnp_req)
        rospy.loginfo('PickNPlace Service is Ready!')

    def _get_ready(self, req):
        '''
        Enter pick_n_place pose.
        @return, boolean, return true if successful, return false otherwise.
        '''

        # check whether it is already in ready state
        if self._ready:
            rospy.logwarn("fetch_pnp_service/get_ready: already ready")
            return True

        # Get ready to pick
        rospy.loginfo("fetch_pnp_service/get_ready")
        # dimension of _gripper link body, approx as rectangle
        self._clear_obstacles(all_obj=True)

        # 1. Raise torso
        self.robot.move_torso(req.torso_height)
        self._close_gripper()
        self.robot.move_head(pan=0, tilt=req.table_camera_tilt)
        rospy.sleep(1)  # wait for it to finish TODO

        # 2. Detect table
        res = self._set_table_as_obstacle()
        if not res:
            rospy.logerr("fetch_pnp_service/get_ready failed")
            return False

        # 3. Move to pick pose first
        res = self._move_to_ready_pick_pose()
        if not res:
            rospy.logerr("fetch_pnp_service/get_ready failed!!")
            return False

        # 4. Move closer to table if needed
        # !!!!This is dangerous if you do not detect table edge!!
        res = self._move_closer_to_table(self._table_dist)
        if not res:
            rospy.logerr("fetch_pnp_service/get_ready failed")
            return False

        # 5. Final head tilt
        self.robot.move_head(pan=0, tilt=req.final_camera_tilt)

        self._ready = True
        rospy.loginfo('fetch_pnp_service: get_ready succeeded')
        return True

    def _exit_ready_pose(self):
        '''
        Exit pick_n_place pose.
        @return, boolean, return true if successful, return false otherwise.
        '''

        # check whether it is in ready state
        if not self._ready:
            rospy.logwarn("SimplePnp/_exit_ready_pose: Not in ready state")

        rospy.loginfo("fetch_pnp_service/_exit_ready_pose")

        # # 1. Move away from table
        # if MOVE_CLOSER_TO_TABLE:
        #     res = self._move_away_from_table()
        #     if not res:
        #         rospy.logerr("fetch_pnp_service/_exit_ready_pose failed")
        #         raise RuntimeError("fetch_pnp_service/_exit_ready_pose failed")
        #         return False

        # # 2. Move arm to tuck pose
        # if ASK_BEFORE_EXECUTE:
        #     self.robot.arm_move_to_joint(home_joints, plan_only=True)
        #     execute = raw_input('move to ready pick pose, execute?')
        #     if execute != 'y':
        #         rospy.loginfo("SimplePnp: fail to move to ready pick pose")
        #         return False

        # res = self.robot.arm_move_to_joint(home_joints, plan_only=False)
        # if not res:
        #     rospy.loginfo("SimplePnp: fail to move to ready pick pose")
        #     return False

        # # 3. Move torso down
        # self.robot.move_head(pan=0, tilt=0)

        self._ready = False

        # delete motion planning obstacle
        self._scene.remove_world_object()

        return True

    def _set_table_as_obstacle(self):
        '''
        Add table to planning scene as an obstacle
        @return, boolean, return true if successful, return false otherwise.
        '''

        # call table segmentor to get table marker
        rospy.loginfo("Calling table segmentor")
        resp = self._table_segmentor_srv(0)
        table_msg = resp.marker
        rospy.loginfo("table_marker msg arrived: {}".format(table_msg))

        table_height = table_msg.pose.position.z + table_msg.scale.z / 2
        while table_height > 1.0:
            rospy.loginfo("Calling table segmentor again")
            resp = self._table_segmentor_srv(0)
            table_msg = resp.marker
            rospy.loginfo("table_marker msg arrived: {}".format(table_msg))
            table_height = table_msg.pose.position.z + table_msg.scale.z / 2

        # Add table as obstacle
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position = table_msg.pose.position
        box_pose.pose.orientation.w = 1.0
        ## Add buffer height for safe motion planning
        self._scene.add_box('table', box_pose, size=(table_msg.scale.x, table_msg.scale.y, table_msg.scale.z + MOTION_PLANNING_BUFFER_HEIGHT * 2))
        self._table_height = table_msg.pose.position.z + table_msg.scale.z / 2 # the original table height
        self._table_dist = table_msg.pose.position.x - table_msg.scale.x / 2
        rospy.loginfo("table_height: {}, table_dist {}".format(self._table_height, self._table_dist))

        # Add wall to the left
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = 0.8
        box_pose.pose.position.z = 1.0
        box_pose.pose.orientation.w = 1.0
        self._scene.add_box('left_wall', box_pose, size=(1, 0.2, 2.0))

        return True

    def _move_closer_to_table(self, table_dist):
        '''
        Move closer to table. Part of getting ready sequence
        @return, boolean, return true if successful, return false otherwise.
        '''

        if not MOVE_CLOSER_TO_TABLE:
            return True

        if table_dist < SAFE_TABLE_DISTANCE:
            rospy.loginfo("SimplePnp/_move_closer_to_table: already close to table, skip")
            return True

        self.robot.base_go_straight(table_dist - SAFE_TABLE_DISTANCE)
        return True

    def _move_away_from_table(self, dist=DIST_TO_MOVE_AWAY_FROM_TABLE):
        '''
        Move away from table. Part of exit_pnp_ready_pose sequence
        @return, boolean, return true if successful, return false otherwise.
        '''

        if not MOVE_CLOSER_TO_TABLE:
            return True

        self.robot.base_go_straight(-dist)
        return True

    def _clear_obstacles(self, all_obj=False):
        '''
        Remove collision objects
        @param all_obj, boolean, if true, removes table as well
        '''
        self._scene.remove_world_object()

    # def _move_arm_to_home(self):
    #     '''
    #     Move arm back to home
    #     @return, boolean, return true if successful, return false otherwise.
    #     '''

    #     rospy.loginfo("FetchPnpService/_move_arm_to_home")

    #     # here, home pose is the ready_pick_pose()
    #     return self._move_to_ready_pick_pose()

    def _move_to_ready_pick_pose(self):
        '''
        Move arm back to ready_pick pose
        @return, boolean, return true if successful, return false otherwise.
        '''

        # move arm to ready pick pose
        if ASK_BEFORE_EXECUTE:
            self.robot.arm_move_to_joint(hide_right_joints, plan_only=True, allowed_planning_time=3)
            #Need to expose allowed_planning_time
            execute = raw_input('move to ready pick pose, execute?')
            if execute != 'y':
                rospy.loginfo("SimplePnp: fail to move to ready pick pose")
                return False

        res = self.robot.arm_move_to_joint(hide_right_joints, plan_only=False, allowed_planning_time=3)
        if not res:
            rospy.loginfo("SimplePnp: fail to move to ready pick pose")
            return False

        return True

    # def _move_arm_to_pose(self, pose):
    #     if VIS_GRASP:
    #         self._visualize_grasp(pose)

    #     if ASK_BEFORE_EXECUTE:
    #         res = self.robot.arm_move_to_pose(pose, plan_only=True, allowed_planning_time=3)
    #         if not res:
    #             rospy.logerr("move_arm_to_pose failed")
    #             return False

    #         to_execute = raw_input('execute?')
    #         if to_execute != 'y':
    #             rospy.logerr("move_arm_to_pose failed")
    #             return False

    #     # move arm to pose
    #     res = self.robot.arm_move_to_pose(pose, plan_only=False, allowed_planning_time=3)
    #     if not res:
    #         rospy.logerr("_move_arm_to_pose failed 4")
    #         return False

    #     return res

    # def _move_arm_in_cartesian(self, dx=0, dy=0, dz=0):
    #     if ASK_BEFORE_EXECUTE:
    #         rospy.loginfo('move in x, y, z  by {} {} {}'.format(dx, dy, dz))
    #         to_execute = raw_input('ok?')
    #         if to_execute != 'y':
    #             rospy.logerr("_top_pick failed 5")
    #             return False

    #     res = self.robot.arm_move_in_cartesian(dx, dy, dz)
    #     if not res:
    #         rospy.logerr('move_arm_in_cartesian failed')
    #     return res

    # def _close_gripper(self):
    #     '''
    #     provider
    #     '''
    #     self.robot.gripper_position(position = self.robot.GRIPPER_CLOSED_POS) # close with minimum effort
    #     return True

    # def _open_gripper(self):
    #     '''
    #     provider
    #     '''
    #     self.robot.gripper_position(position = self.robot.GRIPPER_OPENED_POS)
    #     return True

    # def _set_gripper_opening(self, opening):
    #     if opening > self.robot.GRIPPER_OPENED_POS:
    #         opening = self.robot.GRIPPER_OPENED_POS
    #     self.robot.gripper_position(position=opening)
    #     return True

    # def _calculate_top_pick_pose(self, object3d):
    #     grasp_pose = PnpService._calculate_top_grasp_pose(self, object3d)

    #     # protect against hitting table
    #     if grasp_pose.pose.position.z <= self._table_height:
    #         rospy.loginfo("PnpService: Gripper might hit table! Setting z to {} from {}".format(self._table_height, grasp_pose.pose.position.z))
    #         grasp_pose.pose.position.z = self._table_height

    #     # add approach height
    #     grasp_pose.pose.position.z += self._top_grasp_approach_height + GRIPPER_LENGTH

    #     return grasp_pose, self._top_grasp_approach_height

    # def _calculate_top_place_pose(self, target_pose):
    #     # TODO frame transformation

    #     # add approach height
    #     grasp_pose = target_pose
    #     grasp_pose.header.frame_id = "base_link"
    #     if grasp_pose.pose.position.z <= self._table_height:
    #         rospy.loginfo("PnpService: Gripper might hit table! Setting z to {} from {}".format(self._table_height, grasp_pose.pose.position.z))
    #         grasp_pose.pose.position.z = self._table_height

    #     grasp_pose.pose.position.z += self._top_place_approach_dist + self.robot.GRIPPER_LENGTH
    #     quat = T.quaternion_from_euler(0, math.pi / 2, 0, 'rxyz') # rotate by y to make it facing downwards
    #     grasp_pose.pose.orientation.x = quat[0]
    #     grasp_pose.pose.orientation.y = quat[1]
    #     grasp_pose.pose.orientation.z = quat[2]
    #     grasp_pose.pose.orientation.w = quat[3]

    #     return grasp_pose, self._top_place_approach_dist

    def _visualize_grasp_finger(self, pose, gripper_opening=0, dx=0, dy=0, dz=0):
        if not VIS_GRASP:
            return

        br = tf2_ros.StaticTransformBroadcaster()

        left_finger_t = TransformStamped()
        left_finger_t.header.stamp = rospy.Time.now()
        left_finger_t.header.frame_id = 'grasp_pose'
        left_finger_t.child_frame_id = 'left_finger'
        left_finger_t.transform.translation.x = self.robot.GRIPPER_LENGTH - dz
        left_finger_t.transform.translation.y = -gripper_opening / 2 + dx
        left_finger_t.transform.translation.z = -dy
        left_finger_t.transform.rotation.w = 1.0
        br.sendTransform(left_finger_t)  # broadcast transform so can see in rviz

        right_finger_t = TransformStamped()
        right_finger_t.header.stamp = rospy.Time.now()
        right_finger_t.header.frame_id = 'grasp_pose'
        right_finger_t.child_frame_id = 'right_finger'
        right_finger_t.transform.translation.x = self.robot.GRIPPER_LENGTH - dz
        right_finger_t.transform.translation.y = gripper_opening / 2 + dx
        right_finger_t.transform.translation.z = -dy
        right_finger_t.transform.rotation.w = 1.0
        br.sendTransform(right_finger_t)  # broadcast transform so can see in rviz

if __name__ == "__main__":
    # init node
    rospy.init_node('fetch_pnp_service')

    try:
        pnp_server = FetchPnpService()
        # pnp_server.get_ready()
        rospy.spin()
        #pnp_server = PNPServer()
    except Exception as e:
        rospy.logerr(e)
        rospy.logerr("Failed to init pnp_server")