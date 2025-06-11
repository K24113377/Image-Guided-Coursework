#!/usr/bin/env python3

import rclpy
from my_pose_goal.member_subscriber import MemberSubscriber
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()

    # Start robot interface
    my_robot = MoveItPy(node_name="moveit_py")
    my_planning_group = my_robot.get_planning_component("my_planning_group")
    my_planning_group.set_start_state(configuration_name="home")

    # Start subscriber node
    subscriber = MemberSubscriber()

    # Wait until a pose is received
    while rclpy.ok() and subscriber.latest_pose is None:
        rclpy.spin_once(subscriber, timeout_sec=0.1)

    pose_goal = subscriber.latest_pose
    print("Using received pose...")

    # Plan with received pose
    my_planning_group.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link="link_7"  # Replace with your actual end-effector link
    )

    plan_result = my_planning_group.plan()

    if plan_result:
        print("Plan was successful")
        my_robot.execute(plan_result.trajectory, controllers=[])
        print("Execution done")
    else:
        print("Plan was not successful")

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
