import subprocess
import rospy
from std_srvs.srv import Empty

def start_gazebo():
    # Launch Gazebo
    subprocess.Popen(['roslaunch', 'gazebo_ros', 'empty_world.launch'])

def pause_gazebo():
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_sim()
        print("Gazebo simulation paused.")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def unpause_gazebo():
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_sim()
        print("Gazebo simulation unpaused.")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def reset_gazebo():
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
        print("Gazebo world reset.")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
