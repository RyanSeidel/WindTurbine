import threading
import subprocess
import rospy
from std_srvs.srv import Empty

class DigitalTwinGUI(QMainWindow):
    # Add the Gazebo control functions inside your GUI class
    
    def start_gazebo(self):
        # Launch Gazebo in a separate thread so it doesn't block the GUI
        thread = threading.Thread(target=self.launch_gazebo_process)
        thread.start()

    def launch_gazebo_process(self):
        print("Starting Gazebo...")
        subprocess.Popen(['roslaunch', 'gazebo_ros', 'empty_world.launch'])

    def reset_gazebo(self):
        thread = threading.Thread(target=self.reset_gazebo_world)
        thread.start()

    def reset_gazebo_world(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            print("Gazebo world reset.")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    
    def pause_gazebo(self):
        thread = threading.Thread(target=self.pause_gazebo_physics)
        thread.start()

    def pause_gazebo_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            pause_sim()
            print("Gazebo simulation paused.")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def unpause_gazebo(self):
        thread = threading.Thread(target=self.unpause_gazebo_physics)
        thread.start()

    def unpause_gazebo_physics(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause_sim()
            print("Gazebo simulation unpaused.")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
