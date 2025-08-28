#!/usr/bin/env python3
import rospy
import subprocess
import rospkg
from std_msgs.msg import String

def get_gemini_api_path(file_name):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('gemini_API')
    return f"{pkg_path}/src/{file_name}"

def reltr_mode_callback(msg):
    if msg.data == "fin":
        rospy.loginfo("Received 'fin' message. Starting scripts.")
        
        # Run collect_images.py
        try:
            collect_images_script = get_gemini_api_path("collect_images_per_node.py")
            rospy.loginfo(f"Running {collect_images_script}")
            subprocess.run(["python3", collect_images_script], check=True)
            rospy.loginfo("collect_images_per_node.py finished.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error running collect_images_per_node.py: {e}")
            return
        except FileNotFoundError:
            rospy.logerr(f"collect_images_per_node.py not found.")
            return

        # Run answer_numerical.py
        try:
            answer_numerical_script = get_gemini_api_path("answer_numerical.py")
            rospy.loginfo(f"Running {answer_numerical_script}")
            subprocess.run(["python3", answer_numerical_script], check=True)
            rospy.loginfo("answer_numerical.py finished.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error running answer_numerical.py: {e}")
        except FileNotFoundError:
            rospy.logerr(f"answer_numerical.py not found.")


def main():
    rospy.init_node('gemini_orchestrator')
    rospy.Subscriber('/reltr_mode', String, reltr_mode_callback)
    rospy.loginfo("gemini_orchestrator node started, waiting for /reltr_mode message.")
    rospy.spin()

if __name__ == '__main__':
    main()