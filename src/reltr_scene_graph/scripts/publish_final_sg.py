#!/usr/bin/env python3
import rospy, json, os
from std_msgs.msg import String

def main():
    rospy.init_node("final_sg_pub")
    path = rospy.get_param("~path",
            os.path.expanduser("~/.ros/scene_graph_results/final_sg.json"))
    topic = rospy.get_param("~topic", "/final_scene_graph/json")

    pub = rospy.Publisher(topic, String, queue_size=1, latch=True)

    with open(path, "r") as f:
        txt = f.read()
    pub.publish(String(txt))
    rospy.loginfo("Latched %s from %s", topic, path)

    rospy.spin()  # 계속 살아있기
if __name__ == "__main__":
    main()
