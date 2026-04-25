#!/usr/bin/env python3
import rospy


if __name__ == "__main__":
    rospy.init_node("mission_ui")
    rospy.loginfo("图形控制后台已就绪，可通过 mission_ui 插件进行交互操作。")
    rospy.spin()
