#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np

def q_to_R(q):
    return np.array(tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3,:3])

def vec(msg):  # geometry_msgs/Vector3 -> np.array
    return np.array([msg.x, msg.y, msg.z])

class OdomFromTF:
    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.child_frame  = rospy.get_param("~child_frame",  "base_link")
        self.body_frame   = rospy.get_param("~body_frame",   "body")
        self.src_odom_topic = rospy.get_param("~src_odom", "/Odometry")
        self.rate_hz = rospy.get_param("~rate", 30.0)

        self.buf   = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfsub = tf2_ros.TransformListener(self.buf)
        self.pub   = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=20)

        self.last_body_twist = None
        rospy.Subscriber(self.src_odom_topic, Odometry, self.body_odom_cb, queue_size=10)

    def body_odom_cb(self, msg):
        # 假设 msg.child_frame_id == self.body_frame，且 twist 是在 body 坐标下（常见做法）
        self.last_body_twist = msg

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                # 1) 取 pose: odom -> base_link
                ts_obl = self.buf.lookup_transform(self.parent_frame, self.child_frame,
                                                   rospy.Time(0), rospy.Duration(0.2))
                # 2) 取静态外参: body -> base_link（用于 twist 变换）
                ts_b_bl = self.buf.lookup_transform(self.body_frame, self.child_frame,
                                                    rospy.Time(0), rospy.Duration(0.2))
                self.publish(ts_obl, ts_b_bl)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "TF lookup failed: %s" % e)
            r.sleep()

    def publish(self, ts_obl, ts_b_bl):
        odom = Odometry()
        stamp = ts_obl.header.stamp if ts_obl.header.stamp != rospy.Time() else rospy.Time.now()
        odom.header.stamp = stamp
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id  = self.child_frame

        # pose
        odom.pose.pose.position = ts_obl.transform.translation
        odom.pose.pose.orientation = ts_obl.transform.rotation

        # twist: 把 body 的 twist 映射到 base_link
        if self.last_body_twist is not None:
            v_b = vec(self.last_body_twist.twist.twist.linear)
            w_b = vec(self.last_body_twist.twist.twist.angular)

            # 外参：body -> base_link
            r_b_bl = np.array([ts_b_bl.transform.translation.x,
                               ts_b_bl.transform.translation.y,
                               ts_b_bl.transform.translation.z])
            R_b_bl = q_to_R(ts_b_bl.transform.rotation)

            w_bl = R_b_bl.dot(w_b)
            v_bl = R_b_bl.dot(v_b + np.cross(w_b, r_b_bl))

            odom.twist.twist.linear.x,  odom.twist.twist.linear.y,  odom.twist.twist.linear.z  = v_bl.tolist()
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = w_bl.tolist()

            #（可选）拷贝协方差：把 body 的协方差直接用作近似
            odom.twist.covariance = list(self.last_body_twist.twist.covariance)
        else:
            odom.twist.covariance = [0.0]*36

        self.pub.publish(odom)

if __name__ == "__main__":
    rospy.init_node("odom_from_tf")
    OdomFromTF().spin()
