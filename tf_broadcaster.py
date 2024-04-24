#!/usr/bin/env python3
# Made by Elizabeth F.
# Adjusted

import roslib
import rospy
import tf
 
if __name__ == '__main__':
    """Broadcaster for the ar_markers in order to know the transformation from the marker to the map origin
       (Source Elisabeth F, adjusted to the new map)
    """
    
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-1.100, -0.55666, 0.41058),
                          (-0.0129, 0.92591, -0.3777, 0.00639),
                          rospy.Time.now(),
                           "ar_marker/15fix",
                           "map")
        br.sendTransform((0.517, -0.967, 0.819),
                         (-0.01075, 0.7118, -0.702, 0.01994),
                         rospy.Time.now(),
                          "ar_marker/17fix",
                          "map")
        # br.sendTransform((2.55, 0.61, 0.754),
        #                  (0.495, 0.51, -0.505, -0.49),
        #                   rospy.Time.now(),
        #                   "ar_marker/18fix",
        #                   "map")
        # br.sendTransform((4.266, 1.77, 0.588),
        #                  (-0.717, 0.008, 0.0077, 0.69),
        #                  rospy.Time.now(),
        #                   "ar_marker/19fix",
        #                   "map")
        br.sendTransform((-1.6413, -0.0723, 0.80751),
                          (0.4989, -0.49854, 0.5005, -0.50194),
                          rospy.Time.now(),
                           "ar_marker/20fix",
                           "map")
        rate.sleep()
