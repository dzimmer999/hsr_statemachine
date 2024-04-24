#!/usr/bin/python3

import smach
import rospy
from find_marker import FindMarker
from calc_position import CalculatePosition
from smach_ros import IntrospectionServer

def main():
    """Statemachine for aligining the map
        FIND_MARKER: Turn around until you find a marker.
        CALCULATE_POSITION: Calculate the robots position and align rviz based on the found marker
    """

    rospy.init_node('statemachine_localize')
    rospy.loginfo('Initialized statemachine for localization')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_done', 'error'])
    sm.userdata.marker_id = None

    with sm:

        smach.StateMachine.add("FIND_MARKER",
                               FindMarker(),
                               transitions={"found": "CALCULATE_POSITION",
                                            "not_found": "FIND_MARKER"},
                               remapping={'loop_count_out': 'loop_count_in'})

        smach.StateMachine.add("CALCULATE_POSITION",
                                CalculatePosition(),
                                transitions={"calculated" : "sm_done",
                                             "calc_error" : "FIND_MARKER"},
                                remapping={"marker_id" : "marker_id"})
     

    int_server = IntrospectionServer("smach_statemachine", sm, "/SM_ROOT")
    int_server.start()

    outcome = sm.execute()
    rospy.spin()
    int_server.stop()


if __name__ == '__main__':
    main()
