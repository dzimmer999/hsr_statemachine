#!/usr/bin/python3

import smach
import rospy
from find_marker import FindMarker
from calc_position import CalculatePosition
from drive_to_destination import DriveToDestination
from take_picture import TakePicture
from smach_ros import IntrospectionServer

def main():
    """Statemachine containing of these states:
        FIND_MARKER: Turn around until you find a marker.
        CALCULATE_POSITION: Calculate the robots position and align rviz based on the found marker
        DRIVE_TO_DESTINATION: Robot drives to a given destination
        TAKE_PICTURE: Robot takes three pictures
    """
    rospy.init_node('statemachine')
    rospy.loginfo('Initialized statemachine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_done'])
    sm.userdata.marker_id = None
    sm.userdata.error_count = 0
    sm.userdata.destination = None

    with sm:

        smach.StateMachine.add("FIND_MARKER",
                               FindMarker(),
                               transitions={"found": "CALCULATE_POSITION",
                                            "not_found": "FIND_MARKER"})

        smach.StateMachine.add("CALCULATE_POSITION",
                                CalculatePosition(),
                                transitions={"calculated" : "DRIVE_TO_DESTINATION",
                                             "calc_error" : "FIND_MARKER"},
                                remapping={"marker_id" : "marker_id"})

        smach.StateMachine.add("DRIVE_TO_DESTINATION",
                               DriveToDestination(),
                               transitions={"arrived_in_kitchen": "TAKE_PICTURE",
                                            "arrived_at_map" : "sm_done",
                                            "retry": "DRIVE_TO_DESTINATION",
                                            "error": "FIND_MARKER"},
                               remapping={"error_count" : "error_count"})
                                        
        smach.StateMachine.add("TAKE_PICTURE",
                               TakePicture(),
                               transitions={"done": "DRIVE_TO_DESTINATION",
                                            "error": "FIND_MARKER"})
     

    int_server = IntrospectionServer("smach_statemachine", sm, "/SM_ROOT")
    int_server.start()

    outcome = sm.execute()
    rospy.spin()
    int_server.stop()


if __name__ == '__main__':
    main()
