#!/usr/bin/python3

import smach
import rospy
from drive_to_destination import DriveToDestination
from take_picture import TakePicture
from smach_ros import IntrospectionServer

def main():
    """Statemachine without the localization (rviz has to be aligned)
        DRIVE_TO_DESTINATION: Robot drives to a given destination
        TAKE_PICTURE: Robot takes three pictures
    """

    rospy.init_node('statemachine_wo_loc')
    rospy.loginfo('Initialized statemachine without localization')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_done', 'error'])
    sm.userdata.error_count = 0
    sm.userdata.destination = "kitchen"
    
    with sm:

        smach.StateMachine.add("DRIVE_TO_DESTINATION",
                               DriveToDestination(),
                               transitions={"arrived_in_kitchen": "TAKE_PICTURE",
                                            "arrived_at_map" : "sm_done",
                                            "retry": "DRIVE_TO_DESTINATION",
                                            "error": "error"},
                               remapping={"error_count" : "error_count"})
                                        
        smach.StateMachine.add("TAKE_PICTURE",
                               TakePicture(),
                               transitions={"done": "DRIVE_TO_DESTINATION",
                                            "error": "error"})
     
    int_server = IntrospectionServer("smach_statemachine", sm, "/SM_ROOT")
    int_server.start()

    outcome = sm.execute()
    rospy.spin()
    int_server.stop()


if __name__ == '__main__':
    main()
