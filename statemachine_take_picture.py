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

    rospy.init_node('statemachine_take_picture')
    rospy.loginfo('Initialized statemachine for pictures')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_done', 'error'])
    sm.userdata.destination = "kitchen"
    
    with sm:
                       
        smach.StateMachine.add("TAKE_PICTURE",
                               TakePicture(),
                               transitions={"done": "sm_done",
                                            "error": "error"})
     
    int_server = IntrospectionServer("smach_statemachine", sm, "/SM_ROOT")
    int_server.start()

    outcome = sm.execute()
    rospy.spin()
    int_server.stop()


if __name__ == '__main__':
    main()
