#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from RandomWaypoints import *
from Rotate_Z import *

# define state Start
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Init'])

    def execute(self, userdata):
        rospy.loginfo('State Machine wird gestartet')
        rospy.sleep(5)
        return 'Init'

# define state RandomRoutine
class RandomWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Searching_Person','Failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Random Waypoints')
        if mainWP() is True:
            return 'Searching_Person'
        else:
            return 'Failed'

# define state SearchPerson
class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Person','No_Person'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Search Person')
        rospy.loginfo('Person wird gesucht')

        if mainRotation() is True:
            return 'Found_Person'
        else:
            return 'No_Person'
        # return 'Found_Person'

# define state FollowPerson
class FollowPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reached_Person','Person_not_interessed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow Person')
        rospy.sleep(5)
        return 'Reached_Person'

        # return 'Person_not_interessed'

# define state SafeState
class SafeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Waiting','LowBattery'])
        # status_battery = rospy.Subscriber('/diagnostics/battery', queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Safe Stafe')
        # print(status_battery)
        rospy.sleep(5)

        # if status_battery < 11:
        #     return 'LowBattery'
        # else:
            # return 'Waiting'
        rospy.loginfo('Akkustand ist niedrig')
        return 'LowBattery'
        #return 'Waiting'

# define state PersonPerson
class ReachedPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Person_not_interessed','Presentationmode'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Person Reached')
        return 'Person_not_interessed'

        # return 'Presentationmode'

# define state Presentation
class Presentation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Presentation_finished','Presentation_aborted'])

    def execute(self, userdata):
        rospy.loginfo('Präsentationsmodus ist aktiv')
        rospy.sleep(5)
        return 'Presentation_finished'

        # return 'Presentation_aborted'

# define End
class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['End'])

    def execute(self, userdata):
        rospy.loginfo('State Machine wird beendet')
        rospy.sleep(5)
        return 'End'

# main
def main():
    rospy.init_node('StateMachineTB3')
    turtlebot3_model = rospy.get_param("model", "burger")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Ende'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Start', Start(), 
                               transitions={'Init':'RandomWaypoints'})

        smach.StateMachine.add('RandomWaypoints', RandomWaypoints(), 
                               transitions={'Searching_Person':'SearchPerson',
                                            'Failed':'SafeState'})

        smach.StateMachine.add('SearchPerson', SearchPerson(), 
                               transitions={'Found_Person':'FollowPerson',
                                            'No_Person':'RandomWaypoints'})

        smach.StateMachine.add('FollowPerson', FollowPerson(), 
                               transitions={'Reached_Person':'ReachedPerson',
                                            'Person_not_interessed':'SafeState'})

        smach.StateMachine.add('SafeState', SafeState(), 
                               transitions={'Waiting':'RandomWaypoints',
                                            'LowBattery':'End'})

        smach.StateMachine.add('ReachedPerson', ReachedPerson(), 
                               transitions={'Person_not_interessed':'SafeState',
                                            'Presentationmode':'Presentation'})
        
        smach.StateMachine.add('Presentation', Presentation(),
                            transitions={'Presentation_finished':'SafeState',
                                         'Presentation_aborted':'SafeState'})

        smach.StateMachine.add('End', End(),
                            transitions={'End':'Ende'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('TB3_Service', sm, '/TB3')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()