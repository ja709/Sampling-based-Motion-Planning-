#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy



    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            
                blockName = "piano2"
                resp_coordinates = model_coordinates(blockName)
                print '\n'
                print 'Status.success = ', resp_coordinates.success
                print(blockName)
                print("Cube " + str(blockName))
                print("Valeur de X : " + str(resp_coordinates.pose.position.x))
                print("Quaternion X : " + str(resp_coordinates.pose.orientation.x))

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    
    show_gazebo_models()
