#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class ListeningNode:
    def __init__(self):
        rospy.init_node('listening_node', anonymous=True)

        # Subscriber to VOSK final_result topic
        rospy.Subscriber('/speech_recognition/final_result', String, self.speech_callback)

        # Publisher to task_command topic
        self.task_pub = rospy.Publisher('/task_command', String, queue_size=10)

        rospy.loginfo("Listening Node Initialized, awaiting commands...")

        # Define known actions and example objects
        self.actions = ['fetch', 'take']
        self.known_objects = ['apple', 'bottle', 'book', 'cup', 'phone', 'bag'] # extend as needed

    def speech_callback(self, msg):
        text = msg.data.lower().strip()
        rospy.loginfo(f"Voice Command Received: '{text}'")

        detected_action = None
        detected_object = None

        # Detect the action
        for action in self.actions:
            if action in text:
                detected_action = action
                break

        # Detect the object
        for obj in self.known_objects:
            if obj in text:
                detected_object = obj
                break

        # Publish only if both action and object are clearly detected
        if detected_action and detected_object:
            command_str = f"{detected_action} {detected_object}"
            rospy.loginfo(f"Detected Task: {command_str}")

            # Publish to /task_command topic
            self.task_pub.publish(command_str)
        else:
            rospy.logwarn("Unable to detect valid action and/or object. Please repeat clearly.")

if __name__ == '__main__':
    try:
        node = ListeningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
