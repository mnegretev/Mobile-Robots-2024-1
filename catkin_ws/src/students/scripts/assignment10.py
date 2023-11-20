#!/usr/bin/env python3
#
# MOBILE ROBOTS - UNAM, FI, 2024-1
# ASSIGNMENT 10 - SPEECH SYNTHESIS
#

import sys
import rospy
from sound_play.msg import SoundRequest

def main(text_to_say):
    print("INITIALIZING SPEECH SYNTHESIS TEST...")
    rospy.init_node("speech_syn")
    pub_speech = rospy.Publisher("/robotsound", SoundRequest, queue_size=10)
    loop = rospy.Rate(2)

    msg_speech = SoundRequest()
    msg_speech.sound   = -3
    msg_speech.command = 1
    msg_speech.volume  = 1.0
    #
    # EJERCICIO
    # Cambie la voz por alguna de las voces instaladas
    #
    msg_speech.arg2    = "voice_us1_mbrola"
    msg_speech.arg = text_to_say

    loop.sleep()
    print("Sending text to say: " + text_to_say)
    pub_speech.publish(msg_speech)
   
if __name__ == "__main__":
    text_to_say = Jose Carlos""
    if len(sys.argv) > 1:
        text_to_say = sys.argv[1]
    main(text_to_say)
