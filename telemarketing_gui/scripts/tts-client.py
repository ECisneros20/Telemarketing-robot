#!/usr/bin/env python3

import sys
import rospy
from telemarketing_gui.srv import *

# Client (1)
# state     -   String     -   From PC     -   String with the state of the audio ("done" is when the audio is finished)

class GenerateAudioClient:

    def generate_audio_client(text_prompt):

        rospy.wait_for_service("generate_audio")

        try:
            generate_audio = rospy.ServiceProxy("generate_audio", GenerateAudio)
            resp1 = generate_audio(text_prompt)
            return resp1.state

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def usage():

        return f"{sys.argv[0]} [text_prompt]"


if __name__ == "__main__":

    try:
        # Client initialization
        client = GenerateAudioClient()

        if len(sys.argv) == 2:
            text_prompt = sys.argv[1]
        else:
            print(client.usage())
            sys.exit(1)

        rospy.loginfo("Requesting audio generation")
        rospy.loginfo(f"State: {client.generate_audio_client(text_prompt)}")

    except:
        pass
