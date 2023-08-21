#!/usr/bin/env python3

import rospy
from telemarketing_gui.srv import GenerateAudio, GenerateAudioResponse

from transformers import AutoProcessor, BarkModel
import sounddevice as sd
import soundfile as sf
import torch
import scipy
# Link: https://www.youtube.com/watch?v=6RrcQFmNO58
# Link: https://suno-ai.notion.site/8b8e8749ed514b0cbf3f699013548683?v=bc67cff786b04b50b3ceb756fd05f68c

# Server (1)
# txt_prompt     -   String     -   to Jetson Xavier     -   String with the message to be said in audio via tts

class GenerateAudioServer:

    def __init__(self):

        # TTS setup
        self.processor = AutoProcessor.from_pretrained("suno/bark-small")
        self.model = BarkModel.from_pretrained("suno/bark-small")
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"The device is: {self.device}")
        self.model = self.model.to(self.device)
        self.voice_preset = "v2/es_speaker_4"
        self.sampling_rate = self.model.generation_config.sample_rate

    def handle_generate_audio(self, req):

        try:

            # Get the audio file
            inputs = self.processor(text = req.text_prompt, voice_preset = self.voice_preset)
            speech_output = self.model.generate(**inputs.to(self.device))
            filename = f"./telemarketing_gui/scripts/audio.wav"
            scipy.io.wavfile.write(filename, rate = self.sampling_rate, data = speech_output[0].cpu().numpy())

            # Save and play the audio file
            data, fs = sf.read(filename, dtype = "float32")
            sd.play(data,fs)
            status = sd.wait()
            req.state = "done"

        except:

            req.state = "failed"

        return GenerateAudioResponse(req.state)

    def generate_audio_server(self):

        # ROS setup
        rospy.init_node("generate_audio_server")
        s = rospy.Service("generate_audio", GenerateAudio, self.handle_generate_audio)
        rospy.loginfo("The audio is done!")
        rospy.spin()


if __name__ == "__main__":

    try:
        # Server initialization
        srv = GenerateAudioServer()
        srv.generate_audio_server()
    
    except:
        pass
