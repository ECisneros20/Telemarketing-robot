from transformers import AutoProcessor, BarkModel
import sounddevice as sd
import soundfile as sf
import torch
import scipy
# Link: https://suno-ai.notion.site/8b8e8749ed514b0cbf3f699013548683?v=bc67cff786b04b50b3ceb756fd05f68c

import pyttsx3

from gtts import gTTS
# Link: https://thepythoncode.com/article/convert-text-to-speech-in-python?es_id=bd101f3ef4#google_vignette


processor = AutoProcessor.from_pretrained("suno/bark-small")
model = BarkModel.from_pretrained("suno/bark-small")
device = "cuda:0" if torch.cuda.is_available() else "cpu"
print(device)
model = model.to(device)
number_presets = 10

tld = ["com.mx", "es", "us"]

while (1):

    if device == "cpu":

        print("GPU not available")
        break

    text_prompt = input("Write the message to be said: ")

    if text_prompt == "q":

        break

    if text_prompt == "r":
    
        text_prompt = "Los invitamos a conocer más sobre el 'Festival de Cine de Áncash', una iniciativa de difusión de cine descentralizada y liderada por jóvenes peruanos. En su tercera edición, este festival tiene como temática central el ODS 6: 'Agua limpia y saneamiento'. Los esperamos del 21 al 26 de agosto en Huaraz"

    for i in range(number_presets):

        voice_preset = f"v2/es_speaker_{i}"

        inputs = processor(text = text_prompt, voice_preset = voice_preset)
        speech_output = model.generate(**inputs.to(device))
        sampling_rate = model.generation_config.sample_rate
        filename = f"./telemarketing_gui/src/test/festival-msg-bark-{i}.wav"
        scipy.io.wavfile.write(filename, rate = sampling_rate, data = speech_output[0].cpu().numpy())

        data, fs = sf.read(filename, dtype = "float32")
        sd.play(data,fs)
        status = sd.wait()

    engine = pyttsx3.init()
    engine.setProperty("rate", 100)
    # print(engine.getProperty("volume"))
    engine.setProperty("volume", 0.6)
    voices = engine.getProperty("voices")
    filename = f"./telemarketing_gui/src/test/festival-msg-pytts-{voices[19].id}.wav"
    engine.setProperty("voice", voices[19].id)
    engine.say(text_prompt)
    engine.runAndWait()
    engine.save_to_file(text_prompt, filename)
    filename = f"./telemarketing_gui/src/test/festival-msg-pytts-{voices[20].id}.wav"
    engine.setProperty("voice", voices[20].id)
    engine.say(text_prompt)
    engine.runAndWait()
    engine.save_to_file(text_prompt, filename)

    for j in tld:

        tts = gTTS(text_prompt, lang = "es", tld = j)
        filename = f"./telemarketing_gui/src/test/festival-msg-gtts-{j}.wav"
        tts.save(filename)
        data, fs = sf.read(filename, dtype = "float32")
        sd.play(data,fs)
        status = sd.wait()
