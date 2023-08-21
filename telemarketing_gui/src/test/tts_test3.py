from gtts import gTTS
import sounddevice as sd
import soundfile as sf
from time import time
# Link: https://thepythoncode.com/article/convert-text-to-speech-in-python?es_id=bd101f3ef4#google_vignette

tld = ["com.mx", "es", "us"]

for i in tld:

    start = time()
    text_prompt = "Hola, los invito a todos al Festival de Cine de Áncash. En una hora se realizará la inauguración, la cual podrá ver por un en vivo en Facebook e Instagram"
    tts = gTTS(text_prompt, lang = "es", tld = i)
    print(time() - start)
    filename = f"./telemarketing_gui/src/test/test_gtts_{i}"
    tts.save(filename)
    print(time() - start)
    data, fs = sf.read(filename, dtype = "float32")
    sd.play(data,fs)
    status = sd.wait()
