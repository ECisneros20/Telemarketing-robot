from transformers import AutoProcessor, BarkModel
import matplotlib.pyplot as plt
import sounddevice as sd
import soundfile as sf
from time import time
import pandas as pd
import torch
import scipy
# Link: https://suno-ai.notion.site/8b8e8749ed514b0cbf3f699013548683?v=bc67cff786b04b50b3ceb756fd05f68c

processor = AutoProcessor.from_pretrained("suno/bark-small")
model = BarkModel.from_pretrained("suno/bark-small")
device = "cuda:0" if torch.cuda.is_available() else "cpu"
print(device)
model = model.to(device)
number_presets = 10
number_test_for_preset = 10
x_axis = [str(i) for i in range(number_test_for_preset)]
x_axis.append("Avg")

while (1):

    if device == "cpu":

        print("GPU not available")
        break

    text_prompt = input("Write the message to be said: ")

    if text_prompt == "q":

        break

    list_metrics = {}

    # Initialise the subplot function using number of rows and columns
    figure, axes = plt.subplots(2, 5, figsize = (15, 12))

    for i in range(number_presets):

        list_metrics[i] = []

        for j in range(number_test_for_preset):

            voice_preset = f"v2/es_speaker_{i}"
            start = time()

            inputs = processor(text = text_prompt, voice_preset = voice_preset)
            speech_output = model.generate(**inputs.to(device))
            sampling_rate = model.generation_config.sample_rate
            filename = f"./telemarketing_gui/src/test/bark_out_{i}_{j}.wav"
            scipy.io.wavfile.write(filename, rate = sampling_rate, data = speech_output[0].cpu().numpy())

            final = time()
            print(f"Test {i}_{j} time: {final-start}")
            list_metrics[i].append(final - start)

        list_metrics[i].append(sum(list_metrics[i]) / len(list_metrics[i]))

        data, fs = sf.read(filename, dtype = "float32")
        sd.play(data,fs)
        status = sd.wait()

        ax = axes[int(i/5), i%5]
        ax.bar(x_axis, list_metrics[i])
        ax.set_xlabel("Tests")
        ax.set_ylabel("Time spent (s)")
        ax.set_title(f"Test for the preset #{i}")

    # Combine all the operations and display
    plt.show()

    df = pd.DataFrame.from_dict(list_metrics)
    df.to_csv("./telemarketing_gui/src/test/metrics_test_bark.csv")
