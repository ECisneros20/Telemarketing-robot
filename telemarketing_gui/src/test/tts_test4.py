from fairseq.checkpoint_utils import load_model_ensemble_and_task_from_hf_hub
from fairseq.models.text_to_speech.hub_interface import TTSHubInterface
import sounddevice as sd
import soundfile as sf
import scipy

models, cfg, task = load_model_ensemble_and_task_from_hf_hub(
    "facebook/tts_transformer-es-css10",
    arg_overrides={"vocoder": "hifigan", "fp16": False}
)

model = models[0]
TTSHubInterface.update_cfg_with_data_cfg(cfg, task.data_cfg)
generator = task.build_generator([model], cfg)

text = "Hola, los invito a todos al Festival de Cine de Áncash. En una hora se realizará la inauguración, la cual podrá ver por un en vivo en Facebook e Instagram"

sample = TTSHubInterface.get_model_input(task, text)
wav, rate = TTSHubInterface.get_prediction(task, model, generator, sample)

filename = f"./telemarketing_gui/src/test/test_facebook.wav"
scipy.io.wavfile.write(filename, rate = rate, data = wav)
data, fs = sf.read(filename, dtype = "float32")
sd.play(data,fs)
status = sd.wait()
