import pyttsx3

engine = pyttsx3.init()
engine.setProperty("rate", 100)
print(engine.getProperty("volume"))
engine.setProperty("volume", 0.6)
voices = engine.getProperty("voices")
engine.setProperty("voice", voices[19].id)
engine.say("Diré este texto en español España")
engine.setProperty("voice", voices[20].id)
engine.say("Diré este texto en español Latinoamérica")
engine.runAndWait()
