#text to speech
from sound_play.libsoundplay import SoundClient


import time

print("initialize")
soundhandle = SoundClient()


def say_this(words):
    s1 = soundhandle.voiceSound(words)
    s1.play()

