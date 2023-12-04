

import time
import speech_recognition as sr
from gtts import gTTS
import os


def listen():
    # Initialize the recognizer
    r = sr.Recognizer()

    # Capture audio from the microphone
    with sr.Microphone() as source:
        print("Please say your question:")
        audio = r.listen(source)

        try:
            # Recognize speech using Google Speech Recognition
            user_question = r.recognize_google(audio)
            print(f"You said: {user_question}")
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
            user_question = ""
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")
            user_question = None

    print(user_question)

    return user_question

listen()
