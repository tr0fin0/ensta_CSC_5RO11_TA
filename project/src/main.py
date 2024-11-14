# import sys
# print "\n".join(sys.path)

import naoqi



class PepperAPI:
    def __init__(self, ip, port):
        self.session = naoqi.ALProxy("ALTextToSpeech", ip, port)
        self.session.say("Hello, I'm Pepper! Ready to play the game.")

    def speak(self, text):
        self.session.say(text)



def main():
    pepper = PepperAPI('192.168.2.133', 9559)
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")
    pepper.speak("I'm a Barbie Girl!")



if __name__ == '__main__':
    main()
