#! /usr/bin/env python

'''
Company Name : NEWRRO TECH
Date         : 7/6/2024
ROS Version  : Melodic
Website      : www.newrro.in
e-mail       : info@newrro.in
Contacts     : 8660875098 , 8217629665 , 8722278769
Social media : @newrro_tech , @last_bench_robots
'''

import rospy
import speech_recognition as sr
from std_msgs.msg import String
import time

class SpeechRecognizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speech_recognition_node')
        
        # Create publisher for voice commands
        self.voice_pub = rospy.Publisher('/voice_command', String, queue_size=10)
        
        # Initialize speech recognition components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust recognizer settings
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 4000  # May need adjustment based on environment
        
        print("\n===========================================")
        print("Arjuna Speech Recognition System")
        print("===========================================")
        print("Publishers: /voice_command")
        print("")
        print("Say commands like:")
        print("  - 'move forward 1 meter'")
        print("  - 'move backward 0.5 meters'")
        print("  - 'turn left 90 degrees'")
        print("  - 'turn right 45 degrees'")
        print("  - 'turn left and move forward 1 meter'")
        print("  - 'stop'")
        print("===========================================")
        
        # Adjust for ambient noise
        self.adjust_for_noise()
    
    def adjust_for_noise(self):
        """Calibrate for ambient noise levels"""
        print("\nCalibrating for ambient noise... Please remain quiet.")
        with self.microphone as source:
            try:
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                print("Calibration complete. Ready to listen.")
            except Exception as e:
                print("Error during calibration: {}".format(e))
    
    def listen_continuous(self):
        """Continuously listen for voice commands and publish them"""
        print("\nListening for commands... (Press Ctrl+C to exit)")
        
        while not rospy.is_shutdown():
            with self.microphone as source:
                # Adjust for ambient noise periodically
                if rospy.Time.now().to_sec() % 30 < 0.1:  # Every ~30 seconds
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                
                try:
                    print("Listening...")
                    audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=5.0)
                    
                    try:
                        # Convert speech to text using Google Speech Recognition
                        text = self.recognizer.recognize_google(audio)
                        print("Recognized: " + text)
                        
                        # Publish the recognized command
                        msg = String()
                        msg.data = text
                        self.voice_pub.publish(msg)
                        
                        # Small delay after publishing
                        time.sleep(0.5)
                        
                    except sr.UnknownValueError:
                        print("Could not understand audio")
                    except sr.RequestError as e:
                        print("Error requesting results from Google Speech Recognition service: {}".format(e))
                        
                except sr.WaitTimeoutError:
                    # Timeout occurred, continue listening
                    pass
                except Exception as e:
                    print("Error during listening: {}".format(e))
    
    def test_mode(self):
        """Test mode using terminal input instead of microphone"""
        print("\nTEST MODE ENABLED: Type commands in terminal")
        print("Enter 'quit' or 'exit' to stop")
        
        while not rospy.is_shutdown():
            try:
                command = raw_input("\nEnter command: ")
                if command.lower() in ['quit', 'exit']:
                    break
                    
                if command:
                    # Publish the command
                    msg = String()
                    msg.data = command
                    self.voice_pub.publish(msg)
                    print("Published: " + command)
            except KeyboardInterrupt:
                break

def main():
    try:
        # Create speech recognizer
        speech_recognizer = SpeechRecognizer()
        
        # Check if test mode is enabled
        test_mode = rospy.get_param('~test_mode', False)
        
        if test_mode:
            speech_recognizer.test_mode()
        else:
            # Check if microphone is working
            try:
                with speech_recognizer.microphone as source:
                    audio = speech_recognizer.recognizer.listen(source, timeout=1.0)
                print("Microphone test successful")
                
                # Start continuous listening
                speech_recognizer.listen_continuous()
                
            except Exception as e:
                print("ERROR: Could not access microphone: {}".format(e))
                print("Switching to test mode...")
                speech_recognizer.test_mode()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("ERROR: {}".format(e))
        print("Make sure you have installed the required packages:")
        print("  - pip install SpeechRecognition")
        print("  - pip install pyaudio")
        print("  - sudo apt-get install python-pyaudio python3-pyaudio")

if __name__ == '__main__':
    main()
