# Actionlib messages
import rospy
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsGoal
from lasr_speech.msg import informationGoal
from datetime import datetime
import os
import subprocess

def detectKeyword(self, action, expected):
    result = None
    
    while not result == expected:
        rospy.sleep(1)
        self.keyword_client.wait_for_server()
        self.keyword_client.send_goal(informationGoal('speech', action))
        self.keyword_client.wait_for_result()
        result = self.keyword_client.get_result().data
        print result

def planWakeWord(self, data):
    pass

def keywordCallback(self, data):
    pass

def keywordDetected(self, keyword):
    pass

def talk(self, text, wait=True):
    print('\033[1;36mTIAGO: ' + text + '\033[0m')
    tts_proc = subprocess.Popen(['echo "{}" | festival --tts'.format(text)], shell=True)
    if wait:
        tts_proc.wait()

def logText(self):
    file_path = os.path.dirname(os.path.abspath(__file__)) + '/logfile.txt'
    with open(file_path, "a") as log_file:
        log_file.write('timestamp:\n')
        log_file.write(str(datetime.now()) + '\n')
        tables = rospy.get_param('/tables')
        for table in tables:
            log_file.write(table + ':\n')
            log_file.write(str(tables[table]) + '\n')
        log_file.write('=' * 50)
        log_file.write('\n')
