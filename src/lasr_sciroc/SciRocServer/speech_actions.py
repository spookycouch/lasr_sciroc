# Actionlib messages
import rospy
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsGoal
from datetime import datetime
import os

def planWakeWord(self, data):
        wake_word = data.data
        if not self.running and wake_word == 'start the demo':
            self.running = True
            plan_publisher = rospy.Publisher('/sciroc/planToExec', String, queue_size=1)
            plan_publisher.publish('fullPlan')
            self.sub.unregister()

def keywordCallback(self, data):
    if data.data == self.current_keyword:
        self.keyword_found = True

def keywordDetected(self, keyword):
    self.current_keyword = keyword
    self.keyword_found = False
    start_time = rospy.Time.now().secs
    sub = rospy.Subscriber('/wake_word/wake_word_detected', String, self.keywordCallback)
    while not self.keyword_found:
        current_time = rospy.Time.now().secs
        if current_time - start_time > 15:
            rospy.loginfo('keyword timeout')
            break
        rospy.sleep(1)

def talk(self, speech_in):
    # Create the TTS goal and send it
    print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
    tts_goal = TtsGoal()
    tts_goal.rawtext.lang_id = 'en_GB'
    tts_goal.rawtext.text = speech_in
    self.speech_client.send_goal(tts_goal)
    self.speech_client.wait_for_result()

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
