# Actionlib messages
from pal_interaction_msgs.msg import TtsGoal

def planWakeWord(self, data):
        wake_word = data.data
        if not self.running and wake_word == 'start the demo':
            self.running = True
            self.plan_publisher.publish('p1PlanNew')

def talk(self, speech_in):
    # Create the TTS goal and send it
    print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
    tts_goal = TtsGoal()
    tts_goal.rawtext.lang_id = 'en_GB'
    tts_goal.rawtext.text = speech_in
    self.speech_client.send_goal(tts_goal)