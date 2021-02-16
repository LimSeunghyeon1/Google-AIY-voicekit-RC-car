#!/usr/bin/env python3
# Copyright 2017 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Activates the Google Assistant with either a hotword or a button press, using the
Google Assistant Library.

The Google Assistant Library has direct access to the audio API, so this Python
code doesn't need to record audio.

.. note:

    Hotword detection (such as "Okay Google") is supported only with Raspberry Pi 2/3.
    If you're using a Pi Zero, this code works but you must press the button to activate
    the Google Assistant.
"""

import logging
import platform
import sys
import threading
import time
from gpiozero import Motor
from aiy.pins import PIN_A,PIN_B,PIN_C,PIN_D

from google.assistant.library.event import EventType

from aiy.assistant import auth_helpers
from aiy.assistant.library import Assistant
from aiy.board import Board, Led

motor1=Motor(PIN_A,PIN_B)  ##left tire
motor2=Motor(PIN_D,PIN_C)  ##right tire


def go_straight():
    motor1.forward()
    motor2.forward()
    time.sleep(5)
    motor1.stop()
    motor2.stop()

def turn_right():
    motor1.forward()
    motor2.backward()
    time.sleep(0.5)
    motor1.stop()
    motor2.stop()

def turn_left():
    print("hoho")
    motor1.backward()
    motor2.forward()
    time.sleep(0.5)
    motor1.stop()
    motor2.stop()

def go_backward():
    motor1.backward()
    motor2.backward()
    time.sleep(5)
    motor1.stop()
    motor2.stop()

class MyAssistant:
    """An assistant that runs in the background.

    The Google Assistant Library event loop blocks the running thread entirely.
    To support the button trigger, we need to run the event loop in a separate
    thread. Otherwise, the on_button_pressed() method will never get a chance to
    be invoked.
    """

    def __init__(self):
        self._task = threading.Thread(target=self._run_task)
        self._can_start_conversation = False
        self._assistant = None

        self._board = Board()
        self._board.button.when_pressed= self._on_button_pressed
    def start(self):
        """
        Starts the assistant event loop and begins processing events.
        """
        self._task.start()

    def _run_task(self):
        credentials = auth_helpers.get_assistant_credentials()
        with Assistant(credentials) as assistant:
            self._assistant = assistant
            for event in assistant.start():
                self._process_event(event)

    def _process_event(self, event):
        logging.info(event)
        if event.type == EventType.ON_START_FINISHED:
            self._board.led.status = Led.BEACON_DARK  # Ready.
            self._can_start_conversation = True
            # Start the voicehat button trigger.
            logging.info('Say "OK, Google" or press the button, then speak. '
                         'Press Ctrl+C to quit...')
            

        elif event.type == EventType.ON_CONVERSATION_TURN_STARTED:
            self._can_start_conversation = False
            self._board.led.state = Led.ON  # Listening.

        elif event.type == EventType.ON_END_OF_UTTERANCE:
            self._board.led.state = Led.PULSE_QUICK  # Thinking.

        elif (event.type == EventType.ON_CONVERSATION_TURN_FINISHED
              or event.type == EventType.ON_CONVERSATION_TURN_TIMEOUT
              or event.type == EventType.ON_NO_RESPONSE):
            self._board.led.state = Led.BEACON_DARK  # Ready.
            self._can_start_conversation = True
            
        elif event.type==EventType.ON_RECOGNIZING_SPEECH_FINISHED and event.args:
            print("You said : ",event.args['text'])
            text=event.args['text'].lower()
            if text.find('go straight')>=0:
                self._assistant.stop_conversation()
                while(1):
                    go_straight()
                    break
                ##go_straight()
            elif text.find('turn right')>=0:
                self._assistant.stop_conversation()
                while(1):
                    turn_right()
                    break
            elif text.find('turn left')>=0:
                self._assistant.stop_conversation()
                while(1):
                    turn_left()
                    break
            elif text.find('move back')>=0:
                self._assistant.stop_conversation()
                while(1):
                    go_backward()
                    break
        elif event.type == EventType.ON_ASSISTANT_ERROR and event.args and event.args['is_fatal']:
            sys.exit(1)

    def _on_button_pressed(self): ## able to start conversation pressing the button
        # Check if we can start a conversation. 'self._can_start_conversation'
        # is False when either:
        # 1. The assistant library is not yet ready; OR
        # 2. The assistant library is already in a conversation.
        if self._can_start_conversation:
            print("hi")
            self._assistant.start_conversation()


def main():
    logging.basicConfig(level=logging.INFO)
    MyAssistant().start()


if __name__ == '__main__':
    main()
