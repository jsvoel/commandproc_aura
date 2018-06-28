#!/usr/bin/env python

import re # tokenize using regular expressions
from more_itertools import peekable # fast iterator with peek support
from threading import Thread # launch other programs in own thread
import subprocess32 # launch other programms / commands
from time import sleep # sadness

import rospy
from commandproc_aura.msg import CommandString
from robotendpoint_aura.msg import RoboBookmark, RoboSpeech

loglevel = rospy.get_param('/debug/loglevel', rospy.INFO)

class Command(Thread):
    """ Helper class that launches the commands gives in a seperate proccess.
    if the command is supposed to be blocking, the run method will block until
    the command is executed
    """
    _bookmarkcounter = 49 # keep track of the creation number for bookmark referencing
    _commandstore = {} # store the created commands

    def __init__(self, comtext):
        super(Command, self).__init__()
        # add this command to the reference pool of all commands
        Command._bookmarkcounter += 1
        if Command._bookmarkcounter > 1000:
            Command._bookmarkcounter = 50
        Command._commandstore[Command._bookmarkcounter] = self
        # disect the command to create the necesarry information for its processing
        comtext = comtext.split('_')
        self._blocking = comtext[0][1] == 'r'
        self._command = comtext[1:] # keep all the parts as list of strings
        self._bookmark = Command._bookmarkcounter
        self._bookmarkstring = "\\mrk={}\\".format(Command._bookmarkcounter)
        self._started = False
        self._done = False

    def bookmarkToString(self):
        return self._bookstring

    def is_blocking(self):
        return self._blocking

    def is_completed(self):
        return self._done

    def waitOn(self):
        while not self._done:
            sleep(0.05)
        #self.join() doesn't work well because commands usually are started asynchrone

    def start(self):
        self._started = True
        Thread.start(self)

    def run(self):
        subprocess32.run(self._command)
        self._done = True
        if Command._commandstore[self._bookmark] is self:
            Command._commandstore[self._bookmark] = None

    @classmethod
    def startByBookmark(cls, bookmark):
        bookmark = bookmark.bookmark
        if bookmark in cls._commandstore:
            command = cls._commandstore[bookmark]
            if command:
                rospy.loginfo("Launching command: " + ' '.join(command._command))
                command.start()
            else:
                rospy.logwarn("Attempt to launch not existant command with command {}".format(bookmark))
        else:
            rospy.logerr("Bookmark not found in commandstore: {}".format(bookmark))

class Action:
    def __init__(self):
        self.chunks = []
        self.commands = []

    def execute(self, publishfunc):
        if len(self.chunks) == len(self.commands):
            # no text, just launch all the commands
            # if they are blocking ones, wait
            for command in self.commands:
                command.start()
                if command.is_blocking():
                    command.join()
        else:
            # commands and text mixed, only the last command might be blocking
            # or no commands at all
            speech = RoboSpeech()
            speech.text = ' '.join(self.chunks)
            rospy.loginfo("Text for robot: {}".format(speech.text))
            publishfunc(speech)
            if self.commands and self.commands[-1].is_blocking():
                self.commands[-1].waitOn() # cant use join here because async speech can get here before the thread is launched

class CommandProcessor:
    def __init__(self, speechpublishfunc):
        self._speechpub = speechpublishfunc
        self._command_pattern = re.compile(r'![srw]\w+')

    def _filter(self, answer):
        tokenstream = peekable(answer.split(' '))
        if tokenstream.peek(0) == 0:
            return [] # just return empty list on empty input pretty much
        actionlist = [Action()] # the final output of this function
        for tok in tokenstream:
            if re.match(self._command_pattern, tok): # is the current token a command or just whatever?
                command = Command(tok)
                actionlist[-1].commands.append(command)
                actionlist[-1].chunks.append(command._bookmarkstring)
                if command.is_blocking():
                    try:
                        tokenstream.peek() # if tokenstream is empty, this will throw stop iteration
                        actionlist.append(Action())
                    except:
                        pass # stream is empty, just continue loop to finish the filtering function
            else:
                actionlist[-1].chunks.append(tok) # its whatever, put it at the end of the current chunk
        return actionlist

    def execute(self, commandmsg):
        rospy.loginfo("Processing CommandString: {}".format(commandmsg.command))
        actions = self._filter(commandmsg.command)
        for action in actions:
            action.execute(self._speechpub)


rospy.init_node('commandproc', anonymous=False, log_level=loglevel)

robospeechpub = rospy.Publisher(rospy.get_namespace() + 'robospeech', RoboSpeech, queue_size=10)

cp = CommandProcessor(robospeechpub.publish)

rospy.Subscriber(rospy.get_namespace() + 'robobookmark', RoboBookmark, Command.startByBookmark)
rospy.Subscriber(rospy.get_namespace() + 'commandstring', CommandString, cp.execute)

while not rospy.is_shutdown():
    try:
        rospy.spin()
    except:
        pass

rospy.loginfo("commandproc node shutdown")
