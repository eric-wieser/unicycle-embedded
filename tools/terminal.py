#! python3
import asyncio
import os
import sys
import functools
import readline
import time
import traceback
import signal
import collections

import colorama

import messages_pb2
import comms
from async_helpers import AsyncCmd, async_race, intercept_ctrlc
import protobufcolor
import matlabio

# start off by patching readline
if not hasattr(readline, 'redisplay'):
    if not hasattr(readline, rl):
        raise NotImplementedError("readline.redisplay is missing and can't be patched")
    def redisplay(rl):
        rl._print_prompt()
        rl._update_line()

    readline.rl.__class__.redisplay = redisplay
    readline.redisplay = redisplay

class CommandsBase(AsyncCmd):
    """
    A basic command line, that prints errors in color, and tracks whether to
    redraw the prompt
    """

    ERROR_COLOR = colorama.Fore.RED
    WARN_COLOR = colorama.Fore.YELLOW
    DEBUG_COLOR = colorama.Fore.BLACK + colorama.Style.BRIGHT
    INFO_COLOR = ''

    def __init__(self):
        super().__init__()
        self._at_prompt = True

    async def onecmd(self, line):
        self._at_prompt = False
        try:
            return await super().onecmd(line)
        except Exception as e:
            print(colorama.Style.RESET_ALL + self.ERROR_COLOR, end='')
            traceback.print_exc()
        finally:
            self._at_prompt = True

    def log(self, val, style=''):
        """
        Log text to the command line. If the prompt is active, delete it, then
        redraw it below the printed line
        """
        val = colorama.Style.RESET_ALL + style + str(val)
        if self._at_prompt:
            print('\x1b[2K\r',end='')
            print(val)
            readline.redisplay()
        else:
            print(val)

    def debug(self, text):
        self.log(text, style=self.DEBUG_COLOR)

    def info(self, text):
        self.log(text, style=self.INFO_COLOR)

    def warn(self, text):
        self.log(text, style=self.WARN_COLOR)

    def error(self, text):
        self.log(text, style=self.ERROR_COLOR)


def requires_connection(method):
    """ Takes a method, and wraps it such that it errors if self.stream is None """
    @functools.wraps(method)
    def wrapped(self, *args, **kwargs):
        if not self.stream:
            self.error("Not connected")
            return asyncio.sleep(0)
        return method(self, *args, **kwargs)
    return wrapped



class Commands(CommandsBase):
    prompt = colorama.Style.BRIGHT + "yauc> " + colorama.Style.RESET_ALL
    intro = "Yaw Actuated UniCycle command line interface"

    def __init__(self):
        super().__init__()
        self.stream = None
        self.incoming_task = None
        self.cmdqueue = ['connect']

        self.log_last_printed = time.time()
        self.log_saver = matlabio.LogSaver()

        self.awaited_log_bundle = None

    def send(self, msg):
        self.stream.write(msg)
        # self.info("Sent {!r}".format(msg))

    async def recv_single(self, val):
        """ handle a single incoming packet """
        which = val.WhichOneof('msg')
        if which == 'debug':
            self.debug(val.debug.s)
            return

        elif which == 'log_bundle':
            val = val.log_bundle
            if self.awaited_log_bundle:
                self.awaited_log_bundle.set_result(val)
            else:
                self.warn("Unexpected logs")

        elif which == 'single_log':
            val = val.single_log

            this_time = time.time()
            if this_time - self.log_last_printed < 0.5:
                return
            self.log_last_printed = this_time
            self.info(val)

        else:
            self.info(val)

    async def recv_incoming_task(self):
        try:
            while self.stream:
                try:
                    val = await self.stream.read()
                except comms.CommsError as e:
                    self.error(e)
                    continue

                await self.recv_single(val)

        except comms.SerialException as e:
            self.error("Connection lost")
            self.stream = None
            if self.awaited_log_bundle:
                self.awaited_log_bundle.set_exception(e)

    # methods that perform the actions, with no command parsing

    async def run_connect(self):
        if self.stream:
            self.warn("Already connected")
            return

        ser = comms.connect()

        print("Connected!")

        self.stream = comms.ProtobufStream(comms.COBSStream(ser))
        self.incoming_task = asyncio.ensure_future(self.recv_incoming_task())

    async def run_disconnect(self):
        if not self.stream:
            self.warn("Already disconnected")

        try:
            await self.run_stop()
        except Exception:
            pass
        try:
            self.stream.close()
        except Exception:
            pass

        self.stream = None
        await self.incoming_task

    async def run_go(self, steps=50, forever=False):
        # send the initial message to set things going
        msg = messages_pb2.PCMessage()
        msg.go.SetInParent()
        msg.go.steps = steps if not forever else -1
        self.send(msg)

        if forever:
            try:
                await intercept_ctrlc()
            except KeyboardInterrupt:
                await self.run_stop()
            return

        # prepare to recieve the logs
        get_logs_msg = messages_pb2.PCMessage()
        msg.get_logs.SetInParent()

        async def get_logs():
            while True:
                await asyncio.sleep(0.1)

                # reconnect when possible
                if not self.stream:
                    self.info("Reconnecting")
                    while True:
                        try:
                            await self.run_connect()
                        except comms.NoArduinoFound:
                            await asyncio.sleep(1)
                        else:
                            break
                    self.info("Success!")

                # ask for logs
                self.info('Asking for logs')
                self.awaited_log_bundle = asyncio.Future()
                try:
                    self.send(msg)
                except comms.SerialException:
                    await self.run_disconnect()
                    continue

                # await a result
                self.info('Waiting for reply')
                try:
                    res = await self.awaited_log_bundle
                except comms.SerialException:
                    await self.run_disconnect()
                    continue
                finally:
                    self.awaited_log_bundle = None

                if res.entry:
                    return res

        try:
            val = await async_race(get_logs(), intercept_ctrlc())
        except KeyboardInterrupt:
            self.info('Cancelled')
            await self.run_stop()
        else:
            self.info('Success')
            target = self.log_saver.save(val)
            self.info('Saved to {}'.format(target))

    async def run_stop(self):
        if not self.stream: return
        msg = messages_pb2.PCMessage()
        msg.stop.SetInParent()
        self.send(msg)

    async def run_policy(self):
        msg = messages_pb2.PCMessage()
        msg.controller.SetInParent()
        self.send(msg)
        print("Not really implemented")

    # next come the command parsers

    def do_connect(self, arg):
        """ Connect to the robot """
        try:
            return self.run_connect()
        except comms.NoArduinoFound:
            self.error("Robot not found")
        except comms.ArduinoConnectionFailed as e:
            self.error("Connecting to the arduino on {} failed".format(e.port))
        return asyncio.sleep(0)

    def do_disconnect(self, arg):
        """ Connect to the robot """
        return self.run_disconnect()


    @requires_connection
    def do_go(self, arg):
        """
        Start a test run
        Optionally takes an argument, the number of iterations to run for

        go
        go <n>
        go forever
        """
        if arg == 'forever':
            return self.run_go(forever=True)
        elif arg:
            try:
                steps = int(arg)
            except ValueError:
                print("Invalid argument {!r}".format(arg))
            else:
                return self.run_go(steps)
        else:
            return self.run_go()

    @requires_connection
    def do_stop(self, arg):
        """
        Abort any active run. Takes no arguments
        """
        if arg:
            self.error("stop takes no argument")
            return
        return self.run_stop()

    @requires_connection
    def do_policy(self, args):
        """
        Set the policy, from a mat file
        """
        return self.run_policy()

    # finally come the hooks for the parser

    async def default(self, line):
        if line == 'EOF':
            return True
        else:
            await super().default(line)

    async def postloop(self):
        print("Exiting cli and stopping motors")
        await self.run_disconnect()

colorama.init()
protobufcolor.init()

# get an event loop
if os.name == 'nt':
    loop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
    asyncio.set_event_loop(loop)
else:
    loop = asyncio.get_event_loop()

loop.run_until_complete(Commands().cmdloop())
loop.close()
