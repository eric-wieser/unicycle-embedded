#! python3
import asyncio
import os
import sys
import functools
import time
import traceback
import signal
import collections
import textwrap

import messages_pb2
import comms
from async_helpers import async_race, intercept_ctrlc
import matlabio

from prompt_toolkit import AbortAction
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.shortcuts import print_tokens, style_from_dict, prompt_async
from pygments.token import Token
from pygments.lexer import RegexLexer, bygroups, include

style = style_from_dict({
    Token:         '#ansilightgray',

    Token.Prompt:  '#ansiwhite',
    Token.Error:   '#ansired',
    Token.Warning: '#ansibrown',
    Token.Debug:   '#ansidarkgray',

    Token.ProtobufField: '#ansiteal'
})



class ProtobufLexer(RegexLexer):
    name = 'Lex protobuf output'

    tokens = {
        'root': [
            (r'(\w+)(: )(.*)', bygroups(Token.ProtobufField, Token, Token.ProtobufValue)),
            (r'(\w+)( \{)', bygroups(Token.ProtobufField, Token), 'sub'),
            (r'\s+', Token),
        ],
        'sub': [
            include('root'),
            (r'}', Token, '#pop')
        ]
    }

class CommandsBase:
    """
    A basic command line, that prints errors in color, and handles simple
    commands
    """

    def __init__(self):
        self._commands = {}
        for d in dir(self.__class__):
            if d.startswith('do_'):
                name = d[3:]
                func = getattr(self, d)
                doc = textwrap.dedent(func.__doc__ or '')
                self._commands[name] = (func, doc)

        self._history = InMemoryHistory()

    def print_tokens(self, tokens):
        print_tokens(tokens + [(Token, '\n')], style=style, true_color=True)

    def print_pb_message(self, msg):
        tokens = list(ProtobufLexer().get_tokens(str(msg)))
        self.print_tokens(tokens)

    def debug(self, text):
        self.print_tokens([(Token.Debug, str(text))])

    def info(self, text):
        self.print_tokens([(Token.Info, str(text))])

    def warn(self, text):
        self.print_tokens([(Token.Warning, str(text))])

    def error(self, text):
        self.print_tokens([(Token.Error, str(text))])

    async def do_help(self, arg):
        if arg:
            if arg in self._commands:
                _, doc = self._commands[arg]
                print(doc)
            else:
                self.error('No command {}'.format(arg))
        else:
            print('Valid commands:')
            for c in self._commands:
                print('  ' + c)

    async def loop(self, intro=None):
        while True:
            prompter = prompt_async(
                patch_stdout=True,
                get_prompt_tokens=self.get_prompt_tokens,
                style=style,
                true_color=True,
                on_abort=AbortAction.RETURN_NONE,
                history=self._history)
            try:
                line = await prompter
            except EOFError:
                break

            if not line:
                continue

            # separate command and argument
            cmd, *args = line.split(' ', 1)
            if args:
                arg = args[0]
            else:
                arg = None

            # lookup the command
            try:
                cmd, _ = self._commands[cmd]
            except KeyError:
                self.error('No command {!r}'.format(cmd))
                continue

            # run the command
            try:
                await cmd(arg)
            except Exception:
                self.print_tokens([(Token.Error, traceback.format_exc())])


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

    async def loop(self):
        print("Yaw Actuated UniCycle command line interface")

        await self.run_connect()
        try:
            return await super().loop()
        finally:
            print("Exiting cli and stopping motors")
            await self.run_disconnect()

    def get_prompt_tokens(self, cli):
        return [(Token.Prompt, "yauc> ")]

    def __init__(self):
        super().__init__()
        self.stream = None
        self.incoming_task = None

        self.log_last_printed = time.time()
        self.log_saver = matlabio.LogSaver()

        self.awaited_log_bundle = None
        self.log_queue = None

    def send(self, msg):
        self.stream.write(msg)
        # self.info("Sent {!r}".format(msg))

    async def _recv_single(self, val):
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

            if self.log_queue is not None:
                self.log_queue.append(val)

            this_time = time.time()
            if this_time - self.log_last_printed < 0.5:
                return
            self.log_last_printed = this_time
            self.print_pb_message(val)

        else:
            self.print_pb_message(val)

    async def _recv_incoming_task(self):
        """ A background task to be run that receives incoming packets """
        try:
            while self.stream:
                try:
                    val = await self.stream.read()
                except comms.CommsError as e:
                    self.error(e)
                    continue

                await self._recv_single(val)

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
        self.incoming_task = asyncio.ensure_future(self._recv_incoming_task())

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
            fname, actual_steps = await self.handle_go_forever_response()
        else:
            fname, actual_steps = await self.handle_go_response()

        if not forever and steps > actual_steps:
            self.warn('Rollout was shorter than expected')

        self.info('Saved rollout of {} steps to {}'.format(actual_steps, fname))


    async def handle_go_forever_response(self):
        self.log_queue = q = []

        try:
            await intercept_ctrlc()
        except KeyboardInterrupt:
            self.log_queue = None
            print("Have {} items".format(len(q)))
            await self.run_stop()

        target = self.log_saver.save(q)
        return target, len(q)

    async def handle_go_response(self):
        # prepare to recieve the logs
        msg = messages_pb2.PCMessage()
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
                    return res.entry

        try:
            val = await async_race(get_logs(), intercept_ctrlc())
        except KeyboardInterrupt:
            self.info('Cancelled')
            await self.run_stop()
            raise

        self.info('Success')
        target = self.log_saver.save(val)
        return target, len(val)

    async def run_stop(self):
        if not self.stream: return
        self.debug('Requesting stop')
        msg = messages_pb2.PCMessage()
        msg.stop.SetInParent()
        self.send(msg)

    async def run_policy(self, matfile):
        msg = messages_pb2.PCMessage()
        msg.controller.SetInParent()
        msg.controller.CopyFrom(matlabio.load_policy(matfile))
        print(msg)
        self.send(msg)

    # next come the command parsers

    async def do_connect(self, arg):
        """
        Connect to the robot
        ::
            connect
        """
        try:
            return await self.run_connect()
        except comms.NoArduinoFound:
            self.error("Robot not found")
        except comms.ArduinoConnectionFailed as e:
            self.error("Connecting to the arduino on {} failed".format(e.port))

    def do_disconnect(self, arg):
        """
        Disconnect from the robot
        ::
            disconnect
        """
        return self.run_disconnect()


    @requires_connection
    async def do_go(self, arg):
        """
        Start a test run.

        Optionally takes an argument, the number of iterations to run for
        ::
            go
            go <n>
            go forever
        """
        if arg == 'forever':
            await self.run_go(forever=True)
        elif arg:
            try:
                steps = int(arg)
            except ValueError:
                self.error("Invalid argument {!r}".format(arg))
            else:
                await self.run_go(steps)
        else:
            await self.run_go()

    @requires_connection
    async def do_stop(self, arg):
        """
        Abort any active run. Takes no arguments
        ::
            stop
        """
        if arg:
            self.error("stop takes no argument")
            return
        await self.run_stop()

    @requires_connection
    async def do_policy(self, arg):
        """
        Set the policy, from a mat file
        ::
            policy <file>
        """
        if not arg:
            self.error('No file specified')
            return
        await self.run_policy(matfile=arg)


def enable_win_unicode_console():
    if sys.version_info >= (3, 6):
        # Since PEP 528, Python uses the unicode APIs for the Windows
        # console by default, so WUC shouldn't be needed.
        return

    import win_unicode_console
    win_unicode_console.enable()

if __name__ == '__main__':
    enable_win_unicode_console()

    # get an event loop
    if os.name == 'nt':
        loop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
        asyncio.set_event_loop(loop)
    else:
        loop = asyncio.get_event_loop()

    loop.run_until_complete(Commands().loop())
    loop.close()
