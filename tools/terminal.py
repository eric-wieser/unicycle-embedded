#! python3
import asyncio
import os
import sys
import functools
import readline
import time

import colorama
from colorama import Fore, Style

import messages_pb2
from comms import ProtobufStream, COBSStream, connect, CommsError
from asynccmd import AsyncCmd
import protobufcolor

def print_and_reprompt(val):
    val = str(val)
    print('\x1b[2K\r',end='')
    print(val)
    readline.redisplay()

async def main():
    with connect() as ser:
        stream = ProtobufStream(COBSStream(ser))

        await asyncio.gather(
            show_incoming(stream),
            build_outgoing(stream)
        )

async def show_incoming(reader):
    last_time = time.time()
    while True:
        this_time = time.time()
        try:
            val = await reader.read()
        except CommsError as e:
            pass # print_and_reprompt(e)
        else:
            ok = True
            if val.WhichOneof('msg') != 'log':
                ok = True
            elif this_time - last_time > 0.5:
                ok = True
                last_time = this_time
            else:
                ok = False
            if ok:
                print_and_reprompt(val)

async def build_outgoing(writer):
    await Commands(writer).cmdloop()

class Commands(AsyncCmd):
    prompt = Style.BRIGHT + "yauc> " + Style.RESET_ALL

    def __init__(self, stream):
        self.stream = stream
        super().__init__()

    def send(self, msg):
        self.stream.write(msg)
        print("Sent {!r}".format(msg))

    async def do_go(self, arg):
        """
        Start a test run
        Optionally takes an argument, the number of iterations to run for

        go
        go <n>
        go forever
        """
        msg = messages_pb2.PCMessage()
        msg.go.SetInParent()
        if arg == 'forever':
            msg.go.steps = -1
        elif arg:
            try:
                msg.go.steps = int(arg)
            except ValueError:
                print("Invalid argument {!r}".format(arg))
                return
        self.send(msg)

    async def do_stop(self, arg):
        """
        Abort any active run. Takes no arguments
        """
        if arg:
            print("stop takes no argument")
            return
        msg = messages_pb2.PCMessage()
        msg.stop.SetInParent()
        self.send(msg)

    async def do_policy(self, args):
        """
        Set the policy, from a mat file
        """
        msg = messages_pb2.PCMessage()
        msg.controller.SetInParent()
        self.send(msg)
        print("Not really implemented")

    async def default(self, line):
        if line == 'EOF':
            await self.do_stop('')
            return True
        else:
            await super().default(line)

colorama.init()
protobufcolor.init()

# get an event loop
if os.name == 'nt':
    loop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
    asyncio.set_event_loop(loop)
else:
    loop = asyncio.get_event_loop()

loop.run_until_complete(main())
loop.close()
