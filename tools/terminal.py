import asyncio
import codecs
import collections
import sys
import warnings

import serial
import serial.tools.list_ports
from cobs import cobs

import messages_pb2
from google.protobuf.message import DecodeError

SERIAL_NO = 'A5004HJMA'  # serial number of the microchip, used to find COm port
BAUD_RATE = 57600


class AsyncSerial(serial.Serial):
    """ Simple wrapper of Serial, that provides an async api """
    async def read(self, n=None) -> bytes:
        take_all = n is None
        if take_all: n = 1

        while self.in_waiting < n:
            await asyncio.sleep(0.05)

        if take_all: n = self.in_waiting
        return super().read(n)


class COBSReader:
    """
    Wraps an object with an async read, and emits packets
    decoded using COBS
    """
    def __init__(self, conn):
        self._conn = conn
        self._pending_bytes = b""
        self._pending_packets = collections.deque()

    async def read_raw_packet(self) -> bytes:
        while not self._pending_packets:
            part = await self._conn.read()
            parts = part.split(b'\0')
            if len(parts) > 1:
                old, *complete, new = parts
                self._pending_packets.append(self._pending_bytes + old)
                self._pending_packets.extend(complete)
                self._pending_bytes = new
            else:
                self._pending_bytes += parts[0]
        return self._pending_packets.popleft()

    async def read_packet(self) -> bytes:
        return cobs.decode(await self.read_raw_packet())


class ProtobufReader:
    def __init__(self, _conn):
        self._conn = _conn

    async def read(self) -> messages_pb2.RobotMessage:
        data = await self._conn.read_packet()
        try:
            return messages_pb2.RobotMessage.FromString(data)
        except DecodeError as e:
            raise ValueError('Could not decode {!r}'.format(data)) from e


def connect() -> AsyncSerial:
    try:
        arduino_port = next(
            p.device
            for p in serial.tools.list_ports.comports()
            if p.serial_number == SERIAL_NO
        )
    except StopIteration:
        raise IOError("No Arduino found") from None

    return AsyncSerial(arduino_port, baudrate=BAUD_RATE)


async def main():
    with connect() as ser:
        reader = ProtobufReader(COBSReader(ser))
        while True:
            val = await reader.read()
            print(val)


# run the main coroutine
loop = asyncio.get_event_loop()
loop.run_until_complete(
    asyncio.gather(main())
)
loop.close()
