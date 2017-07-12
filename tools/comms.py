import asyncio
import collections

import serial
import serial.tools.list_ports
from cobs import cobs
from google.protobuf.message import DecodeError
from serial import SerialException

import messages_pb2
# serial number of the microchip, used to find COm port
# One of these is for mac, the other for windows
SERIAL_NOS = ['A5004Hjm', 'A5004HJMA']
BAUD_RATE = 57600

class StreamWrapper:
    def __init__(self, _conn):
        self._conn = _conn

    def close(self):
        self._conn.close()

    def __enter__(self): return self
    def __exit__(self, exc_type, exc_val, exc_tb): self.close()

class CommsError(ValueError): pass

class NoArduinoFound(Exception): pass

class ArduinoConnectionFailed(Exception):
    def __init__(self, port):
        super().__init__(port)
        self.port = port

class AsyncSerial(serial.Serial):
    """ Simple wrapper of Serial, that provides an async api """
    async def read(self, n=None) -> bytes:
        take_all = n is None
        if take_all: n = 1

        while self.in_waiting < n:
            await asyncio.sleep(0.05)

        if take_all: n = self.in_waiting
        return super().read(n)


class COBSStream(StreamWrapper):
    """
    Wraps an object with an async read, and emits packets
    decoded using COBS
    """
    def __init__(self, conn):
        super().__init__(conn)
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
        raw = await self.read_raw_packet()
        try:
            return cobs.decode(raw)
        except cobs.DecodeError as e:
            raise CommsError("Could not cobs-decode {!r}".format(raw)) from e

    def write_packet(self, packet):
        raw = cobs.encode(packet) + b'\0'
        self._conn.write(raw)
        self._conn.flush()


class ProtobufStream(StreamWrapper):
    async def read(self) -> messages_pb2.RobotMessage:
        data = await self._conn.read_packet()
        try:
            return messages_pb2.RobotMessage.FromString(data)
        except DecodeError as e:
            raise CommsError('Could not decode {!r}'.format(data)) from e

    def write(self, msg: messages_pb2.PCMessage):
        assert isinstance(msg, messages_pb2.PCMessage)
        self._conn.write_packet(msg.SerializeToString())


def connect(serial_nos=SERIAL_NOS, baud=BAUD_RATE) -> AsyncSerial:
    try:
        arduino_port = next(
            p.device
            for p in serial.tools.list_ports.comports()
            if p.serial_number in serial_nos
        )
    except StopIteration:
        raise NoArduinoFound from None

    try:
        return AsyncSerial(arduino_port, baudrate=baud)
    except SerialException as e:
        raise ArduinoConnectionFailed(arduino_port)
