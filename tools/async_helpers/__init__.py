"""
This module provides a set of helper tools for working with the builtin
asyncio package
"""
import asyncio

from .shared import SharedFuture
from .cmd import AsyncCmd
from .signal import intercept_ctrlc

async def async_race(*futures):
    """ Race a set of futures. Await the one that finishes first, and cancel the rest """
    done, running = await asyncio.wait(futures, return_when=asyncio.FIRST_COMPLETED)

    for r in running:
        r.cancel()

    return await done.pop()
