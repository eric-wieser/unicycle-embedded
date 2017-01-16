import signal
import asyncio

from .shared import SharedFuture

_futures = {}

def _async_handler(sig, frame):
    _futures[sig].parent.set_result(None)

def wait_for_signal(which=signal.SIGINT, exc_type=None):
    """
    Takes a signal to wait for, and produces a future that completes when that
    signal fires.

    After the future completes, the previous signal handler is restored
    """
    f = _futures.get(which)

    if not f:
        _futures[which] = f = SharedFuture(asyncio.Future())

        @f.parent.add_done_callback
        def on_done(_):
            signal.signal(which, old_handler)
            del _futures[which]

        old_handler = signal.signal(which, _async_handler)

    return f.split()

class CapturedKeyboardInterrupt(Exception, KeyboardInterrupt):
    """ This must be derived from Exception, else it breaks the event loop """

async def intercept_ctrlc():
    await wait_for_signal(signal.SIGINT)
    raise CapturedKeyboardInterrupt

