import asyncio

# for doctests only
def _print_state(*fs):
    def _describe_one(f):
        if f.cancelled():
            return "cancelled"
        elif f.done():
            if f.exception():
                return "exception={}".format(f.exception())
            else:
                return "result={}".format(f.result())
        else:
            return "pending"
    print(*(_describe_one(f) for f in fs))

class SharedFuture:
    """
    A future shared across multiple child futures

    If the parent finishes in any way, then all the pending children finish in the same way:

        >>> async def main():
        ...    parent = asyncio.ensure_future(asyncio.Future())
        ...    a, b = SharedFuture(parent).split(2)
        ...    await asyncio.sleep(0)
        ...    _print_state(parent, a, b)
        ...    parent.set_result(42)
        ...    await asyncio.sleep(0)
        ...    _print_state(parent, a, b)

        >>> loop = asyncio.get_event_loop()
        >>> loop.run_until_complete(main())
        pending pending pending
        result=42 result=42 result=42

    If all the children are cancelled before the parent, then the parent is cancelled:

        >>> async def main():
        ...    parent = asyncio.ensure_future(asyncio.Future())
        ...    a, b = SharedFuture(parent).split(2)
        ...    await asyncio.sleep(0)
        ...    _print_state(parent, a, b)
        ...    a.cancel()
        ...    await asyncio.sleep(0)
        ...    _print_state(parent, a, b)
        ...    b.cancel()
        ...    await asyncio.sleep(0)
        ...    _print_state(parent, a, b)

        >>> loop = asyncio.get_event_loop()
        >>> loop.run_until_complete(main())
        pending pending pending
        pending cancelled pending
        cancelled cancelled cancelled

    Note that in both these examples, the event loop must be given time to process the cancellation chain
    """
    def __init__(self, parent):
        self.parent = parent
        self.children = set()
        parent.add_done_callback(self._parent_done_callback)

    def _child_done_callback(self, child):
        self.children.remove(child)
        if not self.parent.done() and not self.children:
            self.parent.cancel()

    def _parent_done_callback(self, parent):
        for child in self.children:
            if child.done():
                continue
            if parent.cancelled():
                child.cancel()
            else:
                exc = parent.exception()
                if exc is not None:
                    child.set_exception(exc)
                else:
                    child.set_result(parent.result())

    def split(self, n=None):
        """
        Split off child futures from this parent

        If n is specified, return a tuple of futures, else return a single one.
        """
        if n is not None:
            return tuple(self.split() for i in range(n))

        f = asyncio.Future()
        f.add_done_callback(self._child_done_callback)
        self.children.add(f)
        return asyncio.ensure_future(f)


if __name__ == '__main__':
    import doctest
    doctest.testmod()
