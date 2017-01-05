""" patch protobuf to print in color """

import functools

from colorama import Fore, Style
from google.protobuf.text_format import _Printer

def patches(cls: type):
    """
    A decorator that takes a cls to patch, and replaces the method matching
    the name of the provided function with that function.

    The function is called with an additional first argument, the original
    unbound method
    """
    def decorator(f):
        orig = getattr(cls, f.__name__)
        @functools.wraps(orig)
        def wrapped(*args, **kwargs):
            return f(orig, *args, **kwargs)
        setattr(cls, f.__name__, wrapped)
    return decorator

def init():
    @patches(_Printer)
    def PrintField(orig, self, field, value):
        self.out.write(Fore.CYAN)
        return orig(self, field, value)
    @patches(_Printer)
    def PrintFieldValue(orig, self, field, value):
        self.out.write(Style.RESET_ALL)
        return orig(self, field, value)
