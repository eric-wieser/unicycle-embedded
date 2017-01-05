""" patch protobuf to print in color """

import functools

from colorama import Fore, Style
from google.protobuf.text_format import _Printer


def patches(type, name):
    orig = getattr(type, name)
    def dec(f):
        @functools.wraps(orig)
        def wrapped(*args, **kwargs):
            return f(orig, *args, **kwargs)
        setattr(type, name, wrapped)
    return dec

def init():
    @patches(_Printer, 'PrintField')
    def new_pf(orig, self, field, value):
        self.out.write(Fore.CYAN)
        return orig(self, field, value)
    @patches(_Printer, 'PrintFieldValue')
    def new_pfv(orig, self, field, value):
        self.out.write(Style.RESET_ALL)
        return orig(self, field, value)
