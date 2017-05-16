import textwrap
import traceback

from prompt_toolkit import AbortAction, CommandLineInterface
from prompt_toolkit.document import Document
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.shortcuts import (
    print_tokens, style_from_dict, prompt_async, create_prompt_application,
    create_output, create_asyncio_eventloop)

from pygments.token import Token

class CommandBase:
    """
    A basic command line, that prints errors in color, and handles simple
    commands
    """

    def __init__(self, style):
        self._commands = {}
        for d in dir(self.__class__):
            if d.startswith('do_'):
                name = d[3:]
                func = getattr(self, d)
                doc = textwrap.dedent(func.__doc__ or '')
                self._commands[name] = (func, doc)

        self._history = InMemoryHistory()

        self._application = create_prompt_application(
            get_prompt_tokens=self.get_prompt_tokens,
            style=style,
            on_abort=AbortAction.RETURN_NONE,
            history=self._history)

        self._cli = CommandLineInterface(
            application=self._application,
            output=create_output(true_color=True),
            eventloop=create_asyncio_eventloop()
        )

        self._style = style

    def print_tokens(self, tokens):
        self._cli.run_in_terminal(lambda:
            print_tokens(tokens + [(Token, '\n')],
                style=self._style, true_color=True)
        )

    def _log(self, level, text, robot):
        self.print_tokens([(level, str(text))])

    def debug(self, text, **kwargs): self._log(Token.Debug, text, **kwargs)
    def info(self, text, **kwargs): self._log(Token.Info, text, **kwargs)
    def warn(self, text, **kwargs): self._log(Token.Warning, text, **kwargs)
    def error(self, text, **kwargs): self._log(Token.Error, text, **kwargs)

    async def do_help(self, arg):
        """ Get help for a command """
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
            prompter = self._cli.run_async()
            try:
                line = await prompter
            except EOFError:
                break
            if isinstance(line, Document):
                line = line.text

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
            except KeyboardInterrupt:
                self.warn("{!r} was interrupted".format(line))
            except Exception:
                self.print_tokens([(Token.Error, traceback.format_exc())])
