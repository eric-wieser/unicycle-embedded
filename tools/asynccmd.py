from cmd import Cmd
import asyncio
import functools


def _wrap(f):
    @functools.wraps(f)
    async def async_f(*args, **kwargs):
        return f(*args, **kwargs)
    return async_f

class AsyncCmd(Cmd):
    """
    Like cmd.Cmd, but all operations are async
    """

    preloop = _wrap(Cmd.preloop)
    postloop = _wrap(Cmd.postloop)
    precmd = _wrap(Cmd.precmd)
    postcmd = _wrap(Cmd.postcmd)
    default = _wrap(Cmd.default)
    do_help = _wrap(Cmd.do_help)

    @functools.wraps(Cmd.emptyline)
    async def emptyline(self):
        if self.lastcmd:
            return await self.onecmd(self.lastcmd)

    @functools.wraps(Cmd.onecmd)
    async def onecmd(self, line):
        cmd, arg, line = self.parseline(line)
        if not line:
            return await self.emptyline()
        if cmd is None:
            return await self.default(line)
        self.lastcmd = line
        if line == 'EOF' :
            self.lastcmd = ''
        if cmd == '':
            return await self.default(line)
        else:
            try:
                func = getattr(self, 'do_' + cmd)
            except AttributeError:
                return await self.default(line)
            return await func(arg)

    @functools.wraps(Cmd.cmdloop)
    async def cmdloop(self, intro=None):
        await self.preloop()
        if self.use_rawinput and self.completekey:
            try:
                import readline
            except ImportError:
                pass
            else:
                self.old_completer = readline.get_completer()
                readline.set_completer(self.complete)
                readline.parse_and_bind(self.completekey+": complete")
        try:
            if intro is not None:
                self.intro = intro
            if self.intro:
                self.stdout.write(str(self.intro)+"\n")
            stop = None
            while not stop:
                if self.cmdqueue:
                    line = self.cmdqueue.pop(0)
                else:
                    line = await self.get_input()
                line = await self.precmd(line)
                stop = await self.onecmd(line)
                stop = await self.postcmd(stop, line)
            await self.postloop()
        finally:
            if self.use_rawinput and self.completekey:
                try:
                    import readline
                    readline.set_completer(self.old_completer)
                except ImportError:
                    pass

    def _get_line_rawinput(self):
        try:
            return input(self.prompt)
        except EOFError:
            return 'EOF'

    def _get_line_stdin(self):
        self.stdout.write(self.prompt)
        self.stdout.flush()
        line = self.stdin.readline()
        if not len(line):
            line = 'EOF'
        else:
            line = line.rstrip('\r\n')

    async def get_input(self):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None,
            self._get_line_rawinput if self.use_rawinput else self._get_line_stdin
        )