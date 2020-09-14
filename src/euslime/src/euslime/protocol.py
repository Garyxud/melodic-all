from sexpdata import dumps, loads, Symbol
import signal
import traceback

from euslime.bridge import EuslispResult
from euslime.handler import DebuggerHandler
from euslime.logger import get_logger

log = get_logger(__name__)


class Protocol(object):
    def __init__(self, handler, *args, **kwargs):
        self.handler = handler(*args, **kwargs)

    def dumps(self, sexp):
        def with_header(sexp):
            res = dumps(sexp, false_as='nil', none_as='nil')
            # encode to adapt to japanese characters
            header = '{0:06x}'.format(len(res.encode('utf-8')))
            return header + res
        try:
            return with_header(sexp)
        except UnicodeDecodeError:
            # For example in (apropos "default")
            log.warn('UnicodeDecodeError at %s' % sexp)
            assert isinstance(sexp, list)
            sexp = [unicode(x, 'utf-8', 'ignore') if isinstance(x, str)
                    else x for x in sexp]
            return with_header(sexp)

    def make_error(self, id, err):
        debug = DebuggerHandler(id, err)
        self.handler.debugger.append(debug)

        res = [
            Symbol(':debug'),
            0,  # the thread which threw the condition
            len(self.handler.debugger),  # the depth of the condition
            [debug.message, str(), None],  # s-exp with a description
            debug.restarts,  # list of available restarts
            debug.stack[:10],  # stacktrace
            [None],  # pending continuation
        ]
        yield self.dumps(res)

    def make_response(self, id, sexp):
        try:
            res = [Symbol(':return'), {'ok': sexp}, id]
            yield self.dumps(res)
        except Exception as e:
            self.command_id.append(id)
            for r in self.make_error(id, e):
                yield r

    def interrupt(self):
        yield self.dumps([Symbol(":read-aborted"), 0, 1])
        self.handler.euslisp.process.send_signal(signal.SIGINT)
        self.handler.euslisp.reset()
        yield self.dumps([Symbol(':return'),
                          {'abort': "'Keyboard Interrupt'"},
                          self.handler.command_id.pop()])

    def process(self, data):
        data = loads(data)
        if data[0] == Symbol(":emacs-rex"):
            cmd, form, pkg, thread, comm_id = data
            self.handler.command_id.append(comm_id)
            self.handler.package = pkg
        else:
            form = data
            comm_id = None
        func = form[0].value().replace(':', '_').replace('-', '_')
        args = form[1:]

        log.info("func: %s" % func)
        log.info("args: %s" % args)

        try:
            gen = getattr(self.handler, func)(*args)
            if not gen:
                if comm_id:
                    for r in self.make_response(self.handler.command_id.pop(), None):
                        yield r
                return
            for resp in gen:
                if isinstance(resp, EuslispResult):
                    for r in self.make_response(self.handler.command_id.pop(),
                                                resp.value):
                        yield r
                else:
                    yield self.dumps(resp)
        except Exception as e:
            log.error(traceback.format_exc())
            for r in self.make_error(self.handler.command_id[-1], e):
                yield r
