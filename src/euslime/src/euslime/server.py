try:
    import SocketServer as S
except ImportError:
    import socketserver as S

import socket
import time
import traceback
from threading import Event, Thread

from euslime.handler import EuslimeHandler
from euslime.logger import get_logger
from euslime.protocol import Protocol

ENCODINGS = {
    'iso-latin-1-unix': 'latin-1',
    'iso-utf-8-unix': 'utf-8'
}
HEADER_LENGTH = 6

log = get_logger(__name__)


class EuslimeRequestHandler(S.BaseRequestHandler, object):
    def __init__(self, request, client_address, server):
        self.swank = Protocol(EuslimeHandler, server.program, server.loader)
        self.swank.handler.euslisp.color = server.color
        self.encoding = ENCODINGS.get(server.encoding, 'utf-8')
        self.interrupt_request = Event()
        super(EuslimeRequestHandler, self).__init__(
            request, client_address, server)

    def _process_data(self, recv_data):
        try:
            for send_data in self.swank.process(recv_data):
                if self.interrupt_request.is_set():
                    self.interrupt_request.clear()
                    return
                log.debug('response: %s', send_data)
                send_data = send_data.encode(self.encoding)
                self.request.send(send_data)
        except KeyboardInterrupt:
            log.warn("Keyboard Interrupt!")
            self.interrupt_request.set()
            for msg in self.swank.interrupt():
                self.request.send(msg)

    def handle(self):
        """This method handles packets from swank client.
        The basic Slime packet consists of a 6 char hex-string
        followed by a S-exp with newline on the end.

        e.g.) 000016(:return (:ok nil) 1)\n
        """
        log.debug("Entering handle loop...")
        while not self.swank.handler.close_request.is_set():
            try:
                head_data = self.request.recv(HEADER_LENGTH,
                                              socket.MSG_DONTWAIT)
                log.debug('raw header: %s', head_data)
                if not head_data:
                    log.error('Empty header received. Closing socket.')
                    self.request.close()
                    break
                length = int(head_data, 16)
                recv_data = self.request.recv(length)
                log.debug('raw data: %s', recv_data)
                recv_data = recv_data.decode(self.encoding)
                Thread(target=self._process_data, args=(recv_data,)).start()
            except socket.timeout:
                log.error('Socket Timeout')
                break
            except socket.error:
                try:
                    time.sleep(0.01)
                except KeyboardInterrupt:
                    log.warn("Nothing to interrupt!")
                continue
            except Exception:
                log.error(traceback.format_exc())
                break

        log.warn("Server is shutting down")

        self.swank.handler.swank_quit_lisp()

        # to kill daemon
        def kill_server(s):
            s.shutdown()
        Thread(target=kill_server, args=(self.server,)).start()


class EuslimeServer(S.TCPServer, object):
    def __init__(self, server_address,
                 handler_class=EuslimeRequestHandler,
                 encoding='utf-8',
                 program='roseus',
                 loader='~/.euslime/slime-loader.l',
                 color=False):
        log.info("Starting server with encoding {} and color {}".format(
            encoding, color))
        self.encoding = encoding
        self.program = program
        self.loader = loader
        self.color = color

        super(EuslimeServer, self).__init__(server_address, handler_class)

        addr, port = self.server_address

    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.socket.settimeout(3)
        self.socket.bind(self.server_address)
        log.info('Serving on %s:%d', *self.socket.getsockname())


def serve(host='0.0.0.0', port=0, port_filename=str(), encoding='utf-8',
          program='roseus', loader='~/.euslime/slime-loader.l', color=False):
    server = EuslimeServer((host, port),
                           encoding=encoding,
                           program=program,
                           loader=loader,
                           color=color)

    host, port = server.socket.getsockname()

    # writing port number to file
    if port_filename:
        try:
            with open(port_filename, "w") as f:
                f.write("%s" % port)
        except Exception as e:
            log.error("Failed to write port number: %s" % str(e))

    try:
        server.serve_forever()
    except Exception:
        log.error(traceback.format_exc())
    finally:
        server.server_close()


if __name__ == '__main__':
    serve()
