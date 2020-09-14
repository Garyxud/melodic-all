#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import argparse

import euslime
from euslime.logger import get_logger, set_log_level, LOG_LEVELS
from euslime.server import serve

try:
    _input = raw_input
except ImportError:
    _input = input


log = get_logger(__name__)


def main():
    p = argparse.ArgumentParser(
        prog=euslime.__name__,
        version=euslime.__version__,
        description=euslime.__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--emacs-mode", action="store_true",
                   help="Launch with emacs mode")
    p.add_argument("--color", action="store_true",
                   help="Support colored output")
    p.add_argument("--init-file", "-i", type=str,
                   help="Initialization file",
                   default="~/.euslime/slime-loader.l")
    p.add_argument("--euslisp-program", "-e", type=str,
                   help="Backend Euslisp program",
                   default="roseus")
    p.add_argument("--host", type=str,
                   help="Host to serve",
                   default="0.0.0.0")
    p.add_argument("--port", "-p", type=int,
                   help="Port number to serve",
                   default=0)
    p.add_argument("--encoding", type=str,
                   help="Encoding for communication",
                   default="utf-8")
    p.add_argument("--port-filename", type=str,
                   help="Path to file where port number is written",
                   default=str())
    p.add_argument("--log-level", "-l", type=str,
                   help="Log Level", default="debug",
                   choices=LOG_LEVELS.keys())

    args = p.parse_args()

    if args.emacs_mode:
        log.info("Launched with emacs mode")
        init_string = _input("Waiting for initialization command...")
        args = p.parse_args(init_string.split())

    set_log_level(args.log_level)
    serve(host=args.host, port=args.port,
          port_filename=args.port_filename,
          encoding=args.encoding,
          program=args.euslisp_program,
          loader=args.init_file,
          color=args.color)


if __name__ == '__main__':
    main()
