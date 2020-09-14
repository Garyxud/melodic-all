import logging
import os 
import threading
import sys
import time

logdir = '/var/log/roam'
logfilecount = 10
datefmt="%c %s" 

class MyFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        t = time.strftime("%Y-%m-%d %H:%M:%S / %s", ct)
        s = "%s.%03d" % (t, record.msecs)
        return s
                                

file_formatter = MyFormatter('%(asctime)s - %(levelname)s - %(message)s', datefmt=datefmt)

class LoggerStream:
    def __init__(self, func):
        self.lock = threading.Lock()
        self.buffer = ""
        self.func = func
        self.progress = None

    def write(self, str):
        if self.progress and str.partition('\n')[0].endswith(self.progress):
            return
        with self.lock:
            self.buffer += str
            while True:
                pos = self.buffer.find('\n')
                if pos == -1:
                    break
                self.progress = self.buffer[0:pos]
                self.func(self.progress)
                self.progress = None
                self.buffer = self.buffer[pos+1:]

    def flush(self):
        if self.buffer:
            with self.lock:
                self.func(self.buffer)

all_logger = logging.getLogger('')
all_logger_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'all.log'), when='midnight', backupCount=logfilecount)
all_logger_formatter = MyFormatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
all_logger_handler.setFormatter(all_logger_formatter)
all_logger_handler.setLevel(logging.DEBUG)
all_logger.addHandler(all_logger_handler)

console_logger = logging.getLogger('console')
console_logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
console_formatter = MyFormatter('%(message)s')
console_handler.setFormatter(console_formatter)
console_logger.addHandler(console_handler)

console_file_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'console-output.log'), when='midnight', backupCount=logfilecount)
console_file_handler.setFormatter(file_formatter)
console_file_handler.setLevel(logging.DEBUG)
console_logger.addHandler(console_file_handler)

sys.stdout = LoggerStream(console_logger.info)
sys.stderr = LoggerStream(console_logger.error)

def get_logger_stream(name, level = logging.DEBUG):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    return LoggerStream(logger.info)

def make_logger_file(name, level = logging.DEBUG):
    file_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir, name), when='midnight', backupCount=logfilecount)
    file_handler.setFormatter(file_formatter)
    file_handler.setLevel(level)
    logger = logging.getLogger(name)
    logger.addHandler(file_handler)

def get_logger_stream_for_file(name, level = logging.DEBUG):
    make_logger_file(name, level)
    return get_logger_stream(name, level)
