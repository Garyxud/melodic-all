import json
import logging
import logging.config
from logging import DEBUG, INFO, WARN, ERROR, FATAL  # NOQA


LOG_LEVELS = {
    "debug": DEBUG,
    "info": INFO,
    "warn": WARN,
    "error": ERROR,
    "fatal": FATAL,
}


_DEFAULT_CONFIG = {
    'version': 1,
    'root': {
        'level': 'DEBUG',
        'handlers': ['console'],
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'level': 'DEBUG',
            'formatter': 'simple',
        },
    },
    'formatters': {
        'simple': {
            'format': '[%(levelname)s - %(module)s:L%(lineno)s] %(message)s',
        },
    },
}

_LOG_CONFIGURED = False
_LOGGERS = {}


def get_logger(ns=__name__, cfg_path=None):
    global _LOG_CONFIGURED
    if _LOG_CONFIGURED is False:
        try:
            with open(cfg_path, "r") as f:
                cfg = json.load(f, encoding='utf-8',
                                parse_int=True, parse_float=True)
                logging.config.dictConfig(cfg)
        except:
            logging.config.dictConfig(_DEFAULT_CONFIG)
        _LOG_CONFIGURED = True
    if ns not in _LOGGERS:
        _LOGGERS[ns] = logging.getLogger(ns)
    return _LOGGERS[ns]


def set_log_level(level):
    if isinstance(level, str):
        level = LOG_LEVELS[level]
    for l in _LOGGERS.values():
        l.setLevel(level)
