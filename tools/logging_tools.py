# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import logging, time

# -- The custom logger for the task trees package
class CustomFormatter(logging.Formatter):
    """ The custom logger class for the package
    :meta private:
    """
    grey = '\x1b[38;20m'
    cyan ='\x1b[36;20m'
    yellow = '\x1b[33;20m'
    red = '\x1b[31;20m'
    bold_red = '\x1b[31;1m'
    reset = '\x1b[0m'
    # format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)'
    format = '[%(levelname)s] [%(created)16f]: %(message)s'
    FORMATS = {
        logging.DEBUG: cyan + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }
    def format(self, record):
        time_format = "%H:%M:%S %f"
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt=time_format)
        return formatter.format(record)
    def formatException(self, exc_info):
        result = super().formatException(exc_info)
        return repr(result)

def init_logger():
    logger = logging.getLogger('main')
    logger.setLevel(logging.INFO)
    logger.propagate = False
    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setFormatter(CustomFormatter())
        logger.addHandler(ch)
    return logger

logger = init_logger()

def spin():
    while True:
        time.sleep(1.0)
