#!/usr/bin/env python
# Copyright (c) 2020 Pilz GmbH & Co. KG

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import time
import sys
import psutil

OBSERVATION_FREQUENCY_HZ = .5
OBSERVATION_DURATION_SEC = 60
PSEN_SCAN_PROCESS_NAME = "psen_scan_node"
NAME = 'name'
THRESHOLD_PRECENT = 80
RETURN_CODE_SUCCESS = 0
RETURN_CODE_FAILURE = 10
RETURN_CODE_ERROR = 1

# ASCII codes for printing colored text to console
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
NC = '\033[0m'


def get_psen_scan_process():  # type: () -> psutil.Process
    """Return the handle of the psen_scan process if it is currently running,
    None otherwise."""
    for proc in psutil.process_iter([NAME]):
        if proc.info[NAME] == PSEN_SCAN_PROCESS_NAME:
            return proc
    raise RuntimeError("Process %s not found." % PSEN_SCAN_PROCESS_NAME)


if __name__ == "__main__":
    process = None
    while process is None:
        try:
            process = get_psen_scan_process()
        except RuntimeError:
            print("Waiting for %s process to be running." %
                  PSEN_SCAN_PROCESS_NAME)
        time.sleep(1. / OBSERVATION_FREQUENCY_HZ)
    print(YELLOW +
          "Process is running. Will observe its CPU usage now for %d seconds" %
          OBSERVATION_DURATION_SEC + NC)
    start_time = time.time()
    while time.time() - start_time < OBSERVATION_DURATION_SEC:
        try:
            process = get_psen_scan_process()
        except RuntimeError:
            print(RED + "Process not running any more. Aborting Test" + NC)
            sys.exit(RETURN_CODE_FAILURE)
        utilization = process.cpu_percent()
        if utilization < THRESHOLD_PRECENT:
            print("CPU utilization of >%s<: %.1f%%" %
                  (PSEN_SCAN_PROCESS_NAME, utilization))
        else:
            print(RED +
                  "CPU utilization of >%s<: %.1f%% " %
                  (PSEN_SCAN_PROCESS_NAME, utilization) +
                  "exceeds threshold of %d%%" %
                  THRESHOLD_PRECENT + NC)
            sys.exit(RETURN_CODE_ERROR)
        time.sleep(1. / OBSERVATION_FREQUENCY_HZ)
    print(GREEN + "CPU usage was below %d%% for %d seconds. " %
          (THRESHOLD_PRECENT, OBSERVATION_DURATION_SEC) +
          "Test successful." + NC)
    sys.exit(RETURN_CODE_SUCCESS)
