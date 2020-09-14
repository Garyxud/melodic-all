from __future__ import with_statement
import threading

class Multiset:
    def __init__(self):
        self.elements = {}
        self.mutex = threading.Lock()
        self.__iter__ = self.elements.iterkeys
        self.__contains__ = self.elements.__contains__

    def add(self, k):
        with self.mutex:
            if k in self.elements:
                self.elements[k] += 1
            else:
                self.elements[k] = 1

    def remove(self, k):
        with self.mutex:
            if self.elements[k] > 0:
                self.elements[k] -= 1
            else:
                del self.elements[k]
