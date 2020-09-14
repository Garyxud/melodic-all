#!/usr/bin/env python

import sys
import os

def destroy_and_make(dirname):
  print "remake : dirname : ", dirname

  try:
#    os.rmdir(os.path.abspath(dirname))
    os.rename(dirname, dirname + '_')
  except:
    print "Directory \"" + dirname + "\" not found."


  try:
    os.system("./setuptest.py " + dirname + " ")
  except:
    print "Directory \"" + dirname + "\" already exists."




dirname = sys.argv[1]
#print  os.path.abspath(dirname)

if dirname != 'all':
  print "Usage : " + sys.argv[0] + " all";
  exit(1)

for pwd, dirs, files in os.walk('.'):
#  print "dirs : ", dirs
  break

for d in dirs:
  if d.rfind("Servant") > 0:
    print d, d.rfind("Servant")
    destroy_and_make(d)


#destroy_and_make(dirname)

exit(0)

