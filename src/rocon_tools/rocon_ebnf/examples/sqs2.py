#!c:/python26/python.exe
#
# syntax:   sqs2.py  "/string to locate/" fileid 
import sys
import rocon_uri.rule_parser as rule_parser

rule=['sqs  ::=  parms  fileid ',
      'parms::=  sep car* sep ',
	  'sep  ::=  r"\S" ',
	  'car  ::=  r"." ^sep ',
	  'fileid::= r"\S"* ']
#
parms=' '.join(sys.argv[1:])

cmp=rp.match(rule,parms)
if cmp==None:
	print "Error in parsing:"
else:
	id=None
	try:
		id=open(cmp.fileid)
		for l in id.readlines():
			if l.find(cmp.car)>-1: 
				print l[:-1]
	except Exception,e:
		print e
	else:
		if id!=None: id.close()