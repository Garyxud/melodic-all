#!c:/python26/python.exe
import sys,rp
rule=['init n=0',
      'init m=999',
      'sqs  ::=  range? parms  fileid ',
      'range::=  n "-" "*" ',
      '      |   n "-" m ',
	  'n    ::=  r"[0-9]"* ',
	  'm    ::=  r"[0-9]"* ',	  
      'parms::=  sep car* sep ',
	  'sep  ::=  r"\S" ',
	  'car  ::=  r"." ^sep ',
	  'fileid::= r"\S"* ']
parms=' '.join(sys.argv[1:])

cmp=rp.match(rule,parms)
if cmp==None:
	print "Error in parsing:"
else:
	id=None
	n=int(cmp.n)
	m=int(cmp.m)
	try:
		id=open(cmp.fileid)
		for l in id.readlines():
			if l.find(cmp.car,n,m)>-1: 
				print l[:-1]
	except Exception,e:
		print e
	else:
		if id!=None: id.close()