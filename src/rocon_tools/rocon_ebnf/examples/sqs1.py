#!c:/python26/python.exe
#
# first sample using rule
#
import sys
import rocon_uri.rule_parser as rule_parser
#
# we define the rule as a list of (sub)rules
#
# r"\S"*   means regular expression specifying 
#          any character except blank 
rule=['sqs  ::=  parms  fileid ', 
      'parms::=  r"\S"* ',        
      'fileid::= r"\S"* ']        
#
# we concatenate arguments ... as words
parms=' '.join(sys.argv[1:])
#
# we make the parsing 
cmp=rule_parser.rp.match(rule,parms)
# 
#as re module, if the result object is None, 
# the parsing is unsuccessful
if cmp==None:
	print "Error in parsing:"   
else:
    #
    # now, to get values from parsing, 
    # we use rule names as parser arguments.
    # cmp.sqs    will contain input parameters
    # cmp.parms  will contain string to locate
    # cmp.fileid will contain fileid to search in
	try:
		id=open(cmp.fileid)
		for l in id.readlines():
			if l.find(cmp.parms)>-1: 
				print l[:-1]
	except Exception,e:
		print e
	else:
		id.close()

