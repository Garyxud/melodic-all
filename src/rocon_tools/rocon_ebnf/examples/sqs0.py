#!c:/python26/python.exe
#
# Simulate standard grep function.
#
# syntax:  sqs0.py  string_to_locate  fileid  
#
import sys                      #sys module for argv 

search,fileid=sys.argv[1:3]     #get the two words passed in parameter 

try:                
	id=open(fileid)             #Open the file
	for l in id.readlines():	#loop on lines 
		if l.find(search)>-1:   #if locate string, 
			print l[:-1]        #   then print the line 
except Exception,e:
	print e
else:
	id.close()

