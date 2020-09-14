#!c:/python26/python.exe
import sys,rp
rule=['init sqs_ranges=[] ',
      'init sqs_locate="" ',
      'sqs  ::=  ranges? parms  fileid   @sqs_fileid="$fileid" ',
      'ranges::=  "(" range+ ")"  ',
	  '       |   range ',
      '# for ranges, we appended two other types',
      '# and columns start at 1 (not 0), that explains the $n-1 ', 
      'range::=  n "-" "*"              @sqs_ranges.append([$n-1,999]) ',
      '      |   n "-" m                @sqs_ranges.append([$n-1,$m]) ',
      '      |   n "." m                @sqs_ranges.append([$n-1,($m+$n-1)]) ',
      '      |   n "-"                  @sqs_ranges.append([$n-1,999]) ',
	  'n    ::=  r"[0-9]"*  ',
	  'm    ::=  r"[0-9]"* ',	  
      'parms::=  sep car* sep           @sqs_locate="$car"',
	  'sep  ::=  r"\S" ',
	  'car  ::=  r"." ^sep ',
	  'fileid::= r"\S"* ']
parms=' '.join(sys.argv[1:])

cmp=rp.match(rule,parms)
if cmp==None:
	print "Error in parsing:"
else:
    id=None
    try:
        #
        #Opening...
        id=open(cmp.sqs_fileid)
        for l in id.readlines():
            #
            #if no ranges defined, then all the record
            if len(cmp.sqs_ranges)==0:
                data=l
            else:
                data=''
            #
            #Now, for each range, create the record
            for n,m in cmp.sqs_ranges:
                data+=l[n:m]
            if data.find(cmp.sqs_locate)>-1:
	           print l[:-1]
    except Exception,e:
	   print e
    else:
	   if id!=None: id.close()