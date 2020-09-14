#!c:/python26/python.exe
import sys,rp

rule=['init sqs_range="" ',
      'sqs  ::=  SELECT "*" FROM fileid WHERE range operator value ',
      '                                 @sqs_fileid="$fileid"',
      '                                 @sqs_ope="$operator" ',
      
      'range::=  n "-" "*"              @sqs_range=[$n-1,999] ',
      '      |   n "-" m                @sqs_range=[$n-1,$m] ',
      '      |   n "." m                @sqs_range=[$n-1,($m+$n-1)] ',
      '      |   n "-"                  @sqs_range=[$n-1,999] ',
	  'n    ::=  r"[0-9]"*  ',
	  'm    ::=  r"[0-9]"* ',	
      'value::=  "\'" car* "\'"         @sqs_valtype=1  @sqs_value="$car" ',
      '      |   numeric                @sqs_valtype=2  @sqs_value=numeric ',
      'operator::=  LIKE ',
      '         |   "="  ',
      '         |   ">"  ',
      '         |   "<"  ',
	  'car  ::=  r"[^\']" ',
      'numeric::= r"[0-9]"* ',
	  'fileid::= r"\S"* ']
parms=' '.join(sys.argv[1:])

cmp=rp.match(rule,parms)
if cmp==None:
	print "Error in parsing:"
else:
    id=None
    try:
	   id=open(cmp.sqs_fileid)
	   for l in id.readlines():
            data=l[cmp.sqs_range[0]:cmp.sqs_range[1]]
            found=False
            if cmp.sqs_valtype==1:
                if cmp.sqs_ope.upper()=='LIKE':
                    if cmp.sqs_value.startswith('%'):
                        found=data.endswith(cmp.sqs_value[1:])
                    elif cmp.sqs_value.endswith('%'):
                        found=data.startswith(cmp.sqs_value[:-1])
                    else:
                        found=(data.find(cmp.sqs_value)>-1)
                elif cmp.sqs_ope=='=':
                    found=(data == cmp.sqs_value)
                else:
                    raise Exception('Operator not allowed with alpha')
            else:
                try:
                    v1=int(data.strip())
                    v2=int(cmp.sqs_value.strip())
                    if cmp.sqs_ope=='=': found=(v1==v2)
                    elif cmp.sqs_ope=='>': found=(v1>v2)
                    elif cmp.sqs_ope=='<': found=(v1<v2)
                except:
                    pass
            if found:
                print l[:-1]
    except Exception,e:
	    print e
    else:
        if id!=None: id.close()
        
