#!c:/python26/python.exe
import sys,rp

rule=['init sqs_selection=[] ',
      'sqs  ::=  SELECT selection FROM fileid WHERE column operator value ',
      '                                 @sqs_fileid="$fileid"',
      '                                 @sqs_column="$column"',
      '                                 @sqs_ope="$operator"',
      
      'selection::=  col0 cols*      ', 
      'col0     ::=  col                 @sqs_selection.append("$col")',  
      'cols     ::=  "," col             @sqs_selection.append("$col")',
      'col      ::=  r"[A-Za-z0-9]"*  ',
      'column   ::=  r"[A-Za-z0-9]"*  ',
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
        #
        #Read the first two lines that contain columns name & length
        #and prepare dictionary 'columns'
        colnames=[x for x in id.readline()[:-1].split(' ') if x!=""]
        collengths=id.readline()[:-1].split(' ')
        deb=0
        columns={}
        for i,name in enumerate(colnames):
           colsize=len(collengths[i])
           columns[name]=(deb,colsize+deb)
           deb+=colsize+1
        #
        #now loop on lines until end of file
        for l in id.readlines():
            indexes=columns[cmp.sqs_column]     #get indexes associated to column
            data=l[indexes[0]:indexes[1]]       #get the data of where condition
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
                txt=''
                for col in cmp.sqs_selection:
                    c=columns[col]
                    txt+=l[c[0]:c[1]]+' '
                print txt
    except Exception,e:
	    print e
    else:
        if id!=None: id.close()
        
