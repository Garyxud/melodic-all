#!c:/python26/python.exe
import sys,rp

def process(data,cols,col,ope,val):
    ret=False
    indexes=cols[col]
    data=data[indexes[0]:indexes[1]]
    #print data,ope,val
    if isinstance(val,int):
        exec("ret=("+data+ope+str(val)+")")
    else:
        if ope.upper()=='LIKE':
            if val.startswith('%'):
                ret=data.endswith(val[1:])
            elif val.endswith('%'):
                ret=data.startswith(val[:-1])
            else:
                ret=(data.find(val)>-1)
        elif ope=='=':
            ret=(data == val)            
    return ret

rule=['option norun',            #norun, means that lines of code 
                                 #will not be interpreted while parsing string
      'init sqs_selection=[] ',
      'init sqs_cond="" ',
      'sqs  ::=  SELECT selection FROM fileid WHERE condition ',
      '                                  @sqs_fileid="$fileid"',
      'selection::=  col0 cols*      ', 
      'col0     ::=  col                 @sqs_selection.append("$col")',  
      'cols     ::=  "," col             @sqs_selection.append("$col")',
      'col      ::=  r"[A-Za-z0-9]"*  ',

	  'fileid::= r"\S"* ',
      
      'condition ::=  cond1  cond2* ',
      'cond2     ::=  op_or  cond1  ',
      'op_or     ::=  OR                 @sqs_cond+=" or "',
      'cond1     ::=  cond3 cond4*  ',
      'cond4     ::=  op_and  cond3 ',
      'op_and    ::=  AND                @sqs_cond+=" and "',
      'cond3     ::=  column operator value  ',
      '@sqs_cond+= "process(data,cols,\'$column\',\'$operator\'," ',
      '@if sqs_valtype==1:',
      '@   sqs_cond+="\'"+sqs_value+"\'"',
      '@else:',
      '@   sqs_cond+=str(sqs_value)',
      '@sqs_cond+=")"',
      '           |   leftpar condition rightpar  ',
      '           |   condition ',
      'column   ::=  r"[A-Za-z0-9]"*  ',
      'leftpar   ::=  "("                @sqs_cond+=" ( " ',
      'rightpar  ::=  ")"                @sqs_cond+=" ) " ',
      'operator  ::=  LIKE ',
      '           | "=" ',
      '           | "<=" ',
      '           | ">=" ',
      '           | "<" ',
      '           | ">" ',
      'value     ::=  "\'" car* "\'"        @sqs_valtype=1  @sqs_value="$car" ',
      '           |   numeric               @sqs_valtype=2  @sqs_value=$numeric ',
      'car       ::=  r"[^\']" ',
      'numeric   ::=  r"[0-9]"* ' ]
parms=' '.join(sys.argv[1:])

cmp=rp.match(rule,parms)
if cmp==None:
	print "Error in parsing:"
else:
    #
    #cmp.code contains the complete lines of code of the parsing
    #so, we need to exec it in order to have values set.
    #After that, to access values, no need to prefix with cmp. 
    """
    print '----- the code -----'
    print cmp.code
    print '--------------------'
    """
    exec(cmp.code)
    id=None
    try:
        id=open(sqs_fileid)
        #
        #Read the first two lines that contain columns name & length
        #and prepare dictionary 'columns'
        colnames=[x for x in id.readline()[:-1].split(' ') if x!=""]
        collengths=id.readline()[:-1].split(' ')
        deb=0
        cols={}
        for i,name in enumerate(colnames):
           colsize=len(collengths[i])
           cols[name]=(deb,colsize+deb)
           deb+=colsize+1
        #
        #now loop on lines until end of file
        for data in id.readlines():
            """
            print '..... sqs_cond .....'
            print sqs_cond
            print '....................'
            """
            exec("found="+sqs_cond)
            if found:
                txt=''
                for col in sqs_selection:
                    c=cols[col]
                    txt+=data[c[0]:c[1]]+' '
                print txt
    except Exception,e:
	    print e
    else:
        if id!=None: id.close()
        
