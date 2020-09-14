#!/bin/bash

rosrun euscollada collada2eus `rospack find darwin_op_description`/collada/darwin.dae darwin.yaml darwin.l
if [ "$?" != 0 ] ;  then exit ; fi

rosrun euslisp irteusgl -e "(progn (load \"darwin.l\")(darwin)(make-irtviewer)(objects (list *darwin*)))"
