#!/bin/sh
awk 'BEGIN{for(i=0;i<16;++i){printf("%.6f\n%.6f %.6f %.6f %.6f %.6f\nmessage%d\n",i,rand(),rand(),rand(),rand(),rand(),i);}}' > original-data
