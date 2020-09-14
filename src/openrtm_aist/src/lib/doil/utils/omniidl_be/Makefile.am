## -*- Makefile -*-
##------------------------------------------------------------
## Makefile.am for libdoil
##
## $Id$
##------------------------------------------------------------

noinst_DATA = __init__.py

__init__.py:
	echo "#!/usr/bin/env python" > createinit.py
	echo "import sys"            >> createinit.py
	echo "import omniidl_be"     >> createinit.py
	echo "path0 = '.'"   >> createinit.py
	echo "path1 = omniidl_be.__path__[0]" >> createinit.py
	echo "print \"__path__ = ['%s', '%s']\" % (path0, path1)" >> createinit.py
	chmod 755 createinit.py
	./createinit.py `pwd` > __init__.py
	rm ./createinit.py

clean-local:
	find ./ -name '*~' -exec rm {} \;
	find ./ -name '*.pyc' -exec rm {} \;
	rm -f createinit.py
	rm -f __init__.py
