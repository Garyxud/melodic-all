
doc: clean sphinx

sphinx:
	rosdoc_lite .

clean:
	rm -rf doc/html

topages:
	cp -r doc/html/* ~/tmp/rocon_tools/doc/html/rocon_interactions
	cp -r doc/html/.doctrees ~/tmp/rocon_tools/doc/html/rocon_interactions
	cp doc/html/.buildinfo ~/tmp/rocon_tools/doc/html/rocon_interactions
