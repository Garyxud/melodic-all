BASE_URL=http://files.yujinrobot.com/images/templates
FILES="bubble_template.xcf" "bubble_template_highres.xcf" 

templates:
	@for i in ${FILES}; do \
	    [ ! -f $${i} ] && wget ${BASE_URL}/$${i} || echo "Already downloaded, skipping.......$${i}"; \
	done

clean:
	@for i in ${FILES}; do \
		rm -f $${i}*; \
	done
	
