all: classref_en classref_jp

classref_en:
	mkdir -p ClassReference-en
	doxygen Doxyfile_en

classref_jp:
	mkdir -p ClassReference-jp
	doxygen Doxyfile_jp

clean:
	rm -rf ClassReference-en
	rm -rf ClassReference-jp
