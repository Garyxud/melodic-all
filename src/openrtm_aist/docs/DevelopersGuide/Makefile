
all: DevelopersGuide.dvi DevelopersGuideNoClassRef.dvi DevelopersGuide.ps DevelopersGuide.pdf

DevelopersGuide.ps: DevelopersGuide.dvi
	dvips DevelopersGuide.dvi > DevelopersGuide.ps
	cp DevelopersGuide.ps ../

DevelopersGuide.pdf: DevelopersGuide.dvi
	dvipdf DevelopersGuide.dvi
	cp DevelopersGuide.pdf ../

DevelopersGuide.dvi: DevelopersGuide.tex
	rm -f ClassReference/*
	doxygen classref.conf
	ls ClassReference/*.tex | awk -f dox.awk |sh
	rm -f IDLReference/*
	doxygen idlref.conf
	ls IDLReference/*.tex | awk -f dox.awk |sh
	platex DevelopersGuide.tex
	makeindex DevelopersGuide.idx
	platex DevelopersGuide.tex
	while egrep -s 'Rerun (LaTeX|to get cross-references right)' refman.log && [ $$latex_count -gt 0 ] ;\
	    do \
	      echo "Rerunning latex...." ;\
	      platex DevelopersGuide.tex ;\
	      latex_count=`expr $$latex_count - 1` ;\
	    done

DevelopersGuideNoClassRef.dvi: DevelopersGuideNoClassRef.tex
	platex DevelopersGuideNoClassRef.tex
	makeindex DevelopersGuideNoClassRef.idx
	platex DevelopersGuideNoClassRef.tex
	while egrep -s 'Rerun (LaTeX|to get cross-references right)' refman.log && [ $$latex_count -gt 0 ] ;\
	    do \
	      echo "Rerunning latex...." ;\
	      platex DevelopersGuideNoClassRef.tex ;\
	      latex_count=`expr $$latex_count - 1` ;\
	    done

clean:
	rm -rf ClassReference/* IDLReference/* *~ *.aux *.log *.idx *.toc *ind *.ilg DevelopersGuide.dvi DevelopersGuide.ps DevelopersGuide.pdf DevelopersGuideNoClassRef.dvi DevelopersGuideNoClassRef.ps DevelopersGuideNoClassRef.pdf
