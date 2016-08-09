all: clsquare-whitepaper.pdf

%.pdf: %.tex
	pdflatex $*
	bibtex $*
	makeindex $*
	pdflatex $*
	pdflatex $*

clean:
	rm -f *.aux *.bbl *.blg *.idx *.ilg *.ind *.log *.out *.toc

veryclean: clean
	rm -f *.pdf
