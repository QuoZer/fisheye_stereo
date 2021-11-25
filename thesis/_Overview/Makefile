all: pdf pdf

pdf:
	pdflatex report.tex
	bibtex
	pdflatex report.tex

texmk: 
	latexmk -xelatex \
	-synctex=1 report.tex

clean:
	del -fr *.aux
	del -fr *.bbl
	del -fr *.blg
	del -fr *.log
	del -fr *.out
	del -fr *.toc
	del -fr *.dvi
	del -fr *.fls
	del -fr *.fdb_latexmk
	del -fr *.synctex.gz
	del -fr *.xdv
