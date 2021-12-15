all: pdf pdf clean

pdf:
	pdflatex report.tex
	bibtex refs.bib
	pdflatex report.tex

texmk: 
	latexmk -xelatex \
	-synctex=1 report.tex

	pdflatex report.tex

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
