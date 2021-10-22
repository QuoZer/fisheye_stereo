all: build pdf

build:
	latexmk -xelatex \
	-synctex=1 main.tex
pdf:
	pdflatex main.tex

clean:
	rm -f *.aux
	rm -f *.bbl
	rm -f *.blg
	rm -f *.log
	rm -f *.out
	rm -f *.pdf
	rm -f *.toc
