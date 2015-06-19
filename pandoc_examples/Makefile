.PHONY: all clean

all: output_basic.html output_ieee.html output_jcsl.html output_ieee.md output_ieee.txt output_ieee.pdf output_ieee_strict.md output_ieee_github.md output_ieee_strict_smart.md output_ieee_strict_nocite.md

clean:
	rm output_*.*

output_basic.html: test.tex research.bib
	pandoc test.tex --standalone -o output_basic.html --bibliography research.bib

output_ieee.html: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -o output_ieee.html --bibliography research.bib --csl ieee.csl

output_ieee.md: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -o output_ieee.md -t markdown --bibliography research.bib --csl ieee.csl

output_ieee_strict.md: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -o output_ieee_strict.md -t markdown_strict --bibliography research.bib --csl ieee.csl

output_ieee_strict_nocite.md: test_nocite.tex research.bib ieee.csl
	pandoc test_nocite.tex --standalone -o output_ieee_strict_nocite.md -t markdown_strict --bibliography research.bib --csl ieee.csl

output_ieee_github.md: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -o output_ieee_github.md -t markdown_github --bibliography research.bib --csl ieee.csl

output_ieee_strict_smart.md: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -S -o output_ieee_strict_smart.md -t markdown_github --bibliography research.bib --csl ieee.csl

output_ieee.txt: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -o output_ieee.txt --bibliography research.bib --csl ieee.csl

output_ieee.pdf: test.tex research.bib ieee.csl
	pandoc test.tex --standalone -o output_ieee.pdf --bibliography research.bib --csl ieee.csl

output_jcsl.html: test.tex research.bib jcsl.csl
	pandoc test.tex --standalone -o output_jcsl.html --bibliography research.bib --csl jcsl.csl

