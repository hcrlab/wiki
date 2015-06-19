# Pandoc

[Pandoc](http://pandoc.org) is a tool for converting between text formats and generating text programmatically.

One really useful thing it can do is consume **BibTex** and produce **markdown**.

This directory includes a bunch of examples of output formats that Pandoc can produce. To re-make the examples in this directory:

1. Install pandoc.
2. Run `make clean && make all`.

[This](http://tex.stackexchange.com/questions/171793/bibtex-to-html-markdown-etc-using-pandoc) page provides a concise explanation of how to produce markdown from BibTex.

We start with a BibTex file ([research.bib]):
```
@inproceedings{TBBRD13Synthesis,
  title={Device driver synthesis for embedded systems},
  author={Tanguy, Julien and Béchennec, Jean-Luc and Briday, Mikaël and Roux, Olivier H. and Dubé, Sébastien},
  year= {2013},
  booktitle={Proceedings of 2013 IEEE 18th International Conference on
             Emerging Technologies {\&} Factory Automation},
  organization={IEEE},
  url={http://www.etfa2013.org}
}
```

and a LaTeX file describing the document we want to produce (`test.tex`):
```
\documentclass{article}
\usepackage{biblatex}
\bibliography{mybib}

\begin{document}
\autocite{TBBRD13Synthesis}
\printbibliography
\end{document}
```

Optionally, we can also include a csl file (see `ieee.csl`) to describe how the bibliography entries should be rendered.

Finally we can run:
```
pandoc test.tex --standalone -o output_ieee_strict.md -t markdown_strict --bibliography research.bib --csl ieee.csl
```

Take a look at `Makefile` for more examples.

**Note**: This bibliography example does not seem to work correctly for plain markdown (`-t markdown`).
