# Pandoc

[Pandoc](http://pandoc.org) is a tool for converting between text formats and generating text programmatically.

One really useful thing it can do is consume **BibTex** and produce **markdown**.

This directory includes a bunch of examples of output formats that Pandoc can produce. To re-make the examples in this directory:

1. Install pandoc.
2. Run `make clean && make all`.

[This](http://tex.stackexchange.com/questions/171793/bibtex-to-html-markdown-etc-using-pandoc) page provides a concise explanation of how to produce markdown from BibTex.

We start with a BibTex file ([research.bib](research.bib)):
```bibtex
@inproceedings{TBBRD13Synthesis,
  title={Device driver synthesis for embedded systems},
  author={Tanguy, Julien and Béchennec, Jean-Luc and Briday, Mikaël and Roux, Olivier H. and Dubé, Sébastien},
  year= {2013},
  booktitle={Proceedings of 2013 IEEE 18th International Conference on
             Emerging Technologies {\&} Factory Automation},
  organization={IEEE},
  url={http://www.etfa2013.org}
}

@inproceedings{newcombe2015dynamicfusion,
    title={DynamicFusion: Reconstruction and Tracking of Non-rigid Scenes in Real-Time},
    author={Newcombe, Richard A and Fox, Dieter and Seitz, Steven M},
    booktitle={Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition},
    pages={343--352},
    year={2015}
}
```

and a LaTeX file describing the document we want to produce ([test_nocite.tex](test_nocite.tex)):
```latex
\documentclass{article}
\usepackage{biblatex}
\bibliography{mybib}

\begin{document}
\nocite{*}
\printbibliography
\end{document}
```

Optionally, we can also include a csl file (see [ieee.csl](ieee.csl)) to describe how the bibliography entries should be rendered.

Finally we can run:
```bash
pandoc test_nocite.tex --standalone -o output_ieee_strict_nocite.md -t markdown_strict --bibliography research.bib --csl ieee.csl
```

Which will produce the following markdown ([output_ieee_strict_nocite.md](output_ieee_strict_nocite.md)):
```markdown
\[1\] J. Tanguy, J.-L. Béchennec, M. Briday, O. H. Roux, and S. Dubé,
“Device driver synthesis for embedded systems,” in *Proceedings of 2013
iEEE 18th international conference on emerging technologies & factory
automation*, 2013.

\[2\] R. A. Newcombe, D. Fox, and S. M. Seitz, “DynamicFusion:
Reconstruction and tracking of non-rigid scenes in real-time,” in
*Proceedings of the iEEE conference on computer vision and pattern
recognition*, 2015, pp. 343–352.
```

Which will be rendered like this:

---

\[1\] J. Tanguy, J.-L. Béchennec, M. Briday, O. H. Roux, and S. Dubé,
“Device driver synthesis for embedded systems,” in *Proceedings of 2013
iEEE 18th international conference on emerging technologies & factory
automation*, 2013.

\[2\] R. A. Newcombe, D. Fox, and S. M. Seitz, “DynamicFusion:
Reconstruction and tracking of non-rigid scenes in real-time,” in
*Proceedings of the iEEE conference on computer vision and pattern
recognition*, 2015, pp. 343–352.

---

Take a look at [Makefile](Makefile) for more examples.

**Note**: This bibliography example does not seem to work correctly for plain markdown (`-t markdown`).
