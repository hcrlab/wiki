# Paper writing tips

## Each sentence on its own line
In your LaTeX source, put each sentence on its own line.
This makes it easy for people to collaborate using Git.
It avoids merge conflicts unless two people edit the same sentence.
It also makes it easy to see what changes were made using side-by-side text comparison tools like `git diff`.

## Split source across multiple files
You can split your source code across multiple files like so:

**main.tex**
```tex
\begin{document}
  \input{abstract}
  \input{intro}
\end{document}
```

**abstract.tex**
```tex
% !TEX root = main.tex
\begin{abstract}
  This is the abstract.
\end{abstract}
```

**intro.tex**
```tex
% !TEX root = main.tex
\section{Introduction}
This is the introduction.
```

## Makefile
Use this Makefile to help build LaTeX papers on Linux using `latexmk`.
To use it, just put the Makefile in the top level directory and type `make`.
```Makefile
# Name of the output file, e.g., jetson2017programming.pdf
NAME = jetson2017programming

all: latex cleancrap open

latex:
  latexmk -pdf main.tex -bibtex -jobname=$(NAME)

open:
  xdg-open $(NAME).pdf

clean: cleancrap cleanpdf
  
cleancrap:
  rm -f *.aux $(NAME).fdb_latexmk $(NAME).fls $(NAME).log $(NAME).bbl $(NAME).blg $(NAME).out

cleanpdf:
  rm -f $(NAME).pdf
```

## Title pages for reports
For your general exam, you may want to include a separate title page.
Here is an example of how to create a title page for a general exam.
Put it after `\begin{document}` and before your abstract.
```tex
\begin{titlepage}
  \begin{center}
    \vspace*{1cm}
    {\huge End-to-End User Programming of General-Purpose Robots}\\
    \vspace{1.5cm}
    PhD General Examination Report\\
    November 23, 2016\\
    \vspace{1 cm}
    {\large Rosie Jetson}\\
    Department of Computer Science \& Engineering\\
    University of Washington
    \vfill
    \textit{Supervisory Committee:}\\
    Member 1, Advisor\\
    Member 2\\
    Member 3\\
    Member 4, GSR\\
  \end{center}
\end{titlepage}
```
