% kjpaper.cls -- single-column preprint class (J. Kang).
% Layout modelled on the typographic conventions of arXiv:2609.19644
% (11pt A4, Charter text + newtx math, flush-left title, bold abstract,
% ruled running head/foot). Independent re-implementation; no logos.
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{kjpaper}[2026/09/28 v1.0 single-column preprint]
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\LoadClass[11pt,a4paper]{article}
\RequirePackage[utf8]{inputenc}
\RequirePackage[T1]{fontenc}
\RequirePackage[english]{babel}
\RequirePackage{amsmath,amssymb,amsthm}
\RequirePackage{XCharter}
\RequirePackage[xcharter,bigdelims,vvarbb]{newtxmath}
\RequirePackage[scaled=1.1]{zlmtt}
\RequirePackage[left=2.2cm,right=2.2cm,top=3cm,bottom=3cm,
  headheight=40pt,headsep=20pt,a4paper]{geometry}
\RequirePackage[dvipsnames]{xcolor}
\RequirePackage{graphicx,booktabs,multirow,tabularx,colortbl,caption,subcaption}
\RequirePackage{enumitem,changepage,needspace,fancyhdr,url}
\RequirePackage[tracking=smallcaps]{microtype}
\RequirePackage{authblk}
\RequirePackage[ruled,vlined,linesnumbered]{algorithm2e}
\RequirePackage{tikz,pgfplots}
\pgfplotsset{compat=1.18}
\RequirePackage[colorlinks=true,allcolors=blue]{hyperref}
\RequirePackage{cleveref}
\setlength{\parskip}{0.5\baselineskip}
\setlength{\parindent}{0pt}
\setlist{itemsep=2pt,topsep=2pt}
\captionsetup{font=small,labelfont=bf}
% theorem-like environments
\newtheorem{theorem}{Theorem}
\newtheorem{proposition}{Proposition}
\newtheorem{lemma}{Lemma}
\newtheorem{corollary}{Corollary}
\theoremstyle{definition}
\newtheorem{definition}{Definition}
\newtheorem{assumption}{Assumption}
\theoremstyle{remark}
\newtheorem{remark}{Remark}
% fonts for title block
\newcommand{\titlefont}{\normalfont\bfseries\fontsize{21}{23}\selectfont}
\newcommand{\absfont}{\bfseries\linespread{1.2}\fontsize{10}{12}\selectfont}
\newcommand{\footerfont}{\normalfont\fontsize{8}{10}\selectfont}
\setcounter{Maxaffil}{0}
\setlength{\affilsep}{0.2em}
\renewcommand\Authfont{\normalfont\bfseries\fontsize{9}{13}\selectfont}
\renewcommand\Affilfont{\normalfont\fontsize{8}{10}\selectfont}
% metadata
\newtoks\correspondingauthor
\newtoks\paperurl
\def\@keywords{}
\newcommand{\keywords}[1]{\def\@keywords{#1}}
% running head/foot
\pagestyle{fancy}
\fancyhf{}
\fancyhead[C]{\footerfont\@title}
\fancyfoot[R]{\footerfont\thepage}
\renewcommand{\headrulewidth}{1pt}
\renewcommand{\footrulewidth}{1pt}
\let\kj@headrule\headrule \renewcommand{\headrule}{\color{gray}\kj@headrule}
\let\kj@footrule\footrule \renewcommand{\footrule}{\color{gray}\kj@footrule}
\fancypagestyle{firststyle}{%
  \fancyhf{}%
  \fancyhead[R]{\if\relax\the\paperurl\relax\else\href{\the\paperurl}{\footerfont\itshape\the\paperurl}\\\fi
    {\footerfont\itshape Preprint, \today}}%
  \fancyfoot[L]{\if\relax\the\correspondingauthor\relax\else
    \footerfont\itshape Corresponding author: \the\correspondingauthor\fi}%
  \fancyfoot[R]{\footerfont\thepage}}
% abstract captured and typeset inside \maketitle
\renewenvironment{abstract}{\global\setbox\kj@absbox\vbox\bgroup\hsize\linewidth\noindent\absfont}{\egroup}
\newsavebox{\kj@absbox}
\renewcommand{\maketitle}{\bgroup\setlength{\parindent}{0pt}%
  \begin{adjustwidth}{0pt}{24pt}\begin{flushleft}
    {\raggedright\titlefont\@title\par}\vskip11pt
    {\raggedright\@author\par}\vskip20pt
  \end{flushleft}\end{adjustwidth}
  \noindent\usebox{\kj@absbox}\par
  \ifx\@keywords\@empty\else\vskip1em\noindent{\itshape Keywords: \@keywords}\par\fi
  \vskip10pt\noindent{\color{gray}\rule{\linewidth}{0.6pt}}\par\vskip6pt
  \thispagestyle{firststyle}\suppressfloats[t]\egroup}
% section headings: bold, 13pt
\RequirePackage{titlesec}
\titleformat{\section}{\normalfont\bfseries\fontsize{13}{14}\selectfont}{\thesection.}{0.5em}{}
\titleformat{\subsection}{\normalfont\bfseries\fontsize{11}{13}\selectfont}{\thesubsection.}{0.5em}{}
\titleformat{\subsubsection}{\normalfont\bfseries\itshape}{\thesubsubsection.}{0.5em}{}
\newcommand{\stdv}[2][\tiny]{#1\text{$\pm$}#2}
\endinput
