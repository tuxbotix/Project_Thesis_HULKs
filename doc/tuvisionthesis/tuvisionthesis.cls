%%
%% This is file `tuvisionthesis.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% tuvisionthesis.dtx  (with options: `class')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from tuvisionthesis.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file tuvisionthesis.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
\NeedsTeXFormat{LaTeX2e}[1997/06/01]
\def\filedate{2007/10/25}
\def\fileversion{0.19}
\def\filename{tuvisionthesis.cls}
\ProvidesClass{tuvisionthesis}[\filedate\space v\fileversion\space tuvisionthesis class]
\RequirePackage{ifthen}
\RequirePackage{ifpdf}
\newcommand\thecoauthor {}
\newcommand*{\coauthor}[1]{\renewcommand\thecoauthor{#1}}
\newcommand\thecategory {}
\newcommand*{\category}[1]{\renewcommand\thecategory{#1}}

\newcommand\insertthesisfootnote{}
\newcommand*{\thesisfootnote}[1]{\renewcommand\insertthesisfootnote{#1}}

\newcommand\theremark {}
\newcommand{\remark}[1]{\renewcommand\theremark{#1}}
\newcommand\thepreamblefile{}
\newcommand{\preamblefile}[1]{\renewcommand\thepreamblefile{#1}}
\newcommand\werkstyp{}
\DeclareOption{Studienarbeit}{
\renewcommmand\werkstyp{Studienarbeit}
}
\DeclareOption{Diplomarbeit}{
\renewcommand\werkstyp{Diplomarbeit}
}
\DeclareOption{Skript}{
\renewcommand\werkstyp{Skript zur Vorlesung}
}

\newif\ifmitschrift
\DeclareOption{Mitschrift}{
\renewcommand\werkstyp{Mitschrift der Vorlesung}
\mitschrifttrue
}
\DeclareOption{projectthesis}{
\renewcommand\werkstyp{Project Thesis}
}
\DeclareOption{diplomathesis}{
\renewcommand\werkstyp{Diploma Thesis}
}

\ifmitschrift{
\newcommand{\theremark{
Anmmerkung:
Der Stoffumfang ist noch nicht vollständig enthalten,
 und es können sich auch noch Fehler eingeschlichen haben.
Fehlerhinweise bitte an: grigat@tu-harburg.de }}
}
\fi
\newif\ifarticle
\newif\ifnotitle
\newif\ifnotoc
\newif\ifnoloflot
\newif\ifnomenclature
\DeclareOption{article}{\articletrue \notoctrue \notitletrue}
\newif\ifbook
\DeclareOption{book}{%
\ifarticle
\ClassError{tuvisionthesis}{You cannot use option book together with article}{Remove either book or article (no option = report = default)}
\else
\booktrue
\fi
}
\DeclareOption{notitle}{\notitletrue}
\DeclareOption{title}{\notitlefalse}
\DeclareOption{dvipdfm}{%
\ClassWarning{tuvisionthesis}{Option dvipdfm is obsolete}{dvipdfm is added automatically now}
}
\ifpdf
\else
  \ifarticle
   \PassOptionsToClass{dvipdfm}{article}
  \else
   \ifbook
   \PassOptionsToClass{dvipdfm}{book}
   \else
   \PassOptionsToClass{dvipdfm}{report}
   \fi
  \fi
  \typeout{Use dvipdfmx to convert dvi to pdf}
\fi
\newif\ifenglish
\DeclareOption{english}{\englishtrue}
\newif\ifngerman
\DeclareOption{ngerman}{\ngermantrue}
\newif\ifamerican
\DeclareOption{american}{\americantrue}
\newif\ifprintversion
\DeclareOption{printversion}{\printversiontrue}
\DeclareOption{notoc}{\notoctrue}
\DeclareOption{noloflot}{\noloflottrue}
\DeclareOption{nomenclature}{\nomenclaturetrue}
\newif\iftwoside
\DeclareOption{twoside}{
\twosidetrue
\ifarticle
\PassOptionsToClass{twoside}{article}
\else
\ifbook
\PassOptionsToClass{twoside}{book}
\else
\PassOptionsToClass{twoside}{report}
\fi
\fi
}
\ExecuteOptions{fleqn}

\ProcessOptions
\ifarticle
 \LoadClass[a4paper, 12pt]{article}
\else
  \ifbook
  \LoadClass[a4paper, 12pt]{book}
  \else
  \LoadClass[a4paper, 12pt]{report}
  \fi
\fi
\RequirePackage{lmodern}
\RequirePackage[T1]{fontenc}
\RequirePackage[latin9]{inputenc}
\RequirePackage[nottoc]{tocbibind}
\ifnomenclature

\ifngerman
\usepackage[intoc, german]{nomencl}
\else
\usepackage[intoc]{nomencl}
\renewcommand{\nomname}{List of Symbols}
\fi
\makenomenclature
\fi
\RequirePackage[a4paper]{geometry}
\RequirePackage{mathptmx}
\RequirePackage{courier}
\RequirePackage[scaled]{luximono}
\RequirePackage[english, ngerman]{babel}
\RequirePackage{amsmath}
\RequirePackage{dsfont}
\RequirePackage{amssymb}
\RequirePackage{amsfonts}
\RequirePackage{keyval}
\RequirePackage[override]{xcolor}
\RequirePackage{float}
\RequirePackage{afterpage}
\RequirePackage{enumerate}
\RequirePackage{fancyhdr}
\ifpdf
\RequirePackage{graphicx}
\DeclareGraphicsExtensions{.pdf,.png,.jpg,.jpeg}
\else
\RequirePackage{bmpsize}
\RequirePackage{graphicx}
\DeclareGraphicsExtensions{.eps,.pdf,.png,.jpg,.jpeg}
\fi
\setkeys{Gin}{keepaspectratio=true}
\widowpenalty=1000
\clubpenalty=1000
\setcounter{secnumdepth}{3}
\renewcommand{\textfraction}{0}
\renewcommand{\topfraction}{0.75}
\renewcommand{\bottomfraction}{0.75}
\renewcommand{\floatpagefraction}{0.8}
\restylefloat{float}
\pagestyle{fancy}
\ifarticle
\else
\renewcommand{\chaptermark}[1]{\markright{\chaptername {} \thechapter:  #1}{}}
\fi
\renewcommand{\sectionmark}[1]{\markright{\thesection\ #1}}

\iftwoside
\fancyhead[RE,LO]{\let\uppercase\relax\rightmark}
\fancyhead[C]{}
\fancyhead[LE,RO]{\thepage}
\else
\fancyhead[L]{\let\uppercase\relax\rightmark}
\fancyhead[C]{}
\fancyhead[R]{\thepage}
\fi
\lfoot{}
\cfoot{\insertthesisfootnote}
\rfoot[]{}%
\addtolength{\headheight}{1cm}  % Headerhöhe vergrößern, sonst Fehlermeldung
\ifprintversion
\RequirePackage[pdfpagelabels,plainpages=false, bookmarks=true,%
bookmarksopen=true, colorlinks=false, pageanchor=false]{hyperref}
\else
\RequirePackage[pdfpagelabels,plainpages=false, bookmarks=true,%
bookmarksopen=true, colorlinks=true, pageanchor=false]{hyperref}
\fi
\newcommand{\correctlanguage}{%
\ifenglish%
\selectlanguage{english}%
\fi%
\ifngerman%
\selectlanguage{ngerman}%
\fi%
\ifamerican%
\selectlanguage{american}%
\fi%
}
\newcommand{\tuvisionheading}{%
\correctlanguage
%%\setcounter{page}{0}
\begin{titlepage}%
\begin{center}%
\LARGE%
\ifprintversion
\includegraphics[width=5cm]{TUVision_print}\par
\else
\includegraphics[width=5cm]{TUVision_print}\par
\fi
\vspace{1ex}
Technische Universität Hamburg\par
Vision Systems\par
\vspace{5mm}
Prof. Dr.-Ing. R.-R. Grigat\par
\vspace{1cm}


{\huge \bf \@title\par
\vspace{1cm}
\Large
\thecategory\par
\vspace{1cm}
\@author\par}
{\large \thecoauthor \par}
\vspace{6mm}
\Large
\@date\par


\vspace{1cm}


\vfill
\normalsize
\theremark

\vfill
\ifprintversion
\includegraphics[width=7cm]{TUHH_print}\par
\else
\includegraphics[width=7cm]{TUHH_print}\par
\fi

\end{center}
\end{titlepage}
\clearpage{\thispagestyle{empty}\cleardoublepage} % sicherstellen, daß Seitenumbruch
\pagenumbering{roman}
\InputIfFileExists{\thepreamblefile}
{\ClassInfo{tuvisionthesis}{loading \thepreamblefile}}
{\ClassInfo{tuvisionthesis}{\thepreamblefile not found or not used}}
\clearpage{\thispagestyle{empty}\cleardoublepage}
\correctlanguage

\ifnotoc
\else
\tableofcontents                 % gibt Inhaltsverzeichnis hier aus
\clearpage{\thispagestyle{empty}\cleardoublepage}
\ifnoloflot
\else
\listoffigures
\clearpage{\thispagestyle{empty}\cleardoublepage}
\listoftables
\clearpage{\thispagestyle{empty}\cleardoublepage}
\fi
\fi

\ifnomenclature

\markboth{\nomname}{\nomname}%
\printnomenclature[1.8cm]
\clearpage{\thispagestyle{empty}\cleardoublepage}
\fi

\ifarticle
\pagenumbering{arabic}          % Seiten im Text werden arabisch numeriert (1,2,...)
\iftwoside
\setcounter{page}{3}
\else
\setcounter{page}{2}
\fi
\else
\pagenumbering{arabic}          % Seiten im Text werden arabisch numeriert (1,2,...)
\setcounter{page}{1}            % Text neu mit Seite 1 beginnen
\fi
\clearpage{\thispagestyle{empty}\cleardoublepage}               % Seitenumbruch

}
%%%%%%%%%%%%%%%%%%%%%%%%%%%% THESIS ENDING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% ENVIRONMENTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

\AtBeginDocument{
\hypersetup{
pdfauthor=\@author\relax,
pdftitle=\@title\relax
}
\ifnotitle
\correctlanguage
\else
\tuvisionheading
\hypersetup{pageanchor=true}
\clearpage{\thispagestyle{empty}\cleardoublepage}
\fi
}
\endinput
%%
%% End of file `tuvisionthesis.cls'.
