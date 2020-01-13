[![GitHub version](https://img.shields.io/badge/version-1.3.0-green.svg)](https://github.com/hnrck/rrosace/releases/tag/1.3.0)
[![Build Status](https://travis-ci.org/hnrck/rrosace.svg?branch=master)](https://travis-ci.org/hnrck/rrosace)
[![GitHub issues](https://img.shields.io/github/issues/hnrck/rrosace.svg)](https://github.com/hnrck/rrosace/issues)
[![GitHub forks](https://img.shields.io/github/forks/hnrck/rrosace.svg)](https://github.com/hnrck/rrosace/network)
[![GitHub stars](https://img.shields.io/github/stars/hnrck/rrosace.svg)](https://github.com/hnrck/rrosace/stargazers)
[![GitHub license](https://img.shields.io/github/license/hnrck/rrosace)](https://github.com/hnrck/rrosace/blob/master/LICENSE)

# RROSACE 

Scheduling of a Cyber-physcal system simulation -- Cyber-physical models library.

## Description

![rrosace_pic](/res/rrosace.png)

This project is a simple longitudinal flight loop :airplane: based on the ROSACE case study.
Links :link: to the case study, and some implementations, are available below.

RROSACE adds redundancy to ROSACE controllers, in order to highlight scheduling problems.

This project was used as part of a CIFRE PhD thesis :mortar_board: involving ISAE-SUPAERO and Airbus to illustrate the simulation scheduling of cyber-physical systems.

## Table of Contents
* [1. Usage](#1-usage)
  * [1.1. Cloning this repo](#11-cloning-this-repo)
  * [1.2. Testing the implementation](#12-testing-the-implementation)
  * [1.3. Generating the documentation](#13-generating-the-documentation)
  * [1.4. Building and using the project](#14-building-and-using-the-project)
* [2. Related work](#2-related-work)
* [3. Project structure](#3-project-structure)
* [4. Contributing](#4-contributing)
* [5. Credits](#5-credits)
* [6. License](#6-license)

## 1. Usage

### 1.1. Cloning this repo

This repo is available at :link: [hnrck/rrosace](https://github.com/hnrck/rrosace).

Cloning can be done in HTTPS mode:
```bash
$ git clone https://github.com/hnrck/rrosace
$ git checkout tags/1.3.0
```

or with ssh mode:
```bash
$ git clone git@github.com:hnrck/rrosace.git
$ git checkout tags/1.3.0
```

### 1.2. Testing the implementation

The compilation of the library can be test using the following command:
```bash
$ make test
```
Please feel free to report problems with your tests.

### 1.3. Generating the documentation
```bash
$ make doc
```
The documentation can be opened in a browser, or generated in PDF from LaTeX sources.

For browser based navigation, open ````build/doc/html/index.html````
For generating PDF, do:

```bash
$ make -C build/doc/latex
```

Generated PDF will be ```build/doc/latex/refman.pdf```


### 1.4. Building and using the project

In order to build the project, use the following command:

```bash
$ make 
```

Project can be install with:

```bash
$ make install
```

By default, the installation is local.
Installed headers and libraries can be used sourcing the local ```rrosacepathsrc```: ```install/etc/rrosace/rrosacepathsrc```
This can be done using:
```bash
$ source install/etc/rrosace/rrosacepathsrc
```

Absolute path can be deduced and sourced directly before use, for instance in dev environment in any .{bash,zsh,...}rc.
```bash
$ source <path/to/rrosace>/install/etc/rrosace/rrosacepathsrc
```

Furthermore, while installing, the absolute path of the ```rrosacepathsrc``` is given:
```bash
...
-- Installing: <path/to/rrosace>/install/etc/rrosace/rrosacepathsrc
```

After installation and sourcing, ```rrosace``` library can be used as any library.
Compilation using ````rrosace```` library while need linking, using ```-lrrosace```.

The installation can be checked using the simple loop example.
``` bash
$ make check_install
```

The following integration of component is recommended, and used in the loop example:
![rrosace_components](/res/rrosace_components.png)

### 1.5. Other targets

Cleaning the repo:

```bash
make clean
```

Static check on source code (requires clang-tidy):

```bash
make lint
```

Formatting of source code (requires clang-format):

```bash
make format
```

Building / running a simple loop example:

```bash
make example_loop
```

```bash
make run_example_loop
```

### 1.6. Alternative builds and configurations

It is possible to directly use ```cmake``` to build the project, feel free to based your command on the targets in the ```Makefile```

Furthermore, compilation options are possible from the environment, set the following variable in you shell or before your command:
```bash
$ export <VAR>=<my option>
$ make
```
or
```bash
$ <VAR>=<my option> make
```

| option | VAR | default value | alternatives |
|--------|-----|---------------|--------------|
| C compiler | CC | clang | gcc, icc, ... |
| C++ compiler | CXX | clang++ | g++, ... |
| C++ standard | CXXSTD | 98 | 11, 14 |
| Generator | GENERATOR | Ninja | Unix Makefile, ...
| Build type | BUILD_TYPE | RelWithDebInfo | Release, Debug, MinSizeRel |
| Build directory | BUILD_DIR | build |  anywhere |
| Install directory | INSTALL_DIR | install |  anywhere |
| Activate/Deactivate C++ check | CPPCHECK | 1 (activate) |  0 (deactivate) |

## 2. Related work

RROSACE is based on the Open Source ROSACE (Research Open-Source Avionics and Control Engineering) case study.

* Publication of ROSACE :books: available at [ROSACE paper](https://oatao.univ-toulouse.fr/11522/1/Siron_11522.pdf)
* Implementations of ROSACE available at [ROSACE case study](https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/)
* Implementations of ROSACE also available at [OpenAADL/AADLib](https://github.com/OpenAADL/AADLib/) in [example](https://github.com/OpenAADL/AADLib/tree/master/examples/rosace) and [fmu2aadl](https://github.com/OpenAADL/fmu2aadl/tree/master/examples/rosace)
* Publication of RROSACE  :books: available at [RROSACE paper](https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/redundant/report_redundant_rosace_matlab.pdf)
* RROSACE resources available at [RROSACE case study](https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/redundant/)

* Publications using RROSACE :books::
  * [Toward a formalism to study the scheduling of cyber-physical systems simulations](https://scholar.google.fr/scholar?oi=bibs&hl=fr&cluster=8309967183591919001)
  * [Coincidence Problem in CPS Simulations: the R-ROSACE Case Study](https://scholar.google.fr/scholar?oi=bibs&hl=fr&cluster=10404789473361905467)
  * [Implementation of a Cyber-Physical Systems simulation components allocation tool](https://scholar.google.fr/scholar?oi=bibs&hl=fr&cluster=809749533240704673)

## 3. Project structure
```bash
.
├── ChangeLog.md
├── cmake-extra-modules
│   └── *.cmake
├── CMakeLists.txt
├── doc
│   └── Doxyfile.in
├── examples
│   └── *
│       └── {Makefile,*.{h,c,cpp}}
├── include
│   └── rrosace{-*}.h
├── LICENSE
├── Makefile
├── README.md
├── res
│   └── *
├── src
│   └── *.c
├── test
│   └── *.{h,c,cpp}
└── VERSION
```

Important folders:
- ```include``` - the rrosace headers.
- ```src``` - the sources of the rrosace library.
- ```test``` - the rrosace tests folders.
- ```examples``` - the rrosace examples folders, each example has a dedicated folder.

Important files:
- ```README.md``` - this file.
- ```LICENSE``` - the license file.
- ```VERSION``` - the current version of the library.
- ```ChangeLog.md``` - list of changes between revisions.
- ```CMakeLists.txt``` - the project configuration file
- ```Makefile``` - helper for compilation and installation.


## 4. Contributing
Contributions are welcome :unlock:. Feel free to fork too :fork_and_knife:.

## 5. Credits
- Henrick Deschamps  ([:octocat: hnrck](https://github.com/hnrck) ) ([:globe_with_meridians: hnrck.io](https://hnrck.io))

## 6. License
This work is under the [MIT License](https://github.com/hnrck/pretended-blockchain/blob/master/LICENSE). :copyright: Henrick Deschamps, 2019.
