sot-hrp2
===========

[![Pipeline status](https://gepgitlab.laas.fr/stack-of-tasks/sot-hrp2/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/sot-hrp2/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/sot-hrp2/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/sot-hrp2/master/coverage/)


This packages provides a generic Stack Of Tasks library
for the humanoid robot HRP2. This library is highly
portable and can be used in various simulators, and
the robot itself.

Install
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.

To use:
=======
To embed this library an example is provided in:
sot/core/tools/test_abstract_interface.cpp
