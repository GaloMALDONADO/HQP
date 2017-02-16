# HQP
Library for hierarchichal quadratic programming
##Characteristics:
  - Computes HQP based on nulsspace projections and quadritc programming using qpOASES and scipy.
  - It can be used for generation motion of anthromorphic systems such as robots and avatars.  

##Setup:
```json
   $ python setup.py --prefix='your install path'
```

##Required Dependencies:
- Python 2.7, numpy, scipy, matplotlib: 
```json	 
   $ sudo apt-get install python-numpy python-scipy python-matplotlib
- Cython
```json
   $ sudo pip install cython 
```
- qpOASES: https://github.com/GaloMALDONADO/qpOASES
```json
   $ git clone https://github.com/GaloMALDONADO/qpOASES
   $ mkdir build && cd build 
   $ cmake [OPTIONS] 
   $ make install
   $ cd .. && make python
   $ cd interfaces/python && python setup.py [OPTIONS]
```

- Pinocchio: library for rigid multi-body dynamics. Can be download from here: http://stack-of-tasks.github.io/pinocchio/ 
- Gepetto-viewer: A graphical interface for pinocchio. Can be downloaded from here: https://github.com/GaloMALDONADO/gepetto-viewer
- Gepetto-viewer-corba: CORBA server/client for the Graphical Interface of Pinocchio. Can be downloaded from here: https://github.com/GaloMALDONADO/gepetto-viewer-corba   

##Extra Dependencies:
- Models: Contains biomechanical and robotic models. Can be downloaded from here: https://github.com/GaloMALDONADO/Models 
- gmp: can be downloaded from here: https://gmplib.org/devel/repo-usage.html
```json
   $ ./.bootstrap
   $ ./configure --enable-maintainer-mode
   $ make 
   $ make check
```
- pycddlib: the simplest way to install is with pip: 
```json
   $ pip install pycddlib
```
