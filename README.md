# HQP
Library for hierarchichal quadratic programming
##Characteristics:
  - Computes HQP based on nulsspace projections and quadritc programming using qpOASES and scipy.
  - It can be used for generation motion of anthromorphic systems such as robots and avatars.  
##Setup
```json
   $ python setup.py --prefix='your install path'
```

##Required Dependencies:
- Python 2.7, numpy, scipy, matplotlib: $sudo apt-get install python-numpy python-scipy python-matplotlib
- qpOASES: https://github.com/GaloMALDONADO/qpOASES
- Pinocchio: librari for rigid multi-body dynamics. Can be download from here: http://stack-of-tasks.github.io/pinocchio/ 
- Gepetto-viewer: A graphical interface for pinocchio. Can be downloaded from here:
    https://github.com/humanoid-path-planner/gepetto-viewer.git
- Gepetto-viewer-corba: CORBA server/client for the Graphical Interface of Pinocchio. Can be downloaded from here:
    https://github.com/humanoid-path-planner/gepetto-viewer-corba.git

##Extra Dependencies:
- Models: Contains biomechanical and robotic models. Can be downloaded from here: https://github.com/GaloMALDONADO/Models 
- gmp: can be downloaded from here: https://gmplib.org/devel/repo-usage.html
- pycddlib: the simplest way to install is with pip: pip install pycddlib

