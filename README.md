# Introduction

This is the code used for reconstruction of the scenes in the paper "SceneNN: A Scene Meshes Dataset with aNNotations" at 3DV 2016. Project page: http://www.scenenn.net. 

# Compilation

This code can be built with Visual Studio 2013. The following dependencies are required:

* Use g2o devel branch, commit 41b0be79a0821c1466f694dc16a9ee2c5ee92a9b, on Dec 22, 2015. 
* For PCL, we used PCL 1.7.2 all-in-one package. The unofficial precompiled library is available at http://unanancyowen.com/?p=1255&lang=en
  Note that this PCL compilation does not include openni_grabber, and therefore ONI file is not supported. 
* CUDA 7.0

# License 

The source code is released under MIT license.

For easy compilation of the system, the following third party libraries are included. Please be aware that they can be released under different licenses: 

* g2o <GraphOptimizer/external/g2o> - BSD license
* vertigo <GraphOptimizer/vertigo> - GPLv3 license
* SuiteSparse <FragmentOptimizer/external/SuiteSparse> - LGPL3+ license
* Eigen <FragmentOptimizer/external/Eigen> - MPL2 license

# Acknowledgement

This code is forked and slightly modified from the public code released by Qianyi Zhou et al. at https://github.com/qianyizh/ElasticReconstruction. For in-depth description of the components of the system, please visit their repository. Please also visit their project homepage at
http://redwood-data.org/indoor/ for their papers. 
