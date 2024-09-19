## Título de Paper

This repository contains the source code of the paper [Robust Fitting of Ellipsoids by Separating Interior and Exterior Points During Optimization](https://doi.org/10.1007/s10851-016-0700-6).

This code executes the proposed method for a set of input images. The contents of this code are provided without any warranty. They are intended for evaluational purposes only.

![Alt text](Example.PNG?raw=true "Example of ellipsoid fitting to scanned data")

### Pre-requisites

- Matlab (tested on v2018b or earlier). 
- The FitEllipsoidEnhanced.m script provides the same functionality as FitEllipsoidEnhancedOpt.m, but the former does not need MEX file compilation while the latter is faster.
---

### Set up

You might want to start by running any of the Demo*.m scripts.


### Run the Demo

Run Demo*.m in Matlab
- DemoFitEllipsoidSynthetic.m: Fit a randomly generated ellipsoid (ground truth available)
- DemoFitEllipsoidScanned.m: Fit an ellipsoid to a set of 3D scanned data
- DemoFitEllipsoidStereo.m: Fit an ellipsoid to a real image

---

### Citation

Please, cite this work as:

E. Lopez-Rubio, K. Thurnhofer-Hemsi, O. D. de Cozar-Macias, E. B. Blazquez-Parra, J. Muñoz-Perez, and I. Ladron de Guevara-Lopez. Robust Fitting of Ellipsoids by Separating Interior and Exterior Points During Optimization. Journal of Mathematical Imaging and Vision. ISSN: 0924-9907.
https://doi.org/10.1007/s10851-016-0700-6.
(https://link.springer.com/article/10.1007/s10851-016-0700-6)