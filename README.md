# how2sketch
This is the code for the following paper: 

Hennessey, James W., Han Liu, Holger Winnemöller, Mira Dontcheva and Niloy J. Mitra. “How2Sketch: generating easy-to-follow tutorials for sketching 3D objects.” I3D (2017)

The code in this repo contains the core algorithms for candidate generation, selection and tutorial generation from the paper. It outputs a json file for a seperate application to view the tutorial. The code is currently being released for documentation purposes of the implementation only. We hope to release a single more user-friendly application later.  

## Dependencies 
* QT
* Gurobi
* GGAL
* Eigen
* IGL

## Interface Instructions
* File -> Import.... (json file of segments/prmitives)
* Select "Segment Boxes" radio button
* Find relations (highlighted in red) using the "Find" button
* Generate candidates
* View candidates by selecting "Complete Candidate" button
* Run Optimisation
* Save (JSON File)
