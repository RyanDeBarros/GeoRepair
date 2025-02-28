## GeoRepair

My term project for COMP 438: Geometric Modelling and Processing.
GeoRepair is an application using libigl under the hood to detect and repair common mesh defects like:
* Duplicate faces
* Duplicate vertices
* Null faces
* Degenerate faces
* Unpatched holes
* Inverted normals
* Non-manifold edges
* Invalid vertex/face values

Each feature has customizable parameters, and there is also an undo/redo system to track changes.
