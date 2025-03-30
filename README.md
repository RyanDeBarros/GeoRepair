## GeoRepair

My term project for COMP 438: Geometric Modelling and Processing.
GeoRepair is an application using libigl under the hood to detect and repair common mesh defects like:
* Duplicate faces
* Duplicate vertices
* Isolated vertices
* Null faces
* Degenerate faces
* Unpatched holes
* Inverted normals
* Non-manifold edges
* Invalid vertex/face values
* Unconnected submeshes

Each feature has customizable parameters, and there is also an undo/redo system to track changes.

### Research papers

This project implements several algorithms from different papers/resources. In particular:

* [Mesh Smoothing](https://graphics.stanford.edu/courses/cs468-12-spring/LectureSlides/06_smoothing.pdf) from Stanford lecture slides.
* [Bilateral Mesh Denoising](https://www.sci.utah.edu/~shachar/Publications/bmd03.pdf) by Shachar Fleishman, Iddo Drori, and Daniel Cohen-Or from School of Computer cience, Tel Aviv University.
* [Triangulation by Ear Clipping](https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf) by David Eberly. Licensed under [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/)
