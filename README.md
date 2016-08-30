# Geometry

A small convenience header-only library for linear algebra.

##Dependencies, Installation, Usage
Dependencies: None  
Installation: Download the Header, include it, and you're ready to go.  
Usage: In namspace *geom::*, the library offers a struct for storing n-dimensional vertices in *R^n* or *Z^n* (i.e. as doubles or integers) with several constructors and overloads their operators. Furthermore, it offers some convenience functions for 2 dimensional vertices like *segment intersection*, *angle calculation*, *cross product*, *perpendicular* and *normal*,  *normaliziation*   

##Restrictions
1. Both polygons **P**, **Q** must be non self-intersecting and
2. have to share the same start and end point.
3. All intersections between **P** and **Q** must me trensversal


##Dependencies
Two lightweight header-only libraries:  
1. [Geometry](https://github.com/CrikeeIP/Geometry)  
2. [FunctionalPlus](https://github.com/Dobiasd/FunctionalPlus)  

And boost (::geometry and ::index, to be exact)  
4. [Boost](http://www.boost.org/)


##Disclaimer
The functionality in this library initially grew due to my personal need for it while using C++ on a regular basis. I try my best to make it error free and as comfortable to use as I can. The API still might change in the future. If you have any suggestions, find errors, miss some functions or want to give general feedback/criticism, I'd love to hear from you. Of course, [contributions](https://github.com/CrikeeIP/OPTICS-Clustering/pulls) are also very welcome.

##License
Distributed under the Boost Software License, Version 1.0. (See accompanying file LICENSE or copy at http://www.boost.org/LICENSE_1_0.txt)
