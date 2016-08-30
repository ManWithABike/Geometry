# Geometry
A small convenience header-only library for linear algebra.

##Dependencies, Installation, Usage
**Dependencies:** None  
**Installation:** Download the Header, include it, and you're ready to go.  
**Usage:** In namspace *geom::*, the library offers a struct for storing n-dimensional vertices in *R^n* or *Z^n* (i.e. as doubles or integers) with several constructors and overloads their operators.  
Furthermore, it offers a namespace *geom2d::* with some convenience functions for two-dimensional algebra like *cross product*, *perpendicular* and *normal*, *angle calculation*, *segment intersection* and higher level stuff like *polytop area* calcuation.

##Disclaimer
The functionality of this library initially grew due to my personal need for it. Several projects which I did not want to depend on something as overkill as Boost rely on this simple collection of linear algebra functions. The API still might change in the future. If you have any suggestions, find errors, miss some functions or want to give general feedback/criticism, I'd love to hear from you. Of course, [contributions](https://github.com/CrikeeIP/Geometry/pulls) are also very welcome.

##License
Distributed under the Boost Software License, Version 1.0. (See accompanying file LICENSE or copy at http://www.boost.org/LICENSE_1_0.txt)
