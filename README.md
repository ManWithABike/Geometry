# Geometry
A small convenience header-only library for linear algebra.

##Dependencies, Installation, Usage
**Dependencies:** None  
**Installation:** Download the Header, include it, and you're ready to go.  
**Usage:** In namespace *geom::*, the library offers a struct for storing n-dimensional vectors in *R^n* or *Z^n* (i.e. as doubles or integers) with several constructors, and overloads their operators.  
Furthermore, it offers a namespace *geom2d::* with some convenient functions for two-dimensional algebra like *cross product*, *perpendicular* and *normal*, *angle calculation*, and higher level stuff like *segment intersection* or *polygon area* calcuation.


##Examples

**Ex1**  
```cpp
#include "geometry.h"
#include <cassert>

typedef geom::Vec<3,double> Vec; //All our vectors have three double coordinates

int main(){
  Vec v1( 1.0, -1.0, -2.2 );
  Vec v2( 3.0, 1.0, -0.2);
  
  Vec sub_v1v2 = v1 - v2; //sub_v1v2 == (-2.0, -2.0, -2.0 )
  Vec normalised = geom::normalise(sub_v1v2); //normalised ==(-sqrt(1/3), -sqrt(1/3), -sqrt(1/3)
  assert( geom::in_range( geom::norm(normalised), 1.0 ) ); //Double calculation isn't exact! Therefor, check result up to double precision
}
```
**Ex2**  

**Ex3**  


##Disclaimer
The functionality of this library initially grew due to my personal need for it. Several projects, which I did not want to depend on a library as overkill as Boost, rely on this simple collection of linear algebra functions. The API might still change in the future. If you have any suggestions, find errors, miss some functions or want to give general feedback/criticism, I'd love to hear from you. Of course, [contributions](https://github.com/CrikeeIP/Geometry/pulls) are also very welcome.

##License
Distributed under the Boost Software License, Version 1.0. (See accompanying file LICENSE or copy at http://www.boost.org/LICENSE_1_0.txt)
