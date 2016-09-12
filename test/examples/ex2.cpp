#include "geometry/geometry.h"

typedef geom::Vec<int,2> Vec; //All our vectors have two integer coordinates

int main(){
  Vec v1( 0, 0 );
  Vec v2( 2, 0 );
  Vec v3( 1, 1 );
  geom2d::polygon<int> p ( {v1,v2,v3} );

  double polygon_area = geom2d::area(p);
  std::cout << "Polygon area: " << polygon_area << std::endl; //polygon_area == 1
}