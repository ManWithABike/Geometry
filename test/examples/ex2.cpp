#include "geometry/geometry.h"

typedef geom::Vec<2,int> Vec; //All our vectors have two integer coordinates

int main(){
  Vec v1( 0, 0 );
  Vec v2( 2, 0 );
  Vec v3( 1, 1 );
  polygon p (v1,v2,v3);
  double polygon_area = geom2d::area(p); //polygon_area == 0.5
}