#include "geometry/geometry.h"

int main(){
  geom2d::polygon<double> p ( {0, -1}, {-1, 0}, {0,1}, {1,0} );
  std::cout << "Point in polygon test: " << geom2d::point_in_polygon({-0.4,0.2}, p) << std::endl; //Check point in polygon
}