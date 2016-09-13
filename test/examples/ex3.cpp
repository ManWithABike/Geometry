#include "geometry/geometry.h"

int main(){
  geom2d::polygon<double> p ( {{0, -1}, {-1, 0}, {0,1}, {1,0}} );

  std::cout << "Point in polygon test: " << geom2d::point_in_polygon({-0.4,0.2}, p) << std::endl; //Check if a point is in polygon

  geom2d::point_cloud<int> c ({ {1,1}, {0,1}, {10,10}, {-1,5}, {-3, -5}, {3, -1}, {5,6} }); //Compute the convex hull of a set of 2d-points
  geom2d::polygon<int> conv_hull = geom2d::convex_hull(c);
  for( const auto& x : conv_hull ){
	std::cout << x.print() << " ; ";
  }
}