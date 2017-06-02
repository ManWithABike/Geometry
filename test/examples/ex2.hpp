#include "geometry/geometry.hpp"

typedef geom::Vec<int,2> Vec2i; //All our vectors have two integer coordinates

int ex2(){
	Vec2i v1( 0, 0 );
	Vec2i v2( 2, 0 );
	Vec2i v3( 1, 1 );
	geom2d::polygon<int> p ( {v1,v2,v3} );

	double polygon_area = geom2d::area(p);
	std::cout << "Polygon area: " << polygon_area << std::endl; //polygon_area == 1
	assert( geom::in_range(polygon_area, 1.0) );

	return 0;
}