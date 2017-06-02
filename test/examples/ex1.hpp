#include "geometry/geometry.hpp"
#include <cassert>

typedef geom::Vec<double,3> Vec3d; //All our vectors have three double coordinates

int ex1() {
	Vec3d v1( 1.0, -1.0, -2.2 );
	Vec3d v2( 3.0, 1.0, -0.2 );

	Vec3d sub_v1v2 = v1 - v2;
	std::cout << "sub_v1v2: " << sub_v1v2.print() << std::endl; //sub_v1v2 == (-2.0, -2.0, -2.0 )
	
	Vec3d normalised = geom::normalize( sub_v1v2 );
	std::cout << "sub_v1v2 normalised: " << normalised.print() << std::endl; //normalised ==(-sqrt(1/3), -sqrt(1/3), -sqrt(1/3)
	
	assert( geom::in_range( geom::norm( normalised ), 1.0 ) ); //Double calculation isn't exact! Therefor, check result up to double precision
	
	std::cout << "Example1 successful executed!" << std::endl;

	return 0;
}