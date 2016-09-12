#include "geometry/geometry.h"
#include <cassert>

typedef geom::Vec<double,3> Vec; //All our vectors have three double coordinates

int main() {
	Vec v1( 1.0, -1.0, -2.2 );
	Vec v2( 3.0, 1.0, -0.2 );

	Vec sub_v1v2 = v1 - v2;
	std::cout << "sub_v1v2: " << sub_v1v2.print() << std::endl; //sub_v1v2 == (-2.0, -2.0, -2.0 )
	
	Vec normalised = geom::normalize( sub_v1v2 );
	std::cout << "sub_v1v2 normalised: " << sub_v1v2.print() << std::endl; //normalised ==(-sqrt(1/3), -sqrt(1/3), -sqrt(1/3)
	
	assert( geom::in_range( geom::norm( normalised ), 1.0 ) ); //Double calculation isn't exact! Therefor, check result up to double precision
	
	std::cout << "Example1 successful executed!" << std::endl;
}