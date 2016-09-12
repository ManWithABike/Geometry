#include "geometry/geometry.h"
#include <cassert>

typedef geom::Vec<3, double> Vec; //All our vectors have three double coordinates

int main() {
	Vec v1( 1.0, -1.0, -2.2 );
	Vec v2( 3.0, 1.0, -0.2 );

	Vec sub_v1v2 = v1 - v2; //sub_v1v2 == (-2.0, -2.0, -2.0 )
	Vec normalised = geom::normalise( sub_v1v2 ); //normalised ==(-sqrt(1/3), -sqrt(1/3), -sqrt(1/3)
	assert( geom::in_range( geom::norm( normalised ), 1.0 ) ); //Double calculation isn't exact! Therefor, check result up to double precision
}