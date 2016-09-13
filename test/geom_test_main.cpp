// Copyright Ingo Proff 2016.
// https://github.com/CrikeeIP/Geometry
// Distributed under the MIT Software License (X11 license)
// (See accompanying file LICENSE)


#include "geometry/geometry.h"

#include <iostream>


typedef geom2d::Vec2D<double> Vec2;
typedef geom2d::Vec2D<int> Vec2i;
typedef geom::Vec<double, 3> Vec3;


void test_basic_funcionality(){
	//Vector initialization
	const auto const_vec = geom::Vec<int,4>( 5 );

	//Compute Pythagoras & Operators
	Vec3 vec3d_1{ { 0,1,2 } }; //Irgendwie moeglich hier die 3 zu deduzieren? (Laenge des uebergebenen Vektors)
	Vec3 vec3d_2{ {-1,-1,-1} };
	assert( geom::in_range( geom::internal::compute_pythagoras<double, 3, 2>::apply( vec3d_1, vec3d_2 ), 1.0 + 4.0 + 9.0 ) );
	assert( geom::in_range( vec3d_1 * vec3d_2, -3.0 ) );

	vec3d_1 = { { 4,-1,-7 } };
	vec3d_2 = { { 3,-3,5 } };
	assert( geom::in_range( geom::internal::compute_pythagoras<double, 3, 2>::apply( vec3d_1, vec3d_2 ), 1.0 + 4.0 + 144.0 ) );
	assert( geom::in_range( vec3d_1 * vec3d_2, 12.0+3.0-35.0 ) );

	geom::Vec<float, 5> vecf5_1{ {1.5f, 3.7f, -2.6f, 0.12f, -8.0f} };
	geom::Vec<float, 5> vecf5_2{ {1.0f, 4.0f, 0.0f, -0.2f, -10.0f} };
	assert( geom::in_range( geom::internal::compute_pythagoras<float, 5, 4>::apply( vecf5_1, vecf5_2 ), static_cast<float>( 0.25 + 0.3*0.3 + 2.6*2.6 + 0.32*0.32 + 4 ) ) );

	//Norm
	geom::Vec<double, 3> vec3d_3{ {5, -2, 7} };
	double n = geom::norm( vec3d_3 );
	assert( geom::in_range( n, std::sqrt( 25+4+49 ) ) );

	geom::Vec<double, 6> vec6d_1{ { 1, -2, 3, -4, 5, -6 } };
	n = geom::norm( vec6d_1 );
	assert( geom::in_range( n, std::sqrt( 1 + 4 + 9 + 16 + 25 + 36) ) );
	
	Vec2i v2( 1, 1);
	double norm = geom::norm( v2 );
	assert( geom::in_range( norm, std::sqrt( 2 ) ) );
	
	geom::Vec<bool, 2> cv1( true, false );
	norm = geom::norm( geom::normalize<double>( cv1.convert_to_doubles() ) );
	assert( geom::in_range( norm, 1.0 ) );
	
	//Dist
	vec3d_1 = { { 4,-1,-7 } };
	vec3d_2 = { { 3,-3,5 } };
	assert( geom::in_range( geom::square_dist( vec3d_1, vec3d_2 ), 1.0 * 1.0 + 2.0 * 2.0 + 12.0 * 12.0 ) );

	std::cout << "- basic functions test successful" << std::endl;
}

void test_angles() {
	geom2d::Vec2D<double> vec1{ 1,0 };
	geom2d::Vec2D<double> vec2{ 10,0 };

	assert( geom2d::angle( vec1, vec2 ) == 0.0 );
	assert( geom2d::enclosed_angle( vec1, vec2 ) == 0.0 );
	assert( geom2d::positive_angle( vec1, vec2 ) == 0.0 );

	vec2 = { 1,1 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ), 45.0 ) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ), 45.0 ) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ), 45.0 ) );

	vec2 = { -1,-1 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ), -135.0) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ), 135.0) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ), 225.0) );

	vec2 = { -2,0 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ), 180.0) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ), 180.0) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ), 180.0) );

	vec2 = { 0,-1 };
	assert( geom::in_range (geom2d::angle( vec1, vec2 ), -90.0 ) );
	assert( geom::in_range (geom2d::enclosed_angle( vec1, vec2 ), 90.0) );
	assert( geom::in_range (geom2d::positive_angle( vec1, vec2 ), 270.0) );

	vec2 = { -2,-1 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ), -153.43494882292202 ) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ), 153.43494882292202 ) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ), 360 - 153.43494882292202 ) );

	std::cout << "- angle tests successful" << std::endl;
}

void test_cvhull() {
	std::vector<Vec2> p{ { 1,-1 },{ -1,-1 },{ -1,1 },{ 1,1 }, };
	auto hull = geom2d::convex_hull( p );

	assert( hull.size() == 4 );

	p.push_back( { 0,0 } );
	auto hull2 = geom2d::convex_hull( p );
	assert( hull2 == hull );

	p.push_back( { 0,-1.000001 } );
	hull = geom2d::convex_hull( p );
	assert( hull.size() == 5 );

	p.push_back( { 0,2 } );
	assert( geom2d::convex_hull( p ).size() == 6 );

	p.push_back( { 2,0 } );
	assert( geom2d::convex_hull( p ).size() == 7 );

	p.push_back( { 2,0 } );
	assert( geom2d::convex_hull( p ).size() == 7 );

	p = { { 1.45622, 131.4252 },{ 141.13412424, 3526.2342134 } };
	assert( geom2d::convex_hull( p ).size() == 2 && hull[0] != hull[1] );

	p = { { 1324.124, 123.47 } };
	assert( geom2d::convex_hull( p ).size() == 1 );

	p = {};
	assert( geom2d::convex_hull( p ).size() == 0 );

	p = { { 0.0001,0.00010001 },{ 0.2,3.223 },{ 0,0 },{ 3.991, 5.001 },{ 9.9999999, 9.999999999999 },{ -0.1, 18.41324135 },{ 10,10 },{ 9,12 },{ 0,20 },{ -10,10 },{ -2,1.999999999 },{ -3, 4.555 } };
	assert( geom2d::convex_hull( p ).size() == 6 );

	std::cout << "- convex hull tests successful" << std::endl;
}

void test_point_in_polygon() {
	std::vector<Vec2> p{ { 1,-1 },{ -1,-1 },{ -1,1 },{ 1,1 }, };
	assert( geom2d::point_in_polygon( { 0,0 }, p ) == true );

	p = { { 0.0001,-0.0001 },{ -0.0001,-0.0001 },{ -0.0001,0.0001 },{ 0.0001,0.0001 }, };
	assert( geom2d::point_in_polygon( { 0.000001,-0.0000003 }, p ) == true );

	p = { { 99,0 },{ 7,0 } };
	assert( geom2d::point_in_polygon( { 99.0000000000001, 0 }, p ) == false );

	p = { { 3,-1 },{ -1, -1 },{ -1,1 },{ 2,1 },{ 2,0 },{ 3,1 } };
	assert( geom2d::point_in_polygon( { 0,0 }, p ) == true );

	std::cout << "- point in polygon tests successful" << std::endl;
}


int main(){
    std::cout << "Starting *geom::* tests:" << std::endl;

    //vector initialization; vector operators, pythagoras, norm & dist
    test_basic_funcionality();

    //angles between zero-based vectors
    test_angles();

    //convex hull
    test_cvhull();

    //point in polygon
    test_point_in_polygon();


    std::cout << "Congrats! All tests succesful." << std::endl;
}
