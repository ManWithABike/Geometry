// Copyright Ingo Proff 2016.
// https://github.com/CrikeeIP/Geometry
// Distributed under the MIT Software License (X11 license)
// (See accompanying file LICENSE)


#include "geometry/geometry.h"

#include <iostream>


typedef geom2d::Vec2D<double> Vec2;
typedef geom2d::Vec2D<int> Vec2i;
typedef geom::Vec<double, 3> Vec3;
typedef geom::Vec<int, 3> Vec3i;


void test_basic_funcionality(){
	//Vector initialization
	const auto const_vec = geom::Vec<int,4>( 5 );
	assert( const_vec[3] == 5 );
	const_vec.dimension();
	assert( const_vec.dimension() == 4 );
	
	const auto xyz = geom::Vec<unsigned char, 3>( { 1,2,3 } );
	assert( xyz.x() == 1);
	assert( xyz.y() == 2 );
	assert( xyz.z() == 3 );
	assert( xyz.dimension() == 3 );

	const auto xy = geom::Vec<unsigned char, 2>( 1,2 );
	assert( xy.dimension() == 2 );
	assert( xy.x() == 1 );
	assert( xy.y() == 2 );

	//default constructed
	geom::Vec<int, 1> mv_1i;
	assert( mv_1i.dimension() == 1 );
	assert( (mv_1i == geom::Vec<int, 1>( 0 )) );
	geom::Vec<double, 6> mv_6d;
	assert( mv_6d.dimension() == 6 );
	assert( (mv_6d == geom::Vec<double, 6>( 0.0 )) );
	geom::Vec<char, 6> mv_6char;
	assert( (mv_6char == geom::Vec<char, 6>( 0 )) );
	geom::Vec<std::string, 6> mv_6str;
	assert( (mv_6str == geom::Vec<std::string, 6>( "" )) );

	//make_vec
	auto mv_3d = geom::make_vec( 1.2, 2.5, 3.9 );
	auto mv_4i = geom::make_vec( 1, 2, 3, 4 );
	auto mv_1f = geom::make_vec( -1.6f );

	//round
	auto mv_1 = mv_1f.round();
	assert( (mv_1 == geom::Vec<int,1>(-2)) );
	auto mv_3 = mv_3d.round();
	assert( (mv_3 == geom::Vec<int, 3>(1, 3, 4)) );

	//Set coordinates
	mv_3d[2] = 2.14;
	assert( mv_3d[2] == 2.14 );
	assert(mv_3d.Get<2>() == 2.14);
	const auto mv_3d_const = mv_3d;
	assert( mv_3d_const.Get<2>() == 2.14 );

	//Compute Pythagoras & Operators
	Vec3 vec3d_1{ { 0,1,2 } };
	vec3d_1 = vec3d_1.as_doubles();
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
	
	Vec2i v2( 1, 1 );
	double norm = geom::norm( v2 );
	assert( geom::in_range( norm, std::sqrt( 2 ) ) );
	
	geom::Vec<int, 2> cv1( true, false );
	norm = geom::norm( geom::normalize( cv1.as_doubles() ) );
	assert( geom::in_range( norm, 1.0 ) );
	
	//Crossproduct
	assert( geom2d::cross<double>( { 0,1 }, { 1,0 } ) == geom::Sign::NEGATIVE );
	assert( geom2d::cross<double>( { 1,0 }, { 0,1 } ) == geom::Sign::POSITIVE );
	assert( geom2d::cross<double>( { 1e-16,0 }, { 0,1e-16 } ) > 0.0 );
	assert( geom2d::cross<double>( { 1,0 }, { 2,0 } ) == geom::Sign::ZERO );

	//Dist
	vec3d_1 = { { 4,-1,-7 } };
	vec3d_2 = { { 3,-3,5 } };
	assert( geom::in_range( geom::square_dist( vec3d_1, vec3d_2 ), 1.0 * 1.0 + 2.0 * 2.0 + 12.0 * 12.0 ) );

	//Centroid
	std::vector<Vec2i> points( { { 0,0 },{ 1,0 },{ 1,1 },{ 0,1 } } );
	auto c = geom2d::centroid( points );
	if ( c != Vec2( 0.5, 0.5 ) ) {
		assert( false );
	}
	
	std::cout << "- basic functions test successful" << std::endl;
}

void test_angles() {
	geom2d::Vec2D<double> vec1{ 1,0 };
	geom2d::Vec2D<double> vec2{ 10,0 };

	assert( geom2d::angle( vec1, vec2 ).deg() == 0.0 );
	assert( geom2d::enclosed_angle( vec1, vec2 ).deg() == 0.0 );
	assert( geom2d::positive_angle( vec1, vec2 ).deg() == 0.0 );

	vec2 = { 1,1 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ).deg(), 45.0 ) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ).deg(), 45.0 ) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ).deg(), 45.0 ) );

	vec2 = { -1,-1 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ).deg(), -135.0) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ).deg(), 135.0) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ).deg(), 225.0) );

	vec2 = { -2,0 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ).deg(), 180.0) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ).deg(), 180.0) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ).deg(), 180.0) );

	vec2 = { 0,-1 };
	assert( geom::in_range (geom2d::angle( vec1, vec2 ).deg(), -90.0 ) );
	assert( geom::in_range (geom2d::enclosed_angle( vec1, vec2 ).deg(), 90.0) );
	assert( geom::in_range (geom2d::positive_angle( vec1, vec2 ).deg(), 270.0) );

	vec2 = { -2,-1 };
	assert( geom::in_range( geom2d::angle( vec1, vec2 ).deg(), -153.43494882292202 ) );
	assert( geom::in_range( geom2d::enclosed_angle( vec1, vec2 ).deg(), 153.43494882292202 ) );
	assert( geom::in_range( geom2d::positive_angle( vec1, vec2 ).deg(), 360 - 153.43494882292202 ) );

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

	assert( geom2d::point_in_triangle<int>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,1 } ) );

	assert( !geom2d::point_in_triangle<int>( { 0,0 }, { 2,0 }, { 1,2 }, { 0,0 } ) );
	
	assert( !geom2d::point_in_triangle<int>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,0 } ) );

	assert( geom2d::point_in_triangle<int>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,0 }, true ) );

	assert( geom2d::point_in_triangle<int>( { 0,0 }, { 2,0 }, { 1,2 }, { 0,0 }, true ) );

	assert( !geom2d::point_in_triangle<double>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,-0.0000000000001 }, true ) );
	
	assert( geom2d::point_in_triangle<double>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,0.0000000000001 }, true ) );
	
	assert( geom2d::point_in_triangle<double>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,1.99999999999999 }, true ) );

	assert( !geom2d::point_in_triangle<double>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,-0.0000000000001 }, false ) );

	assert( geom2d::point_in_triangle<double>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,0.0000000000001 }, false ) );

	assert( geom2d::point_in_triangle<double>( { 0,0 }, { 2,0 }, { 1,2 }, { 1,1.99999999999999 }, false ) );
	
	std::cout << "- point in polygon tests successful" << std::endl;
}

void test_polygon_area() {
	{
		geom2d::polygon<int> p = { { 0,0 } };
		auto A = geom2d::area( p );
		assert( geom::in_range( A, 0.0 ) );
	}

	{
		geom2d::polygon<int> p = { { 0,0 },{ 1,0 } };
		auto A = geom2d::area( p );
		assert( geom::in_range( A, 0.0 ) );
	}
	{
		geom2d::polygon<double> p = { { 56465,-6847684 },{ 68441,123132.23158468 } };
		auto A = geom2d::area( p );
		assert( geom::in_range( A, 0.0 ) );
	}

	{
		geom2d::polygon<double> p = { {0,0}, {1,0}, {2,1},{1,1} };
		auto A = geom2d::area( p );
		assert( geom::in_range(A, 1.0) );
	}

	{
		geom2d::polygon<int> p = { { 0,0 },{ 1,0 },{ 435764,1 },{ 1,1 } };
		auto A = geom2d::area( p );
		assert( geom::in_range( A, 217882.0 ) );
	}

	std::cout << "- polygonarea tests successful" << std::endl;
}

void test_rectangles() {
	auto rect = geom2d::Rectangle( { 1,1 }, { 0,0 }, std::sqrt(2), true );
	assert( geom::in_range( rect.angle.deg(), 45.0 ) );

	rect = geom2d::Rectangle( { 1,0 }, { 0,0 }, std::sqrt( 1 ), true );
	assert( geom::in_range( rect.angle.deg(), 0.0 ) );

	rect = geom2d::Rectangle( { 1,1 }, { 0,0 }, std::sqrt( 2 ), false );
	assert( geom::in_range( rect.angle.deg(), -45.0 ) );

	rect = geom2d::Rectangle( { 1,0 }, { 0,0 }, std::sqrt( 1 ), false );
	assert( geom::in_range( rect.angle.deg(), 0.0 ) );

	rect = geom2d::Rectangle( { 10,10 }, { 0,10 }, 10, true );
	assert( geom::in_range( rect.angle.deg(), 0.0 ) );
	assert( geom::in_range( rect.points()[2].x(), 0.0 ) );
	assert( geom::in_range( rect.points()[2].y(), 0.0 ) );
	assert( geom::in_range( rect.points()[3].x(), 10.0 ) );
	assert( geom::in_range( rect.points()[3].y(), 0.0 ) );

	rect = geom2d::Rectangle( { 1,1 }, { 0,0 }, 1, true );

	rect = geom2d::Rectangle( { 1,2 }, { 0,1 }, std::sqrt(2), true );
	assert( geom::in_range( rect.angle.deg(), 45.0 ) );
	assert( geom::in_range( rect.centroid().x(), 1.0 ) );
	assert( geom::in_range( rect.centroid().y(), 1.0 ) );

	std::cout << "- rectangle tests successful" << std::endl;
}

void test_encl_parallelogram() {
	{
		geom2d::point_cloud<double> points = { { 1,-1 },{ -1,-1 },{ -1,1 },{ 1,1 }, };
		auto mec = geom2d::min_enclosing_parallelogram( points );
		double A = geom2d::area( mec );
		assert( geom::in_range(A, 4.0 ) );
	}

	{
		geom2d::point_cloud<double> points = { {182, 121}, { 181, 122 }, { 180, 122 }, { 176, 126 }, { 174, 126 }, { 173, 127 }, { 172, 127 }, { 169, 130 }, { 169, 131 }, { 168, 132 }, { 168, 138 }, { 169, 139 }, { 169, 140 }, { 170, 141 }, { 170, 143 }, { 171, 144 }, { 171, 145 }, { 175, 149 }, { 175, 150 }, { 176, 151 }, { 176, 152 }, { 181, 157 }, { 181, 158 }, { 184, 161 }, { 184, 162 }, { 186, 164 }, { 186, 165 }, { 188, 167 }, { 188, 168 }, { 189, 169 }, { 189, 170 }, { 192, 173 }, { 192, 174 }, { 193, 175 }, { 193, 176 }, { 198, 181 }, { 198, 182 }, { 204, 188 }, { 204, 189 }, { 209, 194 }, { 210, 194 }, { 213, 197 }, { 213, 198 }, { 221, 206 }, { 221, 207 }, { 229, 215 }, { 230, 215 }, { 234, 219 }, { 234, 220 }, { 238, 224 }, { 238, 225 }, { 240, 227 }, { 240, 228 }, { 241, 229 }, { 241, 230 }, { 242, 231 }, { 242, 232 }, { 245, 235 }, { 245, 236 }, { 249, 240 }, { 249, 241 }, { 257, 249 }, { 257, 250 }, { 261, 254 }, { 261, 255 }, { 264, 258 }, { 265, 258 }, { 266, 259 }, { 269, 259 }, { 270, 260 }, { 284, 260 }, { 285, 261 }, { 330, 261 }, { 331, 260 }, { 348, 260 }, { 349, 259 }, { 444, 259 }, { 445, 258 }, { 447, 258 }, { 448, 257 }, { 449, 257 }, { 452, 254 }, { 452, 253 }, { 453, 252 }, { 453, 245 }, { 452, 244 }, { 452, 243 }, { 451, 242 }, { 451, 241 }, { 450, 240 }, { 450, 239 }, { 449, 238 }, { 449, 237 }, { 446, 234 }, { 446, 233 }, { 444, 231 }, { 444, 230 }, { 443, 229 }, { 443, 228 }, { 439, 224 }, { 438, 224 }, { 436, 222 }, { 436, 221 }, { 434, 219 }, { 434, 218 }, { 433, 217 }, { 433, 216 }, { 427, 210 }, { 427, 209 }, { 423, 205 }, { 423, 204 }, { 421, 202 }, { 421, 201 }, { 416, 196 }, { 416, 195 }, { 415, 194 }, { 415, 193 }, { 413, 191 }, { 413, 190 }, { 409, 186 }, { 409, 185 }, { 407, 183 }, { 407, 182 }, { 401, 176 }, { 401, 175 }, { 400, 174 }, { 400, 173 }, { 399, 172 }, { 399, 171 }, { 392, 164 }, { 392, 163 }, { 391, 162 }, { 391, 161 }, { 390, 160 }, { 390, 159 }, { 383, 152 }, { 383, 151 }, { 381, 149 }, { 381, 148 }, { 380, 147 }, { 380, 146 }, { 378, 144 }, { 378, 143 }, { 373, 138 }, { 373, 137 }, { 368, 132 }, { 368, 131 }, { 365, 128 }, { 364, 128 }, { 363, 127 }, { 333, 127 }, { 332, 126 }, { 329, 126 }, { 328, 125 }, { 325, 125 }, { 324, 124 }, { 306, 124 }, { 305, 123 }, { 291, 123 }, { 290, 124 }, { 215, 124 }, { 214, 123 }, { 202, 123 }, { 201, 122 }, { 200, 122 }, { 199, 121 } };
		geom2d::polygon<int> result = { {468, 261}, { 258, 261 }, { 151, 121 }, { 361, 121 } };
		auto mec = geom2d::min_enclosing_parallelogram( points );
		for ( std::size_t i = 0; i < mec.size(); i++ ) {
			assert( mec[i].round() == result[i] );
		}
	}

	{
		geom2d::point_cloud<double> points = { {352, 79}, { 351, 80 }, { 348, 80 }, { 347, 81 }, { 340, 81 }, { 339, 82 }, { 329, 82 }, { 328, 83 }, { 323, 83 }, { 322, 84 }, { 317, 84 }, { 316, 85 }, { 311, 85 }, { 310, 86 }, { 305, 86 }, { 304, 87 }, { 302, 87 }, { 301, 88 }, { 298, 88 }, { 297, 89 }, { 294, 89 }, { 293, 90 }, { 290, 90 }, { 289, 91 }, { 286, 91 }, { 285, 92 }, { 283, 92 }, { 282, 93 }, { 279, 93 }, { 278, 94 }, { 273, 94 }, { 272, 95 }, { 269, 95 }, { 268, 96 }, { 265, 96 }, { 264, 97 }, { 262, 97 }, { 261, 98 }, { 258, 98 }, { 257, 99 }, { 256, 99 }, { 255, 100 }, { 252, 100 }, { 251, 101 }, { 249, 101 }, { 248, 102 }, { 247, 102 }, { 246, 103 }, { 243, 103 }, { 242, 104 }, { 241, 104 }, { 240, 105 }, { 237, 105 }, { 236, 106 }, { 235, 106 }, { 234, 107 }, { 232, 107 }, { 231, 108 }, { 228, 108 }, { 227, 109 }, { 222, 109 }, { 221, 110 }, { 219, 110 }, { 218, 111 }, { 215, 111 }, { 214, 112 }, { 213, 112 }, { 212, 113 }, { 211, 113 }, { 210, 114 }, { 208, 114 }, { 207, 115 }, { 206, 115 }, { 205, 116 }, { 202, 116 }, { 201, 117 }, { 198, 117 }, { 197, 118 }, { 196, 118 }, { 195, 119 }, { 193, 119 }, { 192, 120 }, { 189, 120 }, { 188, 121 }, { 187, 121 }, { 186, 122 }, { 185, 122 }, { 184, 123 }, { 182, 123 }, { 181, 124 }, { 180, 124 }, { 177, 127 }, { 177, 128 }, { 176, 129 }, { 176, 135 }, { 177, 136 }, { 177, 137 }, { 180, 140 }, { 181, 140 }, { 182, 141 }, { 182, 146 }, { 183, 147 }, { 183, 148 }, { 186, 151 }, { 187, 151 }, { 190, 154 }, { 191, 154 }, { 192, 155 }, { 194, 155 }, { 200, 161 }, { 200, 162 }, { 201, 163 }, { 201, 166 }, { 202, 167 }, { 202, 169 }, { 203, 170 }, { 203, 171 }, { 204, 172 }, { 204, 175 }, { 205, 176 }, { 205, 177 }, { 209, 181 }, { 210, 181 }, { 211, 182 }, { 245, 182 }, { 246, 181 }, { 250, 181 }, { 251, 180 }, { 254, 180 }, { 255, 179 }, { 260, 179 }, { 261, 178 }, { 268, 178 }, { 269, 177 }, { 271, 177 }, { 272, 176 }, { 275, 176 }, { 276, 175 }, { 277, 175 }, { 278, 174 }, { 281, 174 }, { 282, 173 }, { 284, 173 }, { 285, 172 }, { 286, 172 }, { 287, 173 }, { 287, 176 }, { 285, 178 }, { 285, 179 }, { 284, 180 }, { 284, 181 }, { 283, 182 }, { 283, 184 }, { 282, 185 }, { 282, 186 }, { 281, 187 }, { 281, 188 }, { 279, 190 }, { 279, 191 }, { 278, 192 }, { 278, 204 }, { 279, 205 }, { 279, 206 }, { 283, 210 }, { 284, 210 }, { 285, 211 }, { 287, 211 }, { 288, 212 }, { 289, 212 }, { 290, 213 }, { 291, 213 }, { 293, 215 }, { 294, 215 }, { 295, 216 }, { 296, 216 }, { 297, 217 }, { 298, 217 }, { 299, 218 }, { 302, 218 }, { 303, 219 }, { 305, 219 }, { 306, 220 }, { 307, 220 }, { 308, 221 }, { 309, 221 }, { 310, 222 }, { 313, 222 }, { 314, 223 }, { 322, 223 }, { 323, 224 }, { 333, 224 }, { 334, 223 }, { 335, 224 }, { 346, 224 }, { 347, 225 }, { 393, 225 }, { 394, 226 }, { 397, 226 }, { 398, 227 }, { 402, 227 }, { 403, 228 }, { 404, 228 }, { 405, 229 }, { 406, 229 }, { 407, 230 }, { 408, 230 }, { 409, 231 }, { 411, 231 }, { 412, 232 }, { 413, 232 }, { 418, 237 }, { 419, 237 }, { 422, 240 }, { 422, 241 }, { 425, 244 }, { 426, 244 }, { 427, 245 }, { 433, 245 }, { 434, 244 }, { 435, 244 }, { 436, 243 }, { 437, 243 }, { 440, 240 }, { 440, 239 }, { 441, 238 }, { 441, 232 }, { 440, 231 }, { 440, 230 }, { 439, 229 }, { 439, 228 }, { 437, 226 }, { 437, 225 }, { 436, 224 }, { 436, 223 }, { 434, 221 }, { 434, 220 }, { 433, 219 }, { 433, 217 }, { 432, 216 }, { 432, 215 }, { 429, 212 }, { 429, 211 }, { 427, 209 }, { 427, 208 }, { 426, 207 }, { 426, 206 }, { 424, 204 }, { 424, 201 }, { 423, 200 }, { 423, 199 }, { 422, 198 }, { 422, 196 }, { 421, 195 }, { 421, 194 }, { 420, 193 }, { 420, 192 }, { 419, 191 }, { 419, 189 }, { 418, 188 }, { 418, 187 }, { 417, 186 }, { 417, 183 }, { 416, 182 }, { 416, 181 }, { 415, 180 }, { 415, 178 }, { 414, 177 }, { 414, 176 }, { 413, 175 }, { 413, 174 }, { 412, 173 }, { 412, 170 }, { 411, 169 }, { 411, 168 }, { 409, 166 }, { 409, 165 }, { 408, 164 }, { 408, 163 }, { 407, 162 }, { 407, 160 }, { 406, 159 }, { 406, 158 }, { 405, 157 }, { 405, 156 }, { 404, 155 }, { 404, 154 }, { 403, 153 }, { 403, 152 }, { 401, 150 }, { 401, 149 }, { 400, 148 }, { 400, 146 }, { 399, 145 }, { 399, 144 }, { 398, 143 }, { 398, 142 }, { 397, 141 }, { 397, 140 }, { 396, 139 }, { 396, 138 }, { 394, 136 }, { 394, 133 }, { 393, 132 }, { 393, 131 }, { 392, 130 }, { 392, 129 }, { 390, 127 }, { 390, 126 }, { 389, 125 }, { 389, 123 }, { 388, 122 }, { 388, 121 }, { 387, 120 }, { 387, 119 }, { 386, 118 }, { 386, 117 }, { 385, 116 }, { 385, 114 }, { 384, 113 }, { 384, 112 }, { 383, 111 }, { 383, 110 }, { 382, 109 }, { 382, 108 }, { 381, 107 }, { 381, 106 }, { 379, 104 }, { 379, 103 }, { 378, 102 }, { 378, 101 }, { 377, 100 }, { 377, 98 }, { 376, 97 }, { 376, 96 }, { 375, 95 }, { 375, 94 }, { 374, 93 }, { 374, 92 }, { 372, 90 }, { 372, 89 }, { 371, 88 }, { 371, 85 }, { 370, 84 }, { 370, 83 }, { 367, 80 }, { 366, 80 }, { 365, 79 } };
		geom2d::polygon<int> result = { {448, 245}, { 229, 245 }, { 149, 79 }, { 368, 79 } };
		auto mec = geom2d::min_enclosing_parallelogram( points );
		for ( std::size_t i = 0; i < mec.size(); i++ ) {
			assert( mec[i].round() == result[i] );
		}
	}

	{
		geom2d::point_cloud<double> points = { {299, 70}, { 298, 71 }, { 290, 71 }, { 289, 72 }, { 288, 72 }, { 287, 73 }, { 286, 73 }, { 285, 74 }, { 282, 74 }, { 281, 75 }, { 280, 75 }, { 279, 76 }, { 278, 76 }, { 277, 77 }, { 275, 77 }, { 274, 78 }, { 273, 78 }, { 272, 79 }, { 270, 79 }, { 269, 80 }, { 268, 80 }, { 267, 81 }, { 266, 81 }, { 265, 82 }, { 263, 82 }, { 262, 83 }, { 261, 83 }, { 259, 85 }, { 257, 85 }, { 256, 86 }, { 255, 86 }, { 253, 88 }, { 252, 88 }, { 251, 89 }, { 249, 89 }, { 248, 90 }, { 247, 90 }, { 246, 91 }, { 245, 91 }, { 244, 92 }, { 243, 92 }, { 241, 94 }, { 240, 94 }, { 239, 95 }, { 238, 95 }, { 233, 100 }, { 232, 100 }, { 231, 101 }, { 230, 101 }, { 227, 104 }, { 226, 104 }, { 225, 105 }, { 224, 105 }, { 221, 108 }, { 221, 109 }, { 220, 110 }, { 220, 111 }, { 216, 115 }, { 215, 115 }, { 207, 123 }, { 207, 124 }, { 206, 125 }, { 206, 126 }, { 205, 127 }, { 205, 128 }, { 204, 129 }, { 204, 130 }, { 203, 131 }, { 203, 133 }, { 202, 134 }, { 202, 135 }, { 200, 137 }, { 200, 138 }, { 199, 139 }, { 199, 140 }, { 198, 141 }, { 198, 142 }, { 197, 143 }, { 197, 145 }, { 196, 146 }, { 196, 147 }, { 195, 148 }, { 195, 164 }, { 196, 165 }, { 196, 172 }, { 197, 173 }, { 197, 174 }, { 200, 177 }, { 201, 177 }, { 202, 178 }, { 208, 178 }, { 209, 177 }, { 210, 177 }, { 213, 174 }, { 213, 173 }, { 214, 172 }, { 214, 160 }, { 213, 159 }, { 213, 154 }, { 214, 153 }, { 214, 152 }, { 215, 151 }, { 215, 148 }, { 217, 146 }, { 217, 145 }, { 218, 144 }, { 218, 143 }, { 219, 142 }, { 219, 141 }, { 220, 140 }, { 220, 139 }, { 222, 137 }, { 222, 136 }, { 223, 135 }, { 223, 134 }, { 237, 120 }, { 238, 120 }, { 243, 115 }, { 244, 115 }, { 247, 112 }, { 248, 112 }, { 249, 111 }, { 250, 111 }, { 251, 110 }, { 252, 110 }, { 255, 107 }, { 256, 107 }, { 257, 106 }, { 258, 106 }, { 259, 105 }, { 260, 105 }, { 261, 104 }, { 262, 104 }, { 263, 103 }, { 265, 103 }, { 266, 102 }, { 267, 102 }, { 268, 101 }, { 269, 101 }, { 271, 99 }, { 272, 99 }, { 273, 98 }, { 274, 98 }, { 275, 97 }, { 278, 97 }, { 279, 96 }, { 280, 96 }, { 281, 95 }, { 283, 95 }, { 284, 94 }, { 285, 94 }, { 286, 93 }, { 287, 93 }, { 288, 92 }, { 290, 92 }, { 291, 91 }, { 292, 91 }, { 293, 90 }, { 294, 90 }, { 295, 89 }, { 303, 89 }, { 304, 88 }, { 335, 88 }, { 336, 89 }, { 337, 89 }, { 338, 90 }, { 341, 90 }, { 342, 91 }, { 344, 91 }, { 345, 92 }, { 346, 92 }, { 347, 93 }, { 348, 93 }, { 349, 94 }, { 350, 94 }, { 351, 95 }, { 353, 95 }, { 354, 96 }, { 355, 96 }, { 356, 97 }, { 358, 97 }, { 359, 98 }, { 360, 98 }, { 361, 99 }, { 362, 99 }, { 364, 101 }, { 365, 101 }, { 366, 102 }, { 367, 102 }, { 368, 103 }, { 369, 103 }, { 370, 104 }, { 372, 104 }, { 373, 105 }, { 374, 105 }, { 375, 106 }, { 376, 106 }, { 381, 111 }, { 382, 111 }, { 384, 113 }, { 385, 113 }, { 386, 114 }, { 387, 114 }, { 389, 116 }, { 389, 117 }, { 392, 120 }, { 393, 120 }, { 398, 125 }, { 398, 126 }, { 405, 133 }, { 405, 134 }, { 410, 139 }, { 410, 140 }, { 412, 142 }, { 412, 143 }, { 417, 148 }, { 417, 149 }, { 418, 150 }, { 418, 151 }, { 420, 153 }, { 420, 154 }, { 426, 160 }, { 426, 161 }, { 427, 162 }, { 427, 163 }, { 428, 164 }, { 428, 165 }, { 429, 166 }, { 429, 167 }, { 431, 169 }, { 431, 174 }, { 432, 175 }, { 432, 176 }, { 433, 177 }, { 433, 179 }, { 434, 180 }, { 434, 181 }, { 435, 182 }, { 435, 185 }, { 436, 186 }, { 436, 191 }, { 437, 192 }, { 437, 194 }, { 438, 195 }, { 438, 204 }, { 439, 205 }, { 439, 208 }, { 440, 209 }, { 440, 224 }, { 439, 225 }, { 439, 228 }, { 438, 229 }, { 438, 230 }, { 437, 231 }, { 437, 232 }, { 436, 233 }, { 436, 237 }, { 435, 238 }, { 435, 239 }, { 434, 240 }, { 434, 241 }, { 432, 243 }, { 432, 244 }, { 423, 253 }, { 423, 254 }, { 422, 255 }, { 422, 256 }, { 421, 257 }, { 420, 257 }, { 415, 262 }, { 414, 262 }, { 413, 263 }, { 412, 263 }, { 410, 265 }, { 409, 265 }, { 408, 266 }, { 406, 266 }, { 405, 267 }, { 404, 267 }, { 401, 270 }, { 399, 270 }, { 398, 271 }, { 397, 271 }, { 396, 272 }, { 394, 272 }, { 393, 273 }, { 392, 273 }, { 391, 274 }, { 386, 274 }, { 385, 275 }, { 382, 275 }, { 381, 276 }, { 380, 276 }, { 379, 277 }, { 367, 277 }, { 366, 278 }, { 361, 278 }, { 360, 279 }, { 351, 279 }, { 350, 280 }, { 318, 280 }, { 317, 279 }, { 316, 279 }, { 315, 278 }, { 312, 278 }, { 310, 276 }, { 309, 276 }, { 305, 272 }, { 304, 272 }, { 302, 270 }, { 302, 269 }, { 295, 262 }, { 295, 261 }, { 294, 260 }, { 294, 259 }, { 288, 253 }, { 288, 252 }, { 287, 251 }, { 287, 250 }, { 285, 248 }, { 285, 247 }, { 284, 246 }, { 284, 245 }, { 282, 243 }, { 282, 241 }, { 281, 240 }, { 281, 239 }, { 280, 238 }, { 280, 237 }, { 279, 236 }, { 279, 232 }, { 278, 231 }, { 278, 230 }, { 277, 229 }, { 277, 228 }, { 276, 227 }, { 276, 225 }, { 275, 224 }, { 275, 223 }, { 274, 222 }, { 274, 221 }, { 273, 220 }, { 273, 217 }, { 272, 216 }, { 272, 215 }, { 271, 214 }, { 271, 210 }, { 270, 209 }, { 270, 204 }, { 269, 203 }, { 269, 188 }, { 270, 187 }, { 270, 186 }, { 271, 185 }, { 271, 183 }, { 282, 172 }, { 284, 172 }, { 285, 171 }, { 286, 171 }, { 287, 170 }, { 289, 170 }, { 290, 169 }, { 297, 169 }, { 298, 170 }, { 301, 170 }, { 302, 171 }, { 305, 171 }, { 307, 173 }, { 308, 173 }, { 309, 174 }, { 313, 174 }, { 314, 175 }, { 317, 175 }, { 318, 176 }, { 319, 176 }, { 321, 178 }, { 322, 178 }, { 323, 179 }, { 324, 179 }, { 325, 180 }, { 328, 180 }, { 329, 181 }, { 331, 181 }, { 332, 182 }, { 333, 182 }, { 335, 184 }, { 336, 184 }, { 337, 185 }, { 338, 185 }, { 340, 187 }, { 341, 187 }, { 343, 189 }, { 344, 189 }, { 349, 194 }, { 349, 195 }, { 350, 196 }, { 350, 197 }, { 352, 199 }, { 352, 200 }, { 353, 201 }, { 353, 202 }, { 354, 203 }, { 354, 205 }, { 355, 206 }, { 355, 210 }, { 354, 211 }, { 352, 211 }, { 351, 212 }, { 348, 212 }, { 347, 213 }, { 341, 213 }, { 340, 214 }, { 336, 214 }, { 335, 213 }, { 334, 213 }, { 330, 209 }, { 330, 208 }, { 328, 206 }, { 328, 205 }, { 325, 202 }, { 324, 202 }, { 323, 201 }, { 317, 201 }, { 316, 202 }, { 315, 202 }, { 312, 205 }, { 312, 206 }, { 311, 207 }, { 311, 213 }, { 312, 214 }, { 312, 215 }, { 313, 216 }, { 313, 217 }, { 314, 218 }, { 314, 219 }, { 315, 220 }, { 315, 221 }, { 321, 227 }, { 322, 227 }, { 325, 230 }, { 326, 230 }, { 327, 231 }, { 328, 231 }, { 329, 232 }, { 345, 232 }, { 346, 231 }, { 352, 231 }, { 353, 230 }, { 356, 230 }, { 357, 229 }, { 360, 229 }, { 361, 228 }, { 363, 228 }, { 364, 227 }, { 365, 227 }, { 371, 221 }, { 371, 220 }, { 372, 219 }, { 372, 218 }, { 373, 217 }, { 373, 201 }, { 372, 200 }, { 372, 198 }, { 371, 197 }, { 371, 196 }, { 370, 195 }, { 370, 194 }, { 369, 193 }, { 369, 191 }, { 368, 190 }, { 368, 189 }, { 366, 187 }, { 366, 186 }, { 365, 185 }, { 365, 184 }, { 355, 174 }, { 354, 174 }, { 352, 172 }, { 351, 172 }, { 347, 168 }, { 346, 168 }, { 345, 167 }, { 344, 167 }, { 342, 165 }, { 341, 165 }, { 340, 164 }, { 337, 164 }, { 336, 163 }, { 334, 163 }, { 333, 162 }, { 331, 162 }, { 330, 161 }, { 329, 161 }, { 328, 160 }, { 327, 160 }, { 326, 159 }, { 325, 159 }, { 324, 158 }, { 323, 158 }, { 322, 157 }, { 319, 157 }, { 318, 156 }, { 316, 156 }, { 315, 155 }, { 314, 155 }, { 313, 154 }, { 312, 154 }, { 311, 153 }, { 307, 153 }, { 306, 152 }, { 303, 152 }, { 302, 151 }, { 285, 151 }, { 284, 152 }, { 281, 152 }, { 280, 153 }, { 279, 153 }, { 278, 154 }, { 276, 154 }, { 275, 155 }, { 274, 155 }, { 273, 156 }, { 272, 156 }, { 271, 157 }, { 270, 157 }, { 257, 170 }, { 257, 171 }, { 256, 172 }, { 256, 173 }, { 255, 174 }, { 255, 175 }, { 254, 176 }, { 254, 177 }, { 253, 178 }, { 253, 180 }, { 252, 181 }, { 252, 182 }, { 251, 183 }, { 251, 208 }, { 252, 209 }, { 252, 214 }, { 253, 215 }, { 253, 220 }, { 254, 221 }, { 254, 222 }, { 255, 223 }, { 255, 225 }, { 256, 226 }, { 256, 227 }, { 257, 228 }, { 257, 229 }, { 258, 230 }, { 258, 234 }, { 259, 235 }, { 259, 236 }, { 261, 238 }, { 261, 243 }, { 262, 244 }, { 262, 245 }, { 264, 247 }, { 264, 249 }, { 265, 250 }, { 265, 251 }, { 267, 253 }, { 267, 254 }, { 269, 256 }, { 269, 257 }, { 270, 258 }, { 270, 259 }, { 272, 261 }, { 272, 262 }, { 273, 263 }, { 273, 264 }, { 279, 270 }, { 279, 271 }, { 280, 272 }, { 280, 273 }, { 286, 279 }, { 286, 280 }, { 294, 288 }, { 295, 288 }, { 298, 291 }, { 299, 291 }, { 301, 293 }, { 302, 293 }, { 303, 294 }, { 304, 294 }, { 305, 295 }, { 306, 295 }, { 307, 296 }, { 309, 296 }, { 310, 297 }, { 311, 297 }, { 312, 298 }, { 355, 298 }, { 356, 297 }, { 365, 297 }, { 366, 296 }, { 371, 296 }, { 372, 295 }, { 385, 295 }, { 386, 294 }, { 387, 294 }, { 388, 293 }, { 390, 293 }, { 391, 292 }, { 396, 292 }, { 397, 291 }, { 398, 291 }, { 399, 290 }, { 401, 290 }, { 402, 289 }, { 403, 289 }, { 404, 288 }, { 407, 288 }, { 408, 287 }, { 409, 287 }, { 410, 286 }, { 411, 286 }, { 413, 284 }, { 414, 284 }, { 415, 283 }, { 416, 283 }, { 417, 282 }, { 418, 282 }, { 420, 280 }, { 421, 280 }, { 423, 278 }, { 424, 278 }, { 425, 277 }, { 426, 277 }, { 431, 272 }, { 432, 272 }, { 433, 271 }, { 434, 271 }, { 437, 268 }, { 437, 267 }, { 438, 266 }, { 438, 265 }, { 446, 257 }, { 446, 256 }, { 447, 255 }, { 447, 254 }, { 449, 252 }, { 449, 251 }, { 450, 250 }, { 450, 249 }, { 451, 248 }, { 451, 247 }, { 452, 246 }, { 452, 245 }, { 453, 244 }, { 453, 243 }, { 454, 242 }, { 454, 238 }, { 455, 237 }, { 455, 236 }, { 456, 235 }, { 456, 234 }, { 457, 233 }, { 457, 230 }, { 458, 229 }, { 458, 204 }, { 457, 203 }, { 457, 200 }, { 456, 199 }, { 456, 190 }, { 455, 189 }, { 455, 187 }, { 454, 186 }, { 454, 181 }, { 453, 180 }, { 453, 177 }, { 452, 176 }, { 452, 175 }, { 451, 174 }, { 451, 171 }, { 450, 170 }, { 450, 169 }, { 449, 168 }, { 449, 164 }, { 448, 163 }, { 448, 162 }, { 447, 161 }, { 447, 160 }, { 446, 159 }, { 446, 158 }, { 444, 156 }, { 444, 155 }, { 443, 154 }, { 443, 152 }, { 442, 151 }, { 442, 150 }, { 438, 146 }, { 438, 145 }, { 433, 140 }, { 433, 139 }, { 432, 138 }, { 432, 137 }, { 430, 135 }, { 430, 134 }, { 425, 129 }, { 425, 128 }, { 420, 123 }, { 420, 122 }, { 419, 121 }, { 419, 120 }, { 415, 116 }, { 415, 115 }, { 414, 114 }, { 414, 113 }, { 411, 110 }, { 410, 110 }, { 408, 108 }, { 408, 107 }, { 405, 104 }, { 404, 104 }, { 403, 103 }, { 402, 103 }, { 398, 99 }, { 397, 99 }, { 396, 98 }, { 395, 98 }, { 393, 96 }, { 392, 96 }, { 387, 91 }, { 386, 91 }, { 385, 90 }, { 384, 90 }, { 382, 88 }, { 381, 88 }, { 380, 87 }, { 379, 87 }, { 378, 86 }, { 375, 86 }, { 373, 84 }, { 372, 84 }, { 371, 83 }, { 370, 83 }, { 368, 81 }, { 367, 81 }, { 366, 80 }, { 365, 80 }, { 364, 79 }, { 361, 79 }, { 360, 78 }, { 359, 78 }, { 358, 77 }, { 356, 77 }, { 355, 76 }, { 354, 76 }, { 353, 75 }, { 352, 75 }, { 351, 74 }, { 350, 74 }, { 349, 73 }, { 347, 73 }, { 346, 72 }, { 343, 72 }, { 342, 71 }, { 341, 71 }, { 340, 70 } };
		geom2d::polygon<int> result = { {162, 132}, { 351, 38 }, { 515, 234 }, { 326, 328 } };
		auto mec = geom2d::min_enclosing_parallelogram( points );
		for ( std::size_t i = 0; i < mec.size(); i++ ) {
			assert( mec[i].round() == result[i] );
		}
	}

	std::cout << "- enclosing parallelogram tests successful" << std::endl;
}

void test_segment_intersections() {
	geom2d::LineSegment<double> s1_d( { { -1,0 },{ 1, 0 } } );
	geom2d::LineSegment<double> s2_d( { { -1,0 },{ 1, 0 } } );

	geom2d::LineSegment<int> s1( { {-1,0},{1,0} } );
	geom2d::LineSegment<int> s2( { { -1,1 },{ 1,1 } } );

	auto i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, -1.0 ) && geom::in_range( i.second, -1.0 ) );

	s2 = { { 0,1 },{ 1,-1 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.75 ) && geom::in_range( i.second, 0.5 ) );

	s2 = { { 1,1 },{ -1,-1 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.5 ) );

	//identical
	s2 = { { -1,0 },{ 1, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.5 ) );

	i = geom2d::segment_intersection( s2, s1, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.5 ) );

	//s1 in s2
	s2 = { { -2,0 },{ 1, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 2.0/3.0 ) );

	s2 = { { 1,0 },{ -2, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 1.0 / 3.0 ) );

	s2 = { { -1,0 },{ 2, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 1.0 / 3.0 ) );

	s2 = { { 2,0 },{ -1, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 2.0 / 3.0 ) );

	s2 = { { -2,0 },{ 2, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.5 ) );

	s2 = { { 2,0 },{ -2, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.5 ) );
	
	//s2 in s1
	s2_d = { { 0.75,0 },{ -0.25, 0 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 5.0/8.0 ) && geom::in_range( i.second, 0.5 ) );

	//overlapping
	s2 = { { -2,0 },{ 0, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.25 ) && geom::in_range( i.second, 0.75 ) );

	s2 = { { 0,0 },{ -2, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.25 ) && geom::in_range( i.second, 0.25 ) );

	s2 = { { -2,0 },{ 0, 0 } };
	i = geom2d::segment_intersection( s2, s1, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.75 ) && geom::in_range( i.second, 0.25 ) );

	s2 = { { 0,0 },{ -2, 0 } };
	i = geom2d::segment_intersection( s2, s1, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.25 ) && geom::in_range( i.second, 0.25 ) );

	s2 = { { 0,0 },{ 2, 0 } };
	i = geom2d::segment_intersection( s1, s2, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.75 ) && geom::in_range( i.second, 0.25 ) );

	s2 = { { 0,0 },{ 2, 0 } };
	i = geom2d::segment_intersection( s2, s1, geom2d::OverlapStrategy::ALLOW_OVERLAP );
	assert( geom::in_range( i.first, 0.25 ) && geom::in_range( i.second, 0.75 ) );

	//Touching
	s2_d = { { -2,0 },{ -1, 0 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 0.0 ) && geom::in_range( i.second, 1.0 ) );
	
	s2_d = { { -2,0 },{ -1, 0 } };
	i = geom2d::segment_intersection( s2_d, s1_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 1.0 ) && geom::in_range( i.second, 0.0 ) );

	s2_d = { { -1,0 },{ -2, 0 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 0.0 ) && geom::in_range( i.second, 0.0 ) );

	s2_d = { { 1,0 },{ 24.654, 234.6874 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 1.0 ) && geom::in_range( i.second, 0.0 ) );
	
	s2_d = { { 1,0 },{ -24.654, -234.6874 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 1.0 ) && geom::in_range( i.second, 0.0 ) );

	s2_d = { { 0,0 },{ -24.654, -234.6874 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.0 ) );

	s2_d = { { 0,0 },{ 24.654, 234.6874 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 0.5 ) && geom::in_range( i.second, 0.0 ) );

	s2_d = { { -24.654, -234.6874 }, {0.8675423, 0} };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 1.8675423/2.0, 10 ) && geom::in_range( i.second, 1.0 ) );

	s2_d = { { 1, 1 },{ 0.8675423, 1e-12 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, -1.0 ) && geom::in_range( i.second, -1.0 ) );

	s2_d = { { -2, 0 },{ 0, 1e-20 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, -1.0 ) && geom::in_range( i.second, -1.0 ) );

	s2_d = { { 0.5, 0 },{ 1, 1 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 0.75 ) && geom::in_range( i.second, 0.0 ) );

	s2_d = { { 1,1 },{ 0.5, 0 } };
	i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING );
	assert( geom::in_range( i.first, 0.75 ) && geom::in_range( i.second, 1.0 ) );

	//Error cases
	s2_d = { { -2, 0 },{ -1.0 + 1e-16, 0 } };
	bool caught = false;
	i = { -2,-2 };
	try {
		i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING ); //Overlapping but only touching allowed
	}
	catch ( std::exception ex ) {
		caught = true;
	}
	assert( caught );
	assert( i == std::make_pair( -2.0, -2.0 ) );

	s2_d = { { 1.0 - 1e16, 0 },{ 2, 0 } };
	caught = false;
	i = { -2,-2 };
	try {
		i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_TOUCHING ); //Overlapping but only touching allowed
	}
	catch ( std::exception ex ) {
		caught = true;
	}
	assert( caught );
	assert( i == std::make_pair( -2.0, -2.0 ) );

	s2_d = { { -2, 0 },{ -1.0, 0 } };
	caught = false;
	i = {-2,-2};
	try {
		i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_NO_OVERLAP ); //Touching though not allowed
	}
	catch ( std::exception ex ) {
		caught = true;
	}
	assert( caught );
	assert( i == std::make_pair(-2.0, -2.0) );

	s2_d = { { -1, 2 },{ -1, 0 } };
	caught = false;
	i = { -2,-2 };
	try {
		i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_NO_OVERLAP ); //Touching though not allowed
	}
	catch ( std::exception ex ) {
		caught = true;
	}
	assert( caught );
	assert( i == std::make_pair( -2.0, -2.0 ) );

	s2_d = { { 0.5, 0 },{ 1, 1 } };
	caught = false;
	i = { -2,-2 };
	try {
		i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_NO_OVERLAP ); //Touching though not allowed
	}
	catch ( std::exception ex ) {
		caught = true;
	}
	assert( caught );
	assert( i == std::make_pair( -2.0, -2.0 ) );

	s2_d = { { -1, -1 },{ 0.5, 0 } };
	caught = false;
	i = { -2,-2 };
	try {
		i = geom2d::segment_intersection( s1_d, s2_d, geom2d::OverlapStrategy::ALLOW_NO_OVERLAP ); //Touching though not allowed
	}
	catch ( std::exception ex ) {
		caught = true;
	}
	assert( caught );
	assert( i == std::make_pair( -2.0, -2.0 ) );

	std::cout << "- segment intersection tests successful" << std::endl;
}



int main(){
    std::cout << "Starting *geom::* tests:" << std::endl;

    //vector initialization; vector operators, pythagoras, norm & dist
    test_basic_funcionality();

    //angles between zero-based vectors
    test_angles();

	//segment intersections
	test_segment_intersections();

    //convex hull
    test_cvhull();

    //point in polygon
    test_point_in_polygon();

	//polygon area
	test_polygon_area();

	//Rectangles
	test_rectangles();

	//min encl parallelogram
	test_encl_parallelogram();

    std::cout << "Congrats! All tests succesful." << std::endl;

	return 0;
}
