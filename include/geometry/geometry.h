// Copyright Ingo Proff 2016.
// https://github.com/CrikeeIP/Geometry
// Distributed under the MIT Software License (X11 license)
// (See accompanying file LICENSE)



#pragma once

#include <vector>
#include <string>
#include <cfloat> // DBL_MAX
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <type_traits>



namespace geom {



///////////
//constants
///////////

const double pi = 4.0 * std::atan( 1.0 );

const double e = std::exp( 1.0 );


//////////////////
//Helper functions
//////////////////

inline int sign( double x ) {
	return x == 0.0 ? 0 : (x<0 ? -1 : 1);
}

inline int sign( int x ) {
	return x == 0 ? 0 : (x<0 ? -1 : 1);
}

inline bool in_range( double goal, double x ) {
	double min = std::nextafter( goal, -DBL_MAX );
	double max = std::nextafter(goal, DBL_MAX );
	return min <= x && x <= max;
}

inline bool in_range( float x, float goal ) {
	float min = std::nextafter( goal, -FLT_MAX );
	float  max = std::nextafter( goal, FLT_MAX );
	return min <= x && x <= max;
}

inline bool in_range( int x, int goal ) {
	return x == goal;
}



///////////////
//Vector struct
///////////////

template <typename T, std::size_t N>
struct Vec {
	Vec() : coordinates( N, 0 ) {};
	Vec( const std::vector<T>& coords ) : coordinates(coords){
		assert( coords.size() == N );
	}
	Vec( const Vec<T, N>& vec ) : coordinates(vec.coordinates) {}
	Vec( const T& x ) : coordinates( N, x ) {}
	Vec( const std::initializer_list<T>& coord_list ) : coordinates( coord_list ) {
		assert( coordinates.size() == N );
	}
	
	template<typename = typename std::enable_if<N==2>>
	Vec( T x, T y ) : coordinates( { x, y } ) {}

	template<typename = typename std::enable_if<N==3>>
	Vec( T x, T y, T z ) : coordinates( { x, y, z } ) {}


	std::size_t dimension() {
		return coordinates.size;
	}

	T operator[] ( std::size_t n ) const {
		assert( n < coordinates.size() );
		return coordinates[n];
	}

	template< typename = typename std::enable_if< std::is_convertible<T,double>::value > > //TODO: not disabled from string to double?!
	Vec<double, N> convert_to_doubles() {
		std::vector<double> result;
		result.reserve( N );
		for ( const auto& x : coordinates ) {
			result.push_back( static_cast<double>(x) );
		}
		return{ result };
	}

    inline std::string print() const{
        std::string result ("(");
        for( std::size_t i = 0; i < coordinates.size()-1; i++){
            result += std::to_string( coordinates[i] ) + ",";
        }
        return result + std::to_string( coordinates.back() ) + ")";
    }

private:
	std::vector<T> coordinates;
};

//make_vec because class template arguments (in their class constructors) cannot be deduced
template <std::size_t N, typename T, typename... Args>
geom::Vec<T,N> make_vec( T x, Args&&... xs )
{
	std::size_t size = 1 + sizeof...( xs );
	assert( N == size );
	return{ x, xs... };
}



////////////////////
//Point cloud struct
////////////////////

template <typename T, std::size_t N>
using point_cloud = std::vector<Vec<T, N>>; //TODO: switch to general container, not necessarily vector



///////////
//Internals
///////////

namespace internal{
	//recursive pythagoras template
	template <typename T, size_t N, size_t I>
	struct compute_pythagoras
	{
		//Computes square_sum(v1-v2)
		static inline T apply( const Vec<T, N>& v1, const Vec<T, N>& v2 )
		{
			T const c1 = v1[I];
			T const c2 = v2[I];
			T const d = c1 - c2;
			return d * d + compute_pythagoras<T, N, I - 1>::apply( v1, v2 );
		}
	};

	template <typename T, size_t N>
	struct compute_pythagoras<T, N, 0>
	{
		static inline T apply( const Vec<T, N>& v1, const Vec<T, N>& v2 )
		{
			T const c1 = v1[0];
			T const c2 = v2[0];
			T const d = c1 - c2;
			return d * d;
		}
	};
} ///namespace internal


/////////////////
//Vector operator
/////////////////

template <typename T, std::size_t N>
bool operator == ( const Vec<T,N>& lhs, const Vec<T,N>& rhs ) {
	for ( std::size_t i = 0; i < N; i++ ) {
		if ( lhs[i] != rhs[i] ) {
			return false;
		}
	}
	return true;
}

template <typename T, std::size_t N>
bool operator != ( const Vec<T,N>& lhs, const Vec<T,N>& rhs ) {
	return !(lhs == rhs);
}

template <typename T, std::size_t N>
Vec<T,N> operator+( const Vec<T,N>& lhs, const Vec<T,N>& rhs ) {
	std::vector<T> result;
	result.reserve( N );
	for ( std::size_t i = 0; i < N; i++ ) {
		result.push_back( lhs[i] + rhs[i] );
	}
	return{ result };
}

template <typename T, std::size_t N>
Vec<T, N> operator-( const Vec<T, N>& lhs, const Vec<T, N>& rhs ) {
	std::vector<T> result;
	result.reserve( N );
	for ( std::size_t i = 0; i < N; i++ ) {
		result.push_back( lhs[i] - rhs[i] );
	}
	return{ result };
}

template <typename T, std::size_t N>
T operator*( const Vec<T, N>& lhs, const Vec<T, N>& rhs ) {
	T result = 0;
	for ( std::size_t i = 0; i < N; i++ ) {
		result += lhs[i] * rhs[i];
	}
	return{ result };
}

template <typename T, std::size_t N>
Vec<T, N> operator*( const Vec<T,N>& vec, T factor ) {
	std::vector<T> result;
	result.reserve( N );
	for ( std::size_t i = 0; i < N; i++ ) {
		result.push_back( vec[i] * factor );
	}
	return{ result };
}

template <typename T, std::size_t N>
Vec<T, N> operator*( T factor, const Vec<T, N>& vec) {
	return vec * factor;
}

template <typename T, std::size_t N>
Vec<T, N> operator/( const Vec<T, N>& vec, T divisor ) {
	std::vector<T> result;
	result.reserve( N );
	for ( std::size_t i = 0; i < N; i++ ) {
		result.push_back( vec[i] / divisor );
	}
	return{ result };
}

template <typename T, std::size_t N>
std::ostream& operator<<( std::ostream& os, const Vec<T,N>& vec ) {
	os << vec.print();
	return os;
}




///////////////////
//Vector Arithmetic
///////////////////

template<typename T, std::size_t N>
double norm( const Vec<T, N>& vec ) {
	return std::sqrt( vec*vec );
}


//typename std::enable_if<std::is_convertible<T, double>>::value
template<typename T, std::size_t N>
Vec<double,N> normalize( const Vec<double,N>& vec ) {
	double length = norm( vec );
	assert( !in_range( length, 0.0 ) ); ///You really should know better than normalizing a vector with length 0 just like that
	return vec / length;
}

template<typename T, std::size_t N>
double square_dist( const Vec<T,N>& p1, const Vec<T,N>& p2 ) {
	return internal::compute_pythagoras<T, N, N-1>::apply( p1, p2 );
}

template<typename T, std::size_t N>
double dist( const Vec<T,N>& p1, const Vec<T,N>& p2 ) {
	return std::sqrt( square_dist( p1, p2 ) );
}

}//namespace geom





namespace geom2d {

///////////
//Constants
///////////

#define ALLOW_NO_OVERLAP 0
#define ALLOW_TOUCHING 1
#define ALLOW_OVERLAP 2

//////////////////
//Unit conversions
//////////////////

inline double rad_to_deg( double rad ) {
	return rad * 180 / geom::pi;
}

inline double arc_to_angle( double circle_radius, double arc_length ) {
	assert( circle_radius > 0.0 );
	assert( arc_length >= 0.0 );
	return arc_length * 180.0 / (geom::pi * circle_radius);
}

inline double angle_to_arc( double circle_radius, double angle ) {
	assert( circle_radius > 0 );
	return (geom::pi * circle_radius * angle) / 180.0;
}



///////////////
//Vector struct
///////////////

template <typename T>
using Vec2D = geom::Vec<T, 2>;



//////////////////
//Vector operators
//////////////////

//Returns the cross product of the given vectors, i.e. the area of the parallelogram that the vectors span.
template <typename T>
T cross( const Vec2D<T>& vec1, const Vec2D<T>& vec2 ) {
	return vec1[0] * vec2[1] - vec1[1] * vec2[0];
}



////////////////
//Segment struct
////////////////

template <typename T>
struct LineSegment {
	LineSegment( Vec2D<T> x1, Vec2D<T> x2 ) : x1( x1 ), x2( x2 ) {};
	LineSegment( const LineSegment<T>& segment ) : x1( segment.x1 ), x2( segment.x2 ) {};

	Vec2D<T> x1;
	Vec2D<T> x2;
};



////////////////
//Polygon struct
////////////////
template <typename T>
using polygon = std::vector<Vec2D<T>>;
//typedef std::vector<Vec2D<double>> polygon;



///////////
//Internals
///////////

namespace internal{

	//Determines, whether or not a horizontal ray from x to the right intersects the segment s
	template<typename T>
	bool ray_intersects_segment(const Vec2D<T>& x, const LineSegment<T>& s){
	return (
			( (s.x1[1]>x[1]) != (s.x2[1]>x[1]) ) && ///y-coordinate of x lies between y-coordinates of p[i], p[i+1]
			( x[0] < s.x1[0] + ((s.x2[0]-s.x1[0]) / (s.x2[1]-s.x1[1])) * (x[1]-s.x1[1]) ) ///x is left of segment
		);
	}

	template<typename T>
	std::pair<double, double> segments_on_line_intersection( const LineSegment<T>& s1, const LineSegment<T>& s2, int allow_overlap ) {
		Vec2D<T> x_vec = s1.x2 - s1.x1;
		Vec2D<T> x1y1_vec = s2.x1 - s1.x1;
		Vec2D<T> x1y2_vec = s2.x2 - s1.x1;
		Vec2D<T> x2y1_vec = s2.x1 - s1.x2;
		Vec2D<T> x2y2_vec = s2.x2 - s1.x2;
		Vec2D<T> x_normal = normal( x_vec );

		int x1y1_sign = geom::sign( cross( x_normal, x1y1_vec ) );
		int x1y2_sign = geom::sign( cross( x_normal, x1y2_vec ) );
		int x2y1_sign = geom::sign( cross( x_normal, x2y1_vec ) );
		int x2y2_sign = geom::sign( cross( x_normal, x2y2_vec ) );

		if ( x1y1_sign == x1y2_sign  && x1y2_sign == x2y1_sign && x2y1_sign == x2y2_sign ) {
            //no intersection, because y1&y2 on the same side of the normal
            assert(x1y1_sign != 0); ///sanity check:
			return {-1,-1};
		}

		if(allow_overlap == ALLOW_OVERLAP){
			//TODO
			return {-1,-1};
		}
		else if(allow_overlap == ALLOW_TOUCHING){
			//TODO
			return {-1,-1};
		}
		else{
			std::cout << "Touching or overlapping parallel line segments, which have not been allowed:\n"
			"{ " << s1.x1.print() << " ; " << s1.x2.print() << " }   <>   { " << s2.x1.print() << " ; " << s2.x2.print() << " }" << std::endl;
			assert(false); //Overlapping segments //TODO: Exception?
		}
	}
}//namespace internal



///////////////////
//Vector Arithmetic
///////////////////

//Computes the vector rotated 90 deg counter-clockwise (preserves length)
template<typename T>
Vec2D<T> perpendicular( const Vec2D<T>& vec ) {
	return{ -vec[1], vec[0] };
}

//Computes the vector rotated 90 deg counter-clockwise and normalizes it to length 1
template<typename T>
Vec2D<T> normal( const Vec2D<T>& vec ) {
	return normalize( perpendicular( vec ) );
}

/**Computes the small angle (in [0,180]) enclosed by the two vectors.
* E.g.
anlge( {1,0}, {10,0} ) == 0.0
anlge( {1,0}, {1,1} ) == 45.0
anlge( {1,0}, {-1,-1} ) == 135.0
anlge( {1,0}, {-2,0} ) == 180.0

anlge( {1,0}, {0,-1} ) == 90
anlge( {1,0}, {-2,-1} ) == 153.435
*/
template<typename T>
double enclosed_angle( Vec2D<T> vec1, Vec2D<T> vec2 ) {
	double prod = vec1 * vec2;
	double norm1 = norm( vec1 );
	double norm2 = norm( vec2 );
	double arg = prod / (norm1 * norm2);
	double angle = std::acos( arg );
	angle = rad_to_deg( angle );
	return angle;
}

/**Computes the angle (in (-180,180]) between two vectors, measured counter-clockwise from the first vector ("vec1").
* E.g.
anlge( {1,0}, {10,0} ) == +0.0
anlge( {1,0}, {1,1} ) == +45.0
anlge( {1,0}, {-2,0} ) == +180.0
anlge( {1,0}, {0,-1} ) == -90
anlge( {1,0}, {-1,-1} ) == -135.0
anlge( {1,0}, {-2,-1} ) == -153.435
*/
template<typename T>
double angle( const Vec2D<T>& vec1, const Vec2D<T>& vec2 ) {
	double angle = enclosed_angle( vec1, vec2 );
	double side = cross( vec1, vec2 );
	return (side >= 0.0) ? angle : -angle;
}

/**Computes the angle (in [0,360]) between two vectors, measured counter-clockwise from the first vector ("vec1").
* E.g.
anlge( {1,0}, {10,0} ) == 0.0
anlge( {1,0}, {1,1} ) == 45.0
anlge( {1,0}, {-2,0} ) == 180.0
anlge( {1,0}, {-1,-1} ) == 225.0
anlge( {1,0}, {-2,-1} ) == 206.565
anlge( {1,0}, {0,-1} ) == 270
*/
template<typename T>
double positive_angle( const Vec2D<T>& vec1, const Vec2D<T>& vec2 ) {
	double angle = geom2d::angle( vec1, vec2 );
	double side = cross( vec1, vec2 );
	return (side >= 0.0) ? angle : (360.0 + angle);
}



///////////////////////
//Advanced Vector Stuff
///////////////////////

//Decides whether a point is inside a tiangle using barycentric coordinates
template<typename T, bool on_line_is_inside>
bool point_in_triangle( const Vec2D<T>& x, const Vec2D<T>& A, const Vec2D<T>& B, const Vec2D<T>& C){
	// Compute vectors
	auto v0 = C - A;
	auto v1 = B - A;
	auto v2 = x - A;

	//handle degenerate triangles
	if ( cross(v1, v0 ) == 0 ) {
		if ( !on_line_is_inside ) {
			return false;
		}
		Vec2D<T> s1( A );
		Vec2D<T> s2( (square_dist( A, B )<square_dist( A, C )) ? C : B );
		if ( s1 == s2 ) {
			return x == s1;
		}
		double t_x = (x[0] - s1[0]) / (s2[0] - s1[0]);
		double t_y = (x[1] - s1[1]) / (s2[1] - s1[1]);
		return t_x >= 0.0 && t_x <= 1.0 && geom::in_range( t_x, t_y );
	}

	// Compute dot products
	auto dot00 = v0 * v0;
	auto dot01 = v0 * v1;
	auto dot02 = v0 * v2;
	auto dot11 = v1 * v1;
	auto dot12 = v1 * v2;

	// Compute barycentric coordinates
	double invDenom = 1.0 / static_cast<double>(dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	if ( on_line_is_inside ){
		return (u >= 0) && (v >= 0) && (u + v <= 1);
	}
	else{
		return (u > 0) && (v > 0) && (u + v < 1);
	}
}

//Decides whether a point is inside a polygon
template<typename T>
bool point_in_polygon( const Vec2D<T>& x, const polygon<T>& p){
	//Check bounding box if p is reasonably large to justify another loop through it
	if(p.size() > 10){
		double x_min = p[0][0], x_max = x_min, y_max = p[0][1], y_min = y_max;
		for( const auto& p_i : p){
			if(p_i[0] > x_max){ x_max = p_i[0]; }
			else if(p_i[0] < x_min){ x_min = p_i[0]; }
			if(p_i[1] > y_max){ y_max = p_i[1]; }
			else if(p_i[1] < y_min){ y_min = p_i[1]; }
		}
		if(x[0] < x_min || x[0] > x_max || x[1] < y_min || x[1 > y_min]){
			return false;
		}
	}

	//If point is within bounding box, calculate edge crossings with a horizontal ray from x to the right
	bool inside = false;
	for ( std::size_t i=0, j=p.size()-1; i<p.size(); j=i++ ) {
		LineSegment<T> s ( {p[i], p[j]} );
		if ( internal::ray_intersects_segment( x, s ) ){
            inside = !inside;
        }
	}
	return inside;
}

//Calculates the convex hull of the given set of points
template<typename T>
geom2d::polygon<T> convex_hull(const std::vector<Vec2D<T>>& points){ //TODO: enable any stl container
	//Simple cases
	if ( points.size() < 3 ) {
		return points;
	}

	//Search max, min of y,x
	std::vector<Vec2D<T>> relevant_points;
	Vec2D<T> left, top, right, bottom;
	left = top = right = bottom = points.front();

	for( const auto& p : points){
        if(p[0] < left[0]){
            left = p;
        }
        if(p[0] > right[0]){
            right = p;
        }
        if(p[1] < bottom[1]){
            bottom = p;
        }
        if(p[1] > top[1]){
            top = p;
        }
	}

	//Exclude degenerate case to avoid divisions by 0
	if(geom::in_range(left[0], right[0])){
		return {bottom, top};
	}
	if(geom::in_range(top[1], bottom[1])){
		return {left, right};
	}

	//Sort points into upper/lower half and delete irrelevant ones
	std::vector<Vec2D<T>> relevant_points_upper;
	std::vector<Vec2D<T>> relevant_points_lower( { left } );
	double m = (right[1]-left[1]) / (right[0]-left[0]); ///y per x of middle line
	
	for(const auto& p : points){
		double y_cut = left[1] + m * (p[0] - left[0]);
		if( p[1] > y_cut ) ///p above  middle line? The points left and right do not fulfill this, they are part of the lower hull
		{
			if( !point_in_triangle<T, false>(p, left, top, right)){
				relevant_points_upper.push_back(p);
			}
		}
		else if( p[1] < y_cut ){
			if ( !point_in_triangle<T, false>( p, left, bottom, right ) ) {
				relevant_points_lower.push_back( p );
			}
		}
	}
	relevant_points_lower.push_back( right );

	//Sort relevant points by x-coordinate
    std::sort(std::begin(relevant_points_upper), std::end(relevant_points_upper),
                [](const Vec2D<T>& p1, const Vec2D<T>& p2) -> bool{
                    return (p1[0] == p2[0]) ? (p1[1] < p2[1]) : (p1[0] < p2[0]);
                }
	);

    std::sort(std::begin(relevant_points_lower), std::end(relevant_points_lower),
                [](const Vec2D<T>& p1, const Vec2D<T>& p2) -> bool{
                    return (p1[0] == p2[0]) ? (p1[1] > p2[1]) : (p1[0] > p2[0]);
                }
	);

	//Compute the convex hull
	std::vector<Vec2D<T>> hull;
	hull.reserve( relevant_points_upper.size()+relevant_points_lower.size() );
	
	//lower
	for(const auto& p : relevant_points_lower){
		while(hull.size() >= 2 ){
			auto p0 = hull[hull.size()-2];
			auto p1 = hull[hull.size()-1];
			if( cross( p1-p0, p-p0 ) > 0.0 ){
				hull.pop_back();
			}
			else{ break; }
		}
		hull.push_back(p);
	}

	//upper
	std::size_t n_lower = hull.size();
	for(const auto& p : relevant_points_upper){
		while(hull.size() >= n_lower +2){
			auto p0 = hull[hull.size()-2];
			auto p1 = hull[hull.size()-1];
			if( cross( p1-p0, p-p0 ) > 0.0 ){
				hull.pop_back();
			}
			else{ break; }
		}
		hull.push_back(p);
	}

	return hull;
}



/////////////////////
//Segment arithmetics
/////////////////////



//Given two line segments g1 = x1+s*(x2-x1), g2 = y1+t*(y2-y1), (s,t in [0,1]), the function returns the paramaeter pair s_0, t_0 (each in [0,1]) such that x1+s_0*(x2-x1) = y1+t_0*(y2-y1) is the intersection of g1 and g2, or (-1, -1) if they do not intersect.
template<typename T>
std::pair<double, double> segment_intersection( const LineSegment<T>& s1, const LineSegment<T>& s2, int allow_overlap = ALLOW_NO_OVERLAP ) {
	//Initialize some necessary vectors
	Vec2D<T> x_vec = s1.x2 - s1.x1;
	Vec2D<T> x1y1_vec = s2.x1 - s1.x1;
	Vec2D<T> x1y2_vec = s2.x2 - s1.x1;

	//Check if the segments are parallel
	double cross_prod_x_y1 = cross( x_vec, x1y1_vec );
	double cross_prod_x_y2 = cross( x_vec, x1y2_vec );

	if ( geom::sign( cross_prod_x_y1 ) == geom::sign( cross_prod_x_y2 ) ) {
		//y1 and y2 on same side of x1x2 (possibly on same line)
		if ( geom::sign( cross_prod_x_y1 ) == 0 ) {
			//On same line
			return internal::segments_on_line_intersection( s1, s2, allow_overlap);
		}
		else {
			//parallel but not on same line -> no intersection
			return{ -1.0, -1.0 };
		}
	}

	//If not, compute intersection as in https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
	Vec2D<T> y1y2_vec = s2.x2 - s2.x1;
	Vec2D<T> d = perpendicular( y1y2_vec );
	double scale = x_vec * d;
	assert( scale != 0.0 ); ///happens only if x y are parallel

	double s_0_y = cross_prod_x_y1 / scale;

	if ( s_0_y < 0.0 || s_0_y > 1.0 ) {
		return{-1, -1};
	}

	double s_0_x = x1y1_vec * d / scale;
	if ( s_0_x > 1.0 || s_0_x < 0.0 ) {
		return{-1, -1};
	}

	return{ s_0_x, s_0_y };
}



/////////////////////
//Polygon arithmetics
/////////////////////

//Computes the area of a non self-intersection polygon p as described in http://www.mathopenref.com/coordpolygonarea.html as application of https://en.wikipedia.org/wiki/Green%27s_theorem#Area_Calculation
template <typename T>
double area( const polygon<T>& p ) {
	double area = 0.0;
	for ( std::size_t i = 0; i <= p.size() - 2; i++ ) {
		area += cross( p[i], p[i + 1] );
	}
	area = std::abs( area ) / 2.0;
	return area;
}

//Computes the centroid of a polygon (Which is not necessarily inside the polygon
template <typename T>
Vec2D<double> centroid( const polygon<T>& p ) {
    double x=0.0, y=0.0;
    for(const auto& p_i : p){
        x+=p_i[0];
        y+=p_i[1];
    }
    return {x/p.size(), y/p.size()};
}


}//namespace geom2d
