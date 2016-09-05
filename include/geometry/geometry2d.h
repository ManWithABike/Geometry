// Copyright Ingo Proff 2016.
// https://github.com/CrikeeIP/HomotopyArea
// Distributed under the MIT Software License (X11 license)
// (See accompanying file LICENSE)



#pragma once


#include <vector>
#include <string>
#include <cassert> 

#include <fplus/fplus.h>

#include "geometry.h"



namespace geom2d {


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



	/////////////////
	//Vector operator
	/////////////////

	//Returns the cross product of the given vectors
	template <typename T>
	T cross( const Vec2D<T>& vec1, const Vec2D<T>& vec2 ) {
		return vec1[0] * vec2[1] - vec1[1] * vec2[0];
	}


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


	/////////////////////
	//Segment arithmetics
	/////////////////////

#define ALLOW_NO_OVERLAP 0	
#define ALLOW_TOUCHING 1
#define ALLOW_OVERLAP 2

namespace internal{
	template<typename T>
	fplus::maybe<std::pair<double, double>> segments_on_line_intersection( const LineSegment<T>& s1, const LineSegment<T>& s2, int allow_overlap ) {
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
			return{};
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
			std::cout << "Touching or overlapping parallel line segments:\n(" << fplus::show(x1) << "," << fplus::show(x2) << ")  <>  (" << fplus::show(y1) << "," << fplus::show(y2) << ")" << std::endl;
			assert(false); //Overlapping segments //TODO: Exception?
		}
	}
}//namespace internal

	//Given two line segments g1 = x1+s*(x2-x1), g2 = y1+t*(y2-y1), (s,t in [0,1]), the function returns the paramaeter pair s_0, t_0 (each in [0,1]) such that x1+s_0*(x2-x1) = y1+t_0*(y2-y1) is the intersection of g1 and g2, or (-1, -1) if they do not intersect.
	template<typename T>
	fplus::maybe<std::pair<double, double>> segment_intersection( const LineSegment<T>& s1, const LineSegment<T>& s2, int allow_overlap = ALLOW_NO_OVERLAP ) {
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
				//y1 and y2 on same side
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
			return{};
		}

		double s_0_x = x1y1_vec * d / scale;
		if ( s_0_x > 1.0 || s_0_x < 0.0 ) {
			return{};
		}

		return{ s_0_x, s_0_y };
	}



	/////////////////////
	//Polygon arithmetics
	/////////////////////
	

	typedef std::vector<Vec2D<double>> polygon;


	//Computes the area of a non self-intersection polygon p as described in http://www.mathopenref.com/coordpolygonarea.html as application of https://en.wikipedia.org/wiki/Green%27s_theorem#Area_Calculation
	inline double area( const polygon& p ) {
		double area = 0.0;
		for ( std::size_t i = 0; i <= p.size() - 2; i++ ) {
			area += cross( p[i], p[i + 1] );
		}
		area = std::abs( area ) / 2.0;
		return area;
	}

	//Computes the centroid of a polygon (Which is not necessarily inside the polygon
	inline Vec2D<double> centroid( const polygon& p ) {
		return fplus::sum( p ) / static_cast<double> (p.size());
	}



}//namespace geom2d
