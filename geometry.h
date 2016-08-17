// Copyright Ingo Proff 2016.
// https://github.com/CrikeeIP/Geometry
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)


#pragma once



#include <vector>
#include <string>
#include <cassert>

#include <fplus/fplus.h>


namespace geom {



	///////////
	//constants
	///////////

	const double pi = 4.0 * std::atan( 1.0 );

	const double e = std::exp( 1.0 );



	///////////////
	//Vector struct
	///////////////

	template <typename T, std::size_t N>
	struct Vec {
		Vec( const std::vector<T>& coords ) : coordinates(coords){
			assert( coords.size() == N );
		};
		Vec( const Vec<T, N>& vec ) : coordinates(vec.coordinates) {};
		Vec( const T& x ) : coordinates( N, x ) {};
		Vec( std::initializer_list<T> coord_list ) : coordinates( coord_list ) {
			assert( coordinates.size() == N );
		}

		std::size_t dimension() {
			return coordinates.size;
		}

		T operator[] ( std::size_t n ) const {
			return coordinates[n];
		}

	private:
		std::vector<T> coordinates;
	};

	//make_vec because class template arguments (in their class constructors) cannot be deduced
	template <std::size_t N, typename T, typename... Args>
	geom::Vec<T,N> make_vec( T x, Args&&... xs )
	{
		std::size_t size = 1 + sizeof( xs );
		assert( N == size );
		return{ x, xs... };
	}


	//recursive pythagoras template
	template <typename T, size_t N, size_t I>
	struct compute_pythagoras
	{
		//Computes square_sum(v1-v2)
		static inline T apply( const Vec<T,N>& v1, const Vec<T,N>& v2 )
		{
			T const c1 = v1[I];
			T const c2 = v2[I];
			T const d = c1 - c2;
			return d * d + compute_pythagoras<T, N, I-1>::apply( v1, v2 );
		}
	};

	template <typename T, size_t N>
	struct compute_pythagoras<T, N, 0>
	{
		static inline T apply( const Vec<T,N>& v1, const Vec<T,N>& v2 )
		{
			T const c1 = v1[0];
			T const c2 = v2[0];
			T const d = c1 - c2;
			return d * d;
		}
	};



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
		std::vector<T> result();
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
		std::vector<T> result();
		result.reserve( N );
		for ( std::size_t i = 0; i < N; i++ ) {
			result.push_back( vec[i] / divisor );
		}
		return{ result };
	}

	template <typename T, std::size_t N>
	std::string print( const Vec<T,N>& vec ) {
		return fplus::show_cont_with_frame( ",", "(", ")", vec.coordinates );
	}



	///////////////////
	//Vector Arithmetic
	///////////////////

	template<typename T, std::size_t N>
	double norm( const Vec<T, N>& vec ) {
		const auto null_vec = Vec<T, N>( 0 );
		return std::sqrt( compute_pythagoras<T, N, N - 1>::apply( vec, null_vec ) ); //TODO: Rather without pythagoras (unneccessary vec - 0) and instead
	}

	template<typename T, std::size_t N>
	Vec<T,N> normalize( const Vec<T,N>& vec ) {
		double length = norm( vec );
		return vec / length;
	}

	template<typename T, std::size_t N>
	double square_dist( const Vec<T,N>& p1, const Vec<T,N>& p2 ) {
		return compute_pythagoras<T, N, N-1>::apply( p1, p2 );
	}

	template<typename T, std::size_t N>
	double dist( const Vec<T,N>& p1, const Vec<T,N>& p2 ) {
		return std::sqrt( square_dist( p1, p2 ) );
	}



}//namespace geom
