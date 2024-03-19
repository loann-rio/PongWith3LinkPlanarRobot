#pragma once

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

template<class T>
constexpr T pi = T(3.1415926535897932385L);