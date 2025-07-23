#pragma once
#include <cmath>
inline double harmonic_upper_bound(int k) {
    if (k <= 1) return static_cast<double>(k);
    return std::log2(static_cast<double>(k)) + 1.0;
}