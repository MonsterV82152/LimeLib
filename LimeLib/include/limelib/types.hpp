#pragma once

#ifndef LIMELIB_TYPES_HPP
#define LIMELIB_TYPES_HPP
#include <vector>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <random>
#include <memory>
#include <iostream>
#include <type_traits>

// #define M_PI 3.1415926535
#define MM_TO_IN 0.0393701

namespace limelib
{
    using real_t = float;                               // float for memory-constrained systems
    real_t getRandomReal_t(real_t lower, real_t upper); // Declaration only

    // Automatic conversion function from int to real_t
    inline real_t to_real_t(int value)
    {
        return static_cast<real_t>(value);
    }

    // Automatic conversion function from double to real_t (for precision control)
    inline real_t to_real_t(double value)
    {
        return static_cast<real_t>(value);
    }

    // Template function for automatic conversion to real_t from any numeric type
    template <typename T>
    inline typename std::enable_if<std::is_arithmetic<T>::value, real_t>::type
    to_real_t(T value)
    {
        return static_cast<real_t>(value);
    }

// Macro for easy conversion to real_t
#define REAL_T(x) static_cast<limelib::real_t>(x)

    // Helper function to create shared_ptr with automatic real_t conversion
    template <typename T, typename... Args>
    inline std::shared_ptr<T> make_object(Args &&...args)
    {
        return std::make_shared<T>(to_real_t(args)...);
    }
}

#endif