#include "limelib/types.hpp"

namespace limelib
{
    real_t getRandomReal_t(real_t lower, real_t upper)
    {
        static std::random_device rd;  // Static to avoid recreating for each call
        static std::mt19937 gen(rd()); // Static to maintain state between calls
        std::uniform_real_distribution<real_t> dist(lower, upper);
        return dist(gen);
    }
}