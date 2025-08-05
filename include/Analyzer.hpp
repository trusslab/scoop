#ifndef ANALYZER_HPP
#define ANALYZER_HPP

// Flags
#include "CompilationFlags.hpp"

// Parameters set manually
#define DISPLAY_WINDOW_HEIGHT 720   // Height of the display window (width will be calculated based on aspect ratio)

// struct representing a 2D point and its threshold (negative threshold means no threshold)
// This is used along with function: find_surface_with_seed_point_grayscale()
typedef struct {
    short x;
    short y;
    float threshold;
} SeedPoint;

#endif // ANALYZER_HPP
