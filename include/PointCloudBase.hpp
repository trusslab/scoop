#ifndef POINT_CLOUD_BASE_HPP
#define POINT_CLOUD_BASE_HPP

#include "CompilationFlags.hpp"

#include <boost/bimap.hpp>

// Map between Point Cloud and Depth Frame (Index and Pair of X, Y coordinates)
typedef boost::bimap<int, std::pair<int, int>> PCLDepthFrameBiMapHelper;
class PCLDepthFrameBiMap
{
public:
    PCLDepthFrameBiMapHelper bi_map;

    PCLDepthFrameBiMap();
    ~PCLDepthFrameBiMap();

    void insert(const int &index, const std::pair<int, int> &coordinate);
    void insert(const int &index, const int &x, const int &y);
    bool contains(const int &index) const;
    bool contains(const std::pair<int, int> &pair) const;
    int get_index(const std::pair<int, int> &pair) const;
    std::pair<int, int> get_coordinates(const int &index) const;
    void clear();
};

#endif // POINT_CLOUD_BASE_HPP