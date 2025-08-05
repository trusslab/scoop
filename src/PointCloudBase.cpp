#include "PointCloudBase.hpp"

// Map between Point Cloud and Depth Frame (Index and Pair of X, Y coordinates)
PCLDepthFrameBiMap::PCLDepthFrameBiMap() {}
PCLDepthFrameBiMap::~PCLDepthFrameBiMap() {}

void PCLDepthFrameBiMap::insert(const int &index, const std::pair<int, int> &coordinate)
{
    bi_map.insert(PCLDepthFrameBiMapHelper::value_type(index, coordinate));
}

void PCLDepthFrameBiMap::insert(const int &index, const int &x, const int &y)
{
    insert(index, std::make_pair(x, y));
}

bool PCLDepthFrameBiMap::contains(const int &index) const
{
    return bi_map.left.find(index) != bi_map.left.end();
}

bool PCLDepthFrameBiMap::contains(const std::pair<int, int> &pair) const
{
    return bi_map.right.find(pair) != bi_map.right.end();
}

int PCLDepthFrameBiMap::get_index(const std::pair<int, int> &pair) const
{
    return bi_map.right.at(pair);
}

std::pair<int, int> PCLDepthFrameBiMap::get_coordinates(const int &index) const
{
    return bi_map.left.at(index);
}

void PCLDepthFrameBiMap::clear()
{
    bi_map.clear();
}