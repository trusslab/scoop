#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>

// https://stackoverflow.com/questions/1001307/detecting-endianness-programmatically-in-a-c-program
bool is_current_system_big_endian(void);

// https://stackoverflow.com/questions/2782725/converting-float-values-from-big-endian-to-little-endian
// Function to swap endianness
template <typename T>
T bswap(T val) 
{
    T retVal;
    char *pVal = (char*) &val;
    char *pRetVal = (char*)&retVal;
    int size = sizeof(T);
    for(int i=0; i<size; i++) {
        pRetVal[size-1-i] = pVal[i];
    }

    return retVal;
}

// https://www.geeksforgeeks.org/bit-manipulation-swap-endianness-of-a-number/
// Function to swap a value from 
// big Endian to little Endian and 
// vice versa. 
int swap_endians(int value);

template <class T>
double compute_median(std::vector<T> v)
{
    // if vector isn't empty
    if (!v.empty())
    {
        double median;
        size_t L = v.size(); // store the size
        
        // sort the vector
        std::sort(v.begin(), v.end());
        
        // if the length is even
        if (L  % 2 == 0)
        {
            // take the average of the middle two elements
            median = ((double)(v[L / 2 - 1] + v[L / 2])) / 2.0;
        }
        else // if the length is odd
        {
            // take the middle element
            median = (double) v[(L-1) / 2];
        }
        
        // return the median
        return median;
    }
    else // vector is empty
    {
        throw std::invalid_argument( "Received empty vector when calculating median" );
    }
}

#endif // UTILS_HPP