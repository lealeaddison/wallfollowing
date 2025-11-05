#include <wall_follower/common/utils.h>
#include <vector>
#include <limits>

std::vector<float> crossProduct(const std::vector<float>& v1, const std::vector<float>& v2)
{
    std::vector<float> res(3);

    // v1 * v2 = [y1​z2​−z1​y2​,z1​x2​−x1​z2​,x1​y2​−y1​x2​]
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
    
    /**
     * TODO: (P1.2) Take the cross product between v1 and v2 and store the
     * result in res. v1, v2, and res should be vectors with 3 elements, which
     * correspond to the vector components [x, y, z].
     **/
    return res;
}

int findMinDist(const std::vector<float>& ranges)
{
    int min_idx = -1; //Default if no valid ray is found
    float min_val = std::numeric_limits<float>::infinity();

    /**
     * TODO: (P1.2) Return the index of the shortest ray in the Lidar scan.
     * For example, if the shortest ray is the third one, at index 2, return 2.
     *
     * HINT: The length of each ray is stored in the vector ranges.
     *
     * HINT: Do not take into account any rays which have 0 distance. This means
     * that the ray returned by the sensor is invalid. Invalid rays will have
     * default range 0, which will always be the minimum if you forget to check
     * for validity.
     **/
    for (int i = 0; i < (int)ranges.size(); ++i) {
        float r = ranges[i];
        if (r > 0.0f && r < min_val) {
            min_val = r; // Remember the new minimum distance
            min_idx = i; ?? Remember its index
        }
    }

    return min_idx; // Index of the closest valid ray
}