#include <open_chisel/geometry/Raycast.h>
#include <iostream>

using namespace chisel;

int signum(int x)
{
    return x == 0 ? 0 : x < 0 ? -1 : 1;
}

float mod(float value, float modulus)
{
  return fmod(fmod(value, modulus) + modulus,  modulus);
}

float intbound(float s, float ds)
{
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0)
  {
    return intbound(-s, -ds);
  }
  else
  {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1-s)/ds;
  }
}

bool RayIntersectsAABB(const Vec3& start, const Vec3& end, const Point3& lb, const Point3& rt)
{
    Vec3 dir = (end - start).normalized();
    Vec3 dirfrac(1.0f / dir.x(), 1.0f / dir.y(), 1.0f / dir.z());

    // r.dir is unit direction std::vector of ray
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // start is origin of ray
    float t1 = (lb.x() - start.x()) * dirfrac.x();
    float t2 = (rt.x() - start.x()) * dirfrac.x();
    float t3 = (lb.y() - start.y()) * dirfrac.y();
    float t4 = (rt.y() - start.y()) * dirfrac.y();
    float t5 = (lb.z() - start.z()) * dirfrac.z();
    float t6 = (rt.z() - start.z()) * dirfrac.z();

    float tmin = fmax(fmax(fmin(t1, t2), fmin(t3, t4)), fmin(t5, t6));
    float tmax = fmin(fmin(fmax(t1, t2), fmax(t3, t4)), fmax(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0)
    {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        return false;
    }

    return true;
}

void Raycast(const Vec3& start, const Vec3& end, const Point3& min, const Point3& max, Point3List* output)
{
    assert(!!output);
    // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by John Amanatides and Andrew Woo, 1987
    // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
    // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
    // Extensions to the described algorithm:
    //   • Imposed a distance limit.
    //   • The face passed through to reach the current cube is provided to
    //     the callback.

    // The foundation of this algorithm is a parameterized representation of
    // the provided ray,
    //                    origin + t * direction,
    // except that t is not actually stored; rather, at any given point in the
    // traversal, we keep track of the *greater* t values which we would have
    // if we took a step sufficient to cross a cube boundary along that axis
    // (i.e. change the integer part of the coordinate) in the variables
    // tMaxX, tMaxY, and tMaxZ.

    // Cube containing origin point.
    int x = (int)std::floor(start.x());
    int y = (int)std::floor(start.y());
    int z = (int)std::floor(start.z());
    int endX = (int)std::floor(end.x());
    int endY = (int)std::floor(end.y());
    int endZ = (int)std::floor(end.z());
    Vec3 direction = (end - start);
    float maxDist = direction.squaredNorm();

    // Break out direction std::vector.
    float dx = endX - x;
    float dy = endY - y;
    float dz = endZ - z;

    // Direction to increment x,y,z when stepping.
    int stepX = (int)signum((int)dx);
    int stepY = (int)signum((int)dy);
    int stepZ = (int)signum((int)dz);

    // See description above. The initial values depend on the fractional
    // part of the origin.
    float tMaxX = intbound(start.x(), dx);
    float tMaxY = intbound(start.y(), dy);
    float tMaxZ = intbound(start.z(), dz);

    // The change in t when taking a step (always positive).
    float tDeltaX = ((float)stepX) / dx;
    float tDeltaY = ((float)stepY) / dy;
    float tDeltaZ = ((float)stepZ) / dz;

    // Avoids an infinite loop.
    if (stepX == 0 && stepY == 0 && stepZ == 0)
        return;

    float dist = 0;

    while (true)
    {

        if(x >= min.x() && x < max.x() &&
           y >= min.y() && y < max.y() &&
           z >= min.z() && z < max.z())
        {
            output->push_back(Point3(x, y, z));

            dist = (Vec3(x, y, z) - start).squaredNorm();

            if (dist > maxDist) return;
#if 0
            if (output->size() > 1500000)
            {
                std::cerr << "Error, too many racyast voxels." << std::endl;
                throw std::out_of_range("Too many raycast voxels");
            }
#endif
        }

        if(x == endX && y == endY && z == endZ) break;

        // tMaxX stores the t-value at which we cross a cube boundary along the
        // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
        // chooses the closest cube boundary. Only the first case of the four
        // has been commented in detail.
        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                // Update which cube we are now in.
                x += stepX;
                // Adjust tMaxX to the next X-oriented boundary crossing.
                tMaxX += tDeltaX;
            }
            else
            {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                y += stepY;
                tMaxY += tDeltaY;
            }
            else
            {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }
}

