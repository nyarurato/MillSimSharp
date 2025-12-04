using System;
using System.Numerics;
namespace MillSimSharp.Geometry
{
    internal static class SDFUtils
    {
        // Determine if a voxel is a surface voxel (6-connected) given VoxelGrid methods
        public static bool IsSurfaceVoxel(VoxelGrid voxelGrid, int x, int y, int z, int sizeX, int sizeY, int sizeZ)
        {
            bool current = voxelGrid.GetVoxel(x, y, z);
            
            // Check 6-connected neighbors
            for (int i = 0; i < 6; i++)
            {
                int nx = x, ny = y, nz = z;
                switch (i)
                {
                    case 0: nx = x - 1; break;
                    case 1: nx = x + 1; break;
                    case 2: ny = y - 1; break;
                    case 3: ny = y + 1; break;
                    case 4: nz = z - 1; break;
                    case 5: nz = z + 1; break;
                }
                
                // Out of bounds treated as empty (air)
                if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY || nz < 0 || nz >= sizeZ)
                {
                    // Boundary voxels are always surface (both material and empty)
                    return true;
                }
                
                bool neighbor = voxelGrid.GetVoxel(nx, ny, nz);
                if (neighbor != current) return true;
            }
            return false;
        }

        // Compute an SDF distance at a given voxel index with a bounded search radius (in voxels)
        public static float ComputeDistanceAt(VoxelGrid voxelGrid, BoundingBox bounds, int x, int y, int z,
            float resolution, int searchRadius, int sizeX, int sizeY, int sizeZ, float narrowBand)
        {
            bool isMaterial = voxelGrid.GetVoxel(x, y, z);
            float minDistance = float.MaxValue;
            bool foundSurface = false;

            int minX = Math.Max(0, x - searchRadius);
            int maxX = Math.Min(sizeX - 1, x + searchRadius);
            int minY = Math.Max(0, y - searchRadius);
            int maxY = Math.Min(sizeY - 1, y + searchRadius);
            int minZ = Math.Max(0, z - searchRadius);
            int maxZ = Math.Min(sizeZ - 1, z + searchRadius);

            for (int sz = minZ; sz <= maxZ; sz++)
            {
                for (int sy = minY; sy <= maxY; sy++)
                {
                    for (int sx = minX; sx <= maxX; sx++)
                    {
                        if (IsSurfaceVoxel(voxelGrid, sx, sy, sz, sizeX, sizeY, sizeZ))
                        {
                            int dx = x - sx;
                            int dy = y - sy;
                            int dz = z - sz;
                            float dist = MathF.Sqrt(dx * dx + dy * dy + dz * dz);
                            if (dist < minDistance)
                            {
                                minDistance = dist;
                                foundSurface = true;
                            }
                        }
                    }
                }
            }

            float worldDist = minDistance * resolution;
            if (worldDist > narrowBand) worldDist = narrowBand;
            // Standard SDF convention: negative inside (material), positive outside (empty)
            float signedDistance = isMaterial ? -worldDist : worldDist;
            if (!foundSurface)
            {
                signedDistance = isMaterial ? -narrowBand : narrowBand;
            }
            return signedDistance;
        }
    }
}
