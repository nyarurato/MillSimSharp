using System;
using System.Collections;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Represents a 3D voxel grid for material simulation.
    /// </summary>
    public class VoxelGrid
    {
        private readonly BitArray _voxels;
        private readonly int _sizeX;
        private readonly int _sizeY;
        private readonly int _sizeZ;
        private readonly float _resolution;
        private readonly BoundingBox _bounds;

        /// <summary>
        /// Gets the resolution (voxel size) in millimeters.
        /// </summary>
        public float Resolution => _resolution;

        /// <summary>
        /// Gets the bounding box of the work area.
        /// </summary>
        public BoundingBox Bounds => _bounds;

        /// <summary>
        /// Gets the dimensions of the voxel grid.
        /// </summary>
        public (int X, int Y, int Z) Dimensions => (_sizeX, _sizeY, _sizeZ);

        /// <summary>
        /// Creates a new voxel grid with the specified work area and resolution.
        /// </summary>
        /// <param name="workArea">The bounding box defining the work area.</param>
        /// <param name="resolution">Voxel size in millimeters (default: 0.5mm).</param>
        public VoxelGrid(BoundingBox workArea, float resolution = 0.5f)
        {
            if (resolution <= 0)
                throw new ArgumentException("Resolution must be positive.", nameof(resolution));

            _bounds = workArea;
            _resolution = resolution;

            // Calculate grid dimensions
            Vector3 size = workArea.Size;
            _sizeX = (int)Math.Ceiling(size.X / resolution);
            _sizeY = (int)Math.Ceiling(size.Y / resolution);
            _sizeZ = (int)Math.Ceiling(size.Z / resolution);

            // Initialize all voxels as material (true)
            int totalVoxels = _sizeX * _sizeY * _sizeZ;
            _voxels = new BitArray(totalVoxels, true);
        }

        /// <summary>
        /// Converts world coordinates to voxel indices.
        /// </summary>
        private (int x, int y, int z) WorldToVoxel(Vector3 worldPos)
        {
            Vector3 localPos = worldPos - _bounds.Min;
            return (
                (int)(localPos.X / _resolution),
                (int)(localPos.Y / _resolution),
                (int)(localPos.Z / _resolution)
            );
        }

        /// <summary>
        /// Converts voxel indices to world coordinates (center of voxel).
        /// </summary>
        private Vector3 VoxelToWorld(int x, int y, int z)
        {
            return _bounds.Min + new Vector3(
                (x + 0.5f) * _resolution,
                (y + 0.5f) * _resolution,
                (z + 0.5f) * _resolution
            );
        }

        /// <summary>
        /// Checks if voxel indices are within bounds.
        /// </summary>
        private bool IsValidIndex(int x, int y, int z)
        {
            return x >= 0 && x < _sizeX &&
                   y >= 0 && y < _sizeY &&
                   z >= 0 && z < _sizeZ;
        }

        /// <summary>
        /// Converts 3D indices to 1D array index.
        /// </summary>
        private int GetIndex(int x, int y, int z)
        {
            return x + y * _sizeX + z * _sizeX * _sizeY;
        }

        /// <summary>
        /// Gets the material state of a voxel at the specified indices.
        /// </summary>
        /// <returns>True if the voxel contains material, false if empty.</returns>
        public bool GetVoxel(int x, int y, int z)
        {
            if (!IsValidIndex(x, y, z))
                return false;

            return _voxels[GetIndex(x, y, z)];
        }

        /// <summary>
        /// Sets the material state of a voxel at the specified indices.
        /// </summary>
        /// <param name="isMaterial">True for material, false for empty.</param>
        public void SetVoxel(int x, int y, int z, bool isMaterial)
        {
            if (!IsValidIndex(x, y, z))
                return;

            _voxels[GetIndex(x, y, z)] = isMaterial;
        }

        /// <summary>
        /// Gets the material state of a voxel at world coordinates.
        /// </summary>
        public bool GetVoxelAtWorld(Vector3 worldPos)
        {
            var (x, y, z) = WorldToVoxel(worldPos);
            return GetVoxel(x, y, z);
        }

        /// <summary>
        /// Sets the material state of a voxel at world coordinates.
        /// </summary>
        public void SetVoxelAtWorld(Vector3 worldPos, bool isMaterial)
        {
            var (x, y, z) = WorldToVoxel(worldPos);
            SetVoxel(x, y, z, isMaterial);
        }

        /// <summary>
        /// Removes all voxels within a sphere (sets them to empty).
        /// </summary>
        /// <param name="center">Center of the sphere in world coordinates.</param>
        /// <param name="radius">Radius of the sphere in millimeters.</param>
        public void RemoveVoxelsInSphere(Vector3 center, float radius)
        {
            // Calculate bounding box of the sphere in voxel space
            var (minX, minY, minZ) = WorldToVoxel(center - new Vector3(radius, radius, radius));
            var (maxX, maxY, maxZ) = WorldToVoxel(center + new Vector3(radius, radius, radius));

            // Clamp to grid bounds
            minX = Math.Max(0, minX);
            minY = Math.Max(0, minY);
            minZ = Math.Max(0, minZ);
            maxX = Math.Min(_sizeX - 1, maxX);
            maxY = Math.Min(_sizeY - 1, maxY);
            maxZ = Math.Min(_sizeZ - 1, maxZ);

            float radiusSquared = radius * radius;

            for (int z = minZ; z <= maxZ; z++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    for (int x = minX; x <= maxX; x++)
                    {
                        Vector3 voxelCenter = VoxelToWorld(x, y, z);
                        if (Vector3.DistanceSquared(voxelCenter, center) <= radiusSquared)
                        {
                            SetVoxel(x, y, z, false);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Removes all voxels within a cylinder (sets them to empty).
        /// </summary>
        /// <param name="start">Start point of the cylinder axis in world coordinates.</param>
        /// <param name="end">End point of the cylinder axis in world coordinates.</param>
        /// <param name="radius">Radius of the cylinder in millimeters.</param>
        /// <param name="flatEnds">If true, the cylinder has flat ends. If false, it has hemispherical ends (capsule).</param>
        public void RemoveVoxelsInCylinder(Vector3 start, Vector3 end, float radius, bool flatEnds = false)
        {
            Vector3 axis = end - start;
            float length = axis.Length();
            
            if (length < 1e-6f)
            {
                // Degenerate case: cylinder is a sphere
                RemoveVoxelsInSphere(start, radius);
                return;
            }

            Vector3 axisDir = Vector3.Normalize(axis);

            // Calculate bounding box of the cylinder
            Vector3 min = new Vector3(
                Math.Min(start.X, end.X) - radius,
                Math.Min(start.Y, end.Y) - radius,
                Math.Min(start.Z, end.Z) - radius
            );
            Vector3 max = new Vector3(
                Math.Max(start.X, end.X) + radius,
                Math.Max(start.Y, end.Y) + radius,
                Math.Max(start.Z, end.Z) + radius
            );

            var (minX, minY, minZ) = WorldToVoxel(min);
            var (maxX, maxY, maxZ) = WorldToVoxel(max);

            // Clamp to grid bounds
            minX = Math.Max(0, minX);
            minY = Math.Max(0, minY);
            minZ = Math.Max(0, minZ);
            maxX = Math.Min(_sizeX - 1, maxX);
            maxY = Math.Min(_sizeY - 1, maxY);
            maxZ = Math.Min(_sizeZ - 1, maxZ);

            float radiusSquared = radius * radius;

            for (int z = minZ; z <= maxZ; z++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    for (int x = minX; x <= maxX; x++)
                    {
                        Vector3 voxelCenter = VoxelToWorld(x, y, z);
                        
                        // Calculate distance from voxel to cylinder axis
                        Vector3 toVoxel = voxelCenter - start;
                        float projectionLength = Vector3.Dot(toVoxel, axisDir);
                        
                        // Check if projection is within cylinder length with tolerance
                        if (projectionLength >= -1e-5f && projectionLength <= length + 1e-5f)
                        {
                            Vector3 closestPoint = start + axisDir * projectionLength;
                            float distanceSquared = Vector3.DistanceSquared(voxelCenter, closestPoint);
                            
                            if (distanceSquared <= radiusSquared)
                            {
                                SetVoxel(x, y, z, false);
                            }
                        }
                        else if (!flatEnds)
                        {
                            // Check distance to end caps (spheres)
                            float distToStart = Vector3.DistanceSquared(voxelCenter, start);
                            float distToEnd = Vector3.DistanceSquared(voxelCenter, end);
                            
                            if (distToStart <= radiusSquared || distToEnd <= radiusSquared)
                            {
                                SetVoxel(x, y, z, false);
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Fills the entire grid with material.
        /// </summary>
        public void Clear()
        {
            _voxels.SetAll(true);
        }

        /// <summary>
        /// Counts the number of voxels containing material.
        /// </summary>
        public int CountMaterialVoxels()
        {
            int count = 0;
            for (int i = 0; i < _voxels.Length; i++)
            {
                if (_voxels[i])
                    count++;
            }
            return count;
        }
    }
}
