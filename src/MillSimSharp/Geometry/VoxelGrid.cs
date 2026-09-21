using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Sparse Voxel Octree Node
    /// </summary>
    public class SVONode
    {
        /// <summary>
        /// Children nodes (8 for octree)
        /// </summary>
        public SVONode[] children;
        /// <summary>
        /// Is this node a leaf node?
        /// </summary>
        public bool isLeaf;
        /// <summary>
        /// Value at this node (true = empty, false = material)
        /// </summary>
        public bool value;

        /// <summary>
        /// Constructor for SVONode.
        /// </summary>
        public SVONode()
        {
            children = new SVONode[8];
            isLeaf = false;
            value = false;
        }

        /// <summary>
        /// Gets the value at the specified voxel coordinates.
        /// </summary>
        public bool Get(int x, int y, int z, int level, int maxLevel)
        {
            if (isLeaf)
                return value; // true if empty

            if (level >= maxLevel)
                return false; // not empty

            int childIndex = ((x >> (maxLevel - level - 1)) & 1) |
                             (((y >> (maxLevel - level - 1)) & 1) << 1) |
                             (((z >> (maxLevel - level - 1)) & 1) << 2);

            if (children[childIndex] == null)
                return false; // not empty

            return children[childIndex].Get(x, y, z, level + 1, maxLevel);
        }

        /// <summary>
        /// Sets the value at the specified voxel coordinates.
        /// </summary>
        public void Set(int x, int y, int z, bool val, int level, int maxLevel)
        {
            if (level >= maxLevel)
            {
                isLeaf = true;
                value = val; // true for empty
                return;
            }

            int childIndex = ((x >> (maxLevel - level - 1)) & 1) |
                             (((y >> (maxLevel - level - 1)) & 1) << 1) |
                             (((z >> (maxLevel - level - 1)) & 1) << 2);

            if (children[childIndex] == null)
                children[childIndex] = new SVONode();

            children[childIndex].Set(x, y, z, val, level + 1, maxLevel);
        }

        /// <summary>
        /// Counts the number of empty voxels in this subtree up to the specified max level.
        /// </summary>
        public int CountEmpty(int level, int maxLevel)
        {
            if (isLeaf)
                return value ? 1 : 0;

            if (level >= maxLevel)
                return 0;

            int count = 0;
            for (int i = 0; i < 8; i++)
            {
                if (children[i] != null)
                    count += children[i].CountEmpty(level + 1, maxLevel);
            }
            return count;
        }
    }

    /// <summary>
    /// Represents a 3D voxel grid for material simulation using Sparse Voxel Octree.
    /// </summary>
    public class VoxelGrid
    {
        /// <summary>
        /// Event invoked when voxels are changed. Provides min and max indices of the region that changed.
        /// </summary>
        public event Action<int, int, int, int, int, int>? VoxelsChanged;
        private SVONode? _root;
        private readonly int _sizeX;
        private readonly int _sizeY;
        private readonly int _sizeZ;
        private readonly float _resolution;
        private readonly BoundingBox _bounds;
        private readonly int _maxLevel;
        private readonly int _totalVoxels;

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
        /// Gets or sets whether large removals may collect candidate voxels in parallel.
        /// The sparse voxel octree is always committed on a single thread, so the resulting voxel
        /// state is deterministic regardless of this setting.
        /// </summary>
        public bool UseParallelRemoval { get; set; } = true;

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

            // Calculate max level for SVO
            int maxDim = Math.Max(_sizeX, Math.Max(_sizeY, _sizeZ));
            _maxLevel = (int)Math.Ceiling(Math.Log(maxDim, 2));

            _totalVoxels = _sizeX * _sizeY * _sizeZ;

            // Root is null initially, meaning all voxels are material (true)
            _root = null;
        }

        /// <summary>
        /// Converts world coordinates to voxel indices (floor semantics).
        /// </summary>
        private (int x, int y, int z) WorldToVoxel(Vector3 worldPos)
        {
            Vector3 localPos = worldPos - _bounds.Min;
            return (
                (int)MathF.Floor(localPos.X / _resolution),
                (int)MathF.Floor(localPos.Y / _resolution),
                (int)MathF.Floor(localPos.Z / _resolution)
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

            if (_root == null)
                return true; // default material

            return !_root.Get(x, y, z, 0, _maxLevel); // Get returns true if empty, so ! for material
        }

        /// <summary>
        /// Sets the material state of a voxel at the specified indices.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="isMaterial"></param>
        public void SetVoxel(int x, int y, int z, bool isMaterial)
        {
            if (!IsValidIndex(x, y, z))
                return;

            if (_root == null)
                _root = new SVONode();
            _root.Set(x, y, z, !isMaterial, 0, _maxLevel); // !isMaterial: true for empty, false for material
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

        // Batch edit state: while editing, changes are accumulated and reported once on EndEdit.
        private int _editDepth;
        private bool _editHasChanges;
        private int _editMinX = int.MaxValue, _editMinY = int.MaxValue, _editMinZ = int.MaxValue;
        private int _editMaxX = int.MinValue, _editMaxY = int.MinValue, _editMaxZ = int.MinValue;

        /// <summary>
        /// Begins a batch edit. While active, <see cref="VoxelsChanged"/> is not raised for each
        /// operation; the aggregated changed bounds are reported once by <see cref="EndEdit"/>.
        /// </summary>
        public void BeginEdit()
        {
            _editDepth++;
        }

        /// <summary>
        /// Ends a batch edit and raises <see cref="VoxelsChanged"/> once with the aggregated
        /// changed bounds (if anything actually changed).
        /// </summary>
        /// <exception cref="InvalidOperationException">Thrown when called without a matching BeginEdit.</exception>
        public void EndEdit()
        {
            if (_editDepth == 0)
                throw new InvalidOperationException("EndEdit called without a matching BeginEdit.");

            _editDepth--;
            if (_editDepth > 0 || !_editHasChanges) return;

            int minX = _editMinX, minY = _editMinY, minZ = _editMinZ;
            int maxX = _editMaxX, maxY = _editMaxY, maxZ = _editMaxZ;

            _editHasChanges = false;
            _editMinX = int.MaxValue; _editMinY = int.MaxValue; _editMinZ = int.MaxValue;
            _editMaxX = int.MinValue; _editMaxY = int.MinValue; _editMaxZ = int.MinValue;

            VoxelsChanged?.Invoke(minX, minY, minZ, maxX, maxY, maxZ);
        }

        /// <summary>
        /// Reports a changed region, either immediately or accumulated into the current batch edit.
        /// </summary>
        private void ReportChange(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            if (_editDepth > 0)
            {
                _editHasChanges = true;
                if (minX < _editMinX) _editMinX = minX;
                if (minY < _editMinY) _editMinY = minY;
                if (minZ < _editMinZ) _editMinZ = minZ;
                if (maxX > _editMaxX) _editMaxX = maxX;
                if (maxY > _editMaxY) _editMaxY = maxY;
                if (maxZ > _editMaxZ) _editMaxZ = maxZ;
                return;
            }

            VoxelsChanged?.Invoke(minX, minY, minZ, maxX, maxY, maxZ);
        }

        private const int ParallelRemovalThreshold = 1000;
        private const int MaxParallelRemovalVolume = 4_000_000;

        /// <summary>
        /// Removes voxels in the region sequentially and reports the exact changed bounds.
        /// </summary>
        private void RemoveSequentially(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, Func<int, int, int, bool> shouldRemove)
        {
            int changedMinX = int.MaxValue, changedMinY = int.MaxValue, changedMinZ = int.MaxValue;
            int changedMaxX = int.MinValue, changedMaxY = int.MinValue, changedMaxZ = int.MinValue;

            for (int z = minZ; z <= maxZ; z++)
            for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                if (!shouldRemove(x, y, z) || !GetVoxel(x, y, z)) continue;

                SetVoxel(x, y, z, false);
                if (x < changedMinX) changedMinX = x;
                if (y < changedMinY) changedMinY = y;
                if (z < changedMinZ) changedMinZ = z;
                if (x > changedMaxX) changedMaxX = x;
                if (y > changedMaxY) changedMaxY = y;
                if (z > changedMaxZ) changedMaxZ = z;
            }

            if (changedMinX <= changedMaxX)
            {
                ReportChange(changedMinX, changedMinY, changedMinZ, changedMaxX, changedMaxY, changedMaxZ);
            }
        }

        /// <summary>
        /// Collects candidate voxels in parallel (read-only queries) and commits them to the
        /// sparse voxel octree on a single thread. This keeps results deterministic.
        /// </summary>
        private void RemoveInParallel(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, Func<int, int, int, bool> shouldRemove)
        {
            var removals = new List<(int x, int y, int z)>();
            var sync = new object();

            Parallel.For(minZ, maxZ + 1,
                () => new List<(int x, int y, int z)>(),
                (z, _, local) =>
                {
                    for (int y = minY; y <= maxY; y++)
                    for (int x = minX; x <= maxX; x++)
                    {
                        if (shouldRemove(x, y, z) && GetVoxel(x, y, z))
                        {
                            local.Add((x, y, z));
                        }
                    }
                    return local;
                },
                local =>
                {
                    lock (sync) removals.AddRange(local);
                });

            int changedMinX = int.MaxValue, changedMinY = int.MaxValue, changedMinZ = int.MaxValue;
            int changedMaxX = int.MinValue, changedMaxY = int.MinValue, changedMaxZ = int.MinValue;

            foreach (var (x, y, z) in removals)
            {
                SetVoxel(x, y, z, false);
                if (x < changedMinX) changedMinX = x;
                if (y < changedMinY) changedMinY = y;
                if (z < changedMinZ) changedMinZ = z;
                if (x > changedMaxX) changedMaxX = x;
                if (y > changedMaxY) changedMaxY = y;
                if (z > changedMaxZ) changedMaxZ = z;
            }

            if (changedMinX <= changedMaxX)
            {
                ReportChange(changedMinX, changedMinY, changedMinZ, changedMaxX, changedMaxY, changedMaxZ);
            }
        }

        /// <summary>
        /// Removes voxels in the specified index region for which <paramref name="shouldRemove"/>
        /// returns true. The region is clamped to the grid. Large regions use the deterministic
        /// two-phase parallel path.
        /// </summary>
        internal void RemoveVoxelsInRegion(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, Func<int, int, int, bool> shouldRemove)
        {
            minX = Math.Max(0, minX);
            minY = Math.Max(0, minY);
            minZ = Math.Max(0, minZ);
            maxX = Math.Min(_sizeX - 1, maxX);
            maxY = Math.Min(_sizeY - 1, maxY);
            maxZ = Math.Min(_sizeZ - 1, maxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            int volumeSize = (maxZ - minZ + 1) * (maxY - minY + 1) * (maxX - minX + 1);
            if (UseParallelRemoval && volumeSize > ParallelRemovalThreshold && volumeSize <= MaxParallelRemovalVolume)
            {
                RemoveInParallel(minX, minY, minZ, maxX, maxY, maxZ, shouldRemove);
            }
            else
            {
                RemoveSequentially(minX, minY, minZ, maxX, maxY, maxZ, shouldRemove);
            }
        }

        /// <summary>
        /// Removes voxels whose center lies inside the tool solid described by
        /// <paramref name="signedDistance"/> (negative = inside) within <paramref name="worldBounds"/>.
        /// </summary>
        internal void RemoveVoxelsInRegion(BoundingBox worldBounds, Func<Vector3, float> signedDistance)
        {
            if (worldBounds == null) throw new ArgumentNullException(nameof(worldBounds));
            if (signedDistance == null) throw new ArgumentNullException(nameof(signedDistance));

            var (minX, minY, minZ) = WorldToVoxel(worldBounds.Min);
            var (maxX, maxY, maxZ) = WorldToVoxel(worldBounds.Max);

            RemoveVoxelsInRegion(minX, minY, minZ, maxX, maxY, maxZ,
                (x, y, z) => signedDistance(VoxelToWorld(x, y, z)) < 0f);
        }

        /// <summary>
        /// Removes all voxels within a sphere (sets them to empty).
        /// <para>
        /// Large removals collect candidate voxels in parallel but commit to the sparse voxel
        /// octree on a single thread, so results are deterministic.
        /// </para>
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
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            float radiusSquared = radius * radius;

            bool ShouldRemove(int x, int y, int z)
            {
                // Early rejection: skip Y slice if too far from center
                float yDist = Math.Abs(VoxelToWorld(0, y, 0).Y - center.Y);
                if (yDist > radius) return false;
                return Vector3.DistanceSquared(VoxelToWorld(x, y, z), center) <= radiusSquared;
            }

            RemoveVoxelsInRegion(minX, minY, minZ, maxX, maxY, maxZ, ShouldRemove);
        }

        /// <summary>
        /// Removes all voxels within a cylinder (sets them to empty).
        /// <para>
        /// Large removals collect candidate voxels in parallel but commit to the sparse voxel
        /// octree on a single thread, so results are deterministic.
        /// </para>
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
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            float radiusSquared = radius * radius;

            bool ShouldRemove(int x, int y, int z)
            {
                Vector3 voxelCenter = VoxelToWorld(x, y, z);

                // Calculate distance from voxel to cylinder axis
                Vector3 toVoxel = voxelCenter - start;
                float projectionLength = Vector3.Dot(toVoxel, axisDir);

                // Check if projection is within cylinder length with tolerance
                if (projectionLength >= -1e-5f && projectionLength <= length + 1e-5f)
                {
                    Vector3 closestPoint = start + axisDir * projectionLength;
                    return Vector3.DistanceSquared(voxelCenter, closestPoint) <= radiusSquared;
                }

                if (!flatEnds)
                {
                    // Check distance to end caps (spheres)
                    float distToStart = Vector3.DistanceSquared(voxelCenter, start);
                    float distToEnd = Vector3.DistanceSquared(voxelCenter, end);
                    return distToStart <= radiusSquared || distToEnd <= radiusSquared;
                }

                return false;
            }

            RemoveVoxelsInRegion(minX, minY, minZ, maxX, maxY, maxZ, ShouldRemove);
        }

        /// <summary>
        /// Clears all voxels (resets to all material).
        /// </summary>
        public void Clear()
        {
            _root = null;
        }

        /// <summary>
        /// Returns a dense 3D array of voxel material states for fast access.
        /// </summary>
        public bool[][][] ToDenseArray()
        {
            bool[][][] dense = new bool[_sizeX][][];
            for (int x = 0; x < _sizeX; x++)
            {
                dense[x] = new bool[_sizeY][];
                for (int y = 0; y < _sizeY; y++)
                {
                    dense[x][y] = new bool[_sizeZ];
                    for (int z = 0; z < _sizeZ; z++)
                    {
                        dense[x][y][z] = GetVoxel(x, y, z);
                    }
                }
            }
            return dense;
        }

        /// <summary>
        /// Counts the number of voxels containing material.
        /// </summary>
        public int CountMaterialVoxels()
        {
            return _totalVoxels - (_root?.CountEmpty(0, _maxLevel) ?? 0);
        }

        /// <summary>
        /// Get list of occupied voxel coordinates.
        /// </summary>
        public List<(int x, int y, int z)> GetOccupiedVoxels()
        {
            var list = new List<(int x, int y, int z)>();
            if (_root == null)
            {
                // All voxels are occupied
                for (int z = 0; z < _sizeZ; z++)
                for (int y = 0; y < _sizeY; y++)
                for (int x = 0; x < _sizeX; x++)
                {
                    list.Add((x, y, z));
                }
            }
            else
            {
                TraverseOccupied(_root, 0, 0, 0, 0, _maxLevel, list);
            }
            return list;
        }

        private void TraverseOccupied(SVONode? node, int x, int y, int z, int level, int maxLevel, List<(int, int, int)> list)
        {
            if (level >= maxLevel)
            {
                // Leaf level, add the voxel if occupied
                if (node == null || !node.value)
                {
                    list.Add((x, y, z));
                }
                return;
            }
            int childSize = 1 << (maxLevel - level - 1);
            for (int i = 0; i < 8; i++)
            {
                int cx = x + ((i & 1) != 0 ? childSize : 0);
                int cy = y + ((i & 2) != 0 ? childSize : 0);
                int cz = z + ((i & 4) != 0 ? childSize : 0);
                SVONode child = node?.children[i];
                TraverseOccupied(child, cx, cy, cz, level + 1, maxLevel, list);
            }
        }

        /// <summary>
        /// Convert this VoxelGrid into a Mesh using the marching cubes algorithm.
        /// </summary>
        /// <returns>A Mesh representing the surface of the material in the voxel grid.</returns>
        public Mesh ToMesh()
        {
            return MeshConverter.ConvertToMesh(this);
        }
    }
}
