using System;
using System.Collections.Generic;
using System.Numerics;
namespace MillSimSharp.Geometry
{
    internal class OctreeNode
    {
        public int MinX, MinY, MinZ;
        public int Size; // cube size in voxels (power of two)
        public bool IsLeaf;
        public float Value; // signed distance for leaf
        public OctreeNode[] Children; // 8 children

        public OctreeNode(int minX, int minY, int minZ, int size)
        {
            MinX = minX; MinY = minY; MinZ = minZ; Size = size;
            IsLeaf = true;
            Value = 0.0f;
            Children = null;
        }
    }

    internal class OctreeSDF
    {
        private readonly VoxelGrid _voxelGrid;
        private readonly BoundingBox _bounds;
        private readonly float _resolution;
        private readonly int _sizeX, _sizeY, _sizeZ;
        private readonly float _narrowBand;
        private readonly OctreeNode _root;
        private readonly float[,,]? _precomputedSDF; // Pre-computed dense SDF grid (if using Fast Sweeping)
        private readonly List<(int x, int y, int z)> _surfaceVoxels;
        private readonly Dictionary<int, List<(int x, int y, int z)>> _surfaceVoxelGrid; // Spatial hash for fast lookup
        private readonly int _gridCellSize = 8; // Grid cells of 8x8x8 voxels
        private readonly bool _fastMode;
        private readonly System.Collections.Concurrent.ConcurrentDictionary<int, float> _sampleCache; // Cache for distance samples

        public OctreeSDF(VoxelGrid voxelGrid, float resolution, BoundingBox bounds, int sizeX, int sizeY, int sizeZ, float narrowBand, bool fastMode = false)
        {
            _voxelGrid = voxelGrid;
            _resolution = resolution;
            _bounds = bounds;
            _sizeX = sizeX; _sizeY = sizeY; _sizeZ = sizeZ;
            _narrowBand = narrowBand;
            _fastMode = fastMode;
            _sampleCache = new System.Collections.Concurrent.ConcurrentDictionary<int, float>();
            
            Console.WriteLine($"  Building octree SDF (narrowBand={narrowBand}, fastMode={fastMode})...");
            var sw = System.Diagnostics.Stopwatch.StartNew();
            
            // Use Fast Sweeping Algorithm to pre-compute dense SDF grid
            _precomputedSDF = FastSweepingSDF.ComputeSDF(voxelGrid, sizeX, sizeY, sizeZ, narrowBand);
            Console.WriteLine($"  Fast Sweeping SDF computed in {sw.ElapsedMilliseconds} ms");
            
            // Surface voxels not needed anymore - SDF is already computed
            _surfaceVoxels = new List<(int, int, int)>();
            _surfaceVoxelGrid = new Dictionary<int, List<(int x, int y, int z)>>();
            
            int maxDim = Math.Max(_sizeX, Math.Max(_sizeY, _sizeZ));
            int pow2 = 1; while (pow2 < maxDim) pow2 <<= 1;
            _root = new OctreeNode(0, 0, 0, pow2);
            
            sw.Restart();
            BuildNode(_root);
            Console.WriteLine($"  Octree built in {sw.ElapsedMilliseconds} ms");
        }
        
        private int GetGridKey(int x, int y, int z)
        {
            int gx = x / _gridCellSize;
            int gy = y / _gridCellSize;
            int gz = z / _gridCellSize;
            // Safe bit-packing hash (supports up to 1024x1024x1024 grid cells)
            return (gx & 0x3FF) | ((gy & 0x3FF) << 10) | ((gz & 0x3FF) << 20);
        }

        private Vector3 VoxelToWorld(int x, int y, int z)
        {
            return _bounds.Min + new Vector3(
                (x + 0.5f) * _resolution,
                (y + 0.5f) * _resolution,
                (z + 0.5f) * _resolution);
        }

        private float SampleDistanceAtVoxel(int x, int y, int z)
        {
            // With pre-computed SDF, just read from the grid - O(1) operation!
            if (_precomputedSDF != null)
            {
                // Out of bounds: return positive distance (empty space outside)
                if (x < 0 || x >= _sizeX || y < 0 || y >= _sizeY || z < 0 || z >= _sizeZ)
                {
                    return _narrowBand;
                }
                
                // Convert voxel-space distance to world-space distance
                return _precomputedSDF[x, y, z] * _resolution;
            }
            
            // Fallback to old method (shouldn't happen)
            return SampleDistanceAtVoxelLegacy(x, y, z);
        }
        
        private float SampleDistanceAtVoxelLegacy(int x, int y, int z)
        {
            // Check cache first
            int cacheKey = x + y * _sizeX + z * (_sizeX * _sizeY);
            if (_sampleCache.TryGetValue(cacheKey, out float cachedValue))
                return cachedValue;
            
            // Compute distance
            float result = SampleDistanceAtVoxelNoCache(x, y, z);
            
            // Store in cache
            _sampleCache[cacheKey] = result;
            return result;
        }
        
        private float SampleDistanceAtVoxelNoCache(int x, int y, int z)
        {
            // if outside the voxel grid bounds, return clamped negative narrow band
            if (x < 0 || x >= _sizeX || y < 0 || y >= _sizeY || z < 0 || z >= _sizeZ)
            {
                return -_narrowBand;
            }
            
            bool isMaterial = _voxelGrid.GetVoxel(x, y, z);
            
            // Fast mode: use reduced search radius
            int searchRadius = _fastMode 
                ? Math.Min(3, (int)Math.Ceiling(_narrowBand / _resolution))
                : Math.Max(1, (int)Math.Ceiling(_narrowBand / _resolution));
            
            int minDistSq = int.MaxValue; // Use int for voxel-space distance squared
            int radiusSq = searchRadius * searchRadius;
            
            // Calculate grid cell range to check
            int gx = x / _gridCellSize;
            int gy = y / _gridCellSize;
            int gz = z / _gridCellSize;
            int gridRadius = (searchRadius + _gridCellSize - 1) / _gridCellSize;
            
            // Iterate nearby grid cells
            for (int dz = -gridRadius; dz <= gridRadius; dz++)
            {
                int ngz = gz + dz;
                if (ngz < 0) continue;
                
                for (int dy = -gridRadius; dy <= gridRadius; dy++)
                {
                    int ngy = gy + dy;
                    if (ngy < 0) continue;
                    
                    for (int dx = -gridRadius; dx <= gridRadius; dx++)
                    {
                        int ngx = gx + dx;
                        if (ngx < 0) continue;
                        
                        int key = (ngx & 0x3FF) | ((ngy & 0x3FF) << 10) | ((ngz & 0x3FF) << 20);
                        if (!_surfaceVoxelGrid.TryGetValue(key, out var cellVoxels))
                            continue;
                        
                        foreach (var (sx, sy, sz) in cellVoxels)
                        {
                            int ddx = sx - x;
                            int ddy = sy - y;
                            int ddz = sz - z;
                            
                            // Manhattan distance early skip
                            if (Math.Abs(ddx) > searchRadius || Math.Abs(ddy) > searchRadius || Math.Abs(ddz) > searchRadius)
                                continue;
                            
                            int distSq = ddx * ddx + ddy * ddy + ddz * ddz;
                            
                            if (distSq < minDistSq)
                            {
                                minDistSq = distSq;
                                // Early exit if found exact surface point
                                if (minDistSq == 0)
                                    goto FoundSurface;
                            }
                        }
                    }
                }
            }
            
            FoundSurface:
            float worldDist;
            if (minDistSq == int.MaxValue)
            {
                worldDist = _narrowBand;
            }
            else
            {
                // Only sqrt once at the end
                worldDist = MathF.Sqrt(minDistSq) * _resolution;
                if (worldDist > _narrowBand) worldDist = _narrowBand;
            }
            
            // Standard SDF convention: negative inside (material), positive outside (empty)
            return isMaterial ? -worldDist : worldDist;
        }

        private void BuildNode(OctreeNode node)
        {
            // If node corresponds to a 1x1x1 region (single voxel) sample directly
            if (node.Size <= 1)
            {
                node.IsLeaf = true;
                node.Value = SampleDistanceAtVoxel(node.MinX, node.MinY, node.MinZ);
                return;
            }

            // Sample corners to decide if we need to subdivide
            // With pre-computed SDF, this is now very fast - just array lookups!
            int sx = node.MinX; int sy = node.MinY; int sz = node.MinZ;
            int s = node.Size - 1; // span
            var samples = new List<float>();
            samples.Add(SampleDistanceAtVoxel(sx, sy, sz));
            samples.Add(SampleDistanceAtVoxel(sx + s, sy, sz));
            samples.Add(SampleDistanceAtVoxel(sx, sy + s, sz));
            samples.Add(SampleDistanceAtVoxel(sx + s, sy + s, sz));
            samples.Add(SampleDistanceAtVoxel(sx, sy, sz + s));
            samples.Add(SampleDistanceAtVoxel(sx + s, sy, sz + s));
            samples.Add(SampleDistanceAtVoxel(sx, sy + s, sz + s));
            samples.Add(SampleDistanceAtVoxel(sx + s, sy + s, sz + s));
            // center
            samples.Add(SampleDistanceAtVoxel(sx + node.Size/2, sy + node.Size/2, sz + node.Size/2));

            // Check if we need to subdivide: if signs differ or variation is high
            bool allSameSign = true;
            bool firstPos = samples[0] > 0;
            float minVal = float.MaxValue, maxVal = float.MinValue;
            foreach (var v in samples)
            {
                if ((v > 0) != firstPos) allSameSign = false;
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
            }
            float diff = maxVal - minVal;
            
            // If all same sign and difference is small enough, make this a leaf
            if (allSameSign && diff < _resolution)
            {
                node.IsLeaf = true;
                node.Value = (minVal + maxVal) / 2.0f;
                return;
            }

            // Subdivide - this node spans the surface
            node.IsLeaf = false;
            node.Children = new OctreeNode[8];
            int half = node.Size / 2;
            for (int cz = 0; cz < 2; cz++)
            for (int cy = 0; cy < 2; cy++)
            for (int cx = 0; cx < 2; cx++)
            {
                int idx = cx | (cy << 1) | (cz << 2);
                int nx = node.MinX + cx * half;
                int ny = node.MinY + cy * half;
                int nz = node.MinZ + cz * half;
                var child = new OctreeNode(nx, ny, nz, half);
                node.Children[idx] = child;
                BuildNode(child);
            }
        }

        // Calculate minimum distance from node AABB to any surface voxel - OPTIMIZED with spatial hash
        private float GetMinDistanceToSurface(int minX, int minY, int minZ, int size)
        {
            float minDistSq = float.MaxValue;
            
            // More precise search radius calculation
            int voxelSearchRadius = (int)Math.Ceiling(_narrowBand / _resolution);
            int searchRadius = voxelSearchRadius + size;
            int gridMinX = Math.Max(0, (minX - searchRadius) / _gridCellSize);
            int gridMaxX = Math.Min((_sizeX - 1) / _gridCellSize, (minX + size + searchRadius - 1) / _gridCellSize);
            int gridMinY = Math.Max(0, (minY - searchRadius) / _gridCellSize);
            int gridMaxY = Math.Min((_sizeY - 1) / _gridCellSize, (minY + size + searchRadius - 1) / _gridCellSize);
            int gridMinZ = Math.Max(0, (minZ - searchRadius) / _gridCellSize);
            int gridMaxZ = Math.Min((_sizeZ - 1) / _gridCellSize, (minZ + size + searchRadius - 1) / _gridCellSize);
            
            // Threshold for early exit (in voxel space squared)
            float thresholdSq = 0.01f; // Very close to surface
            
            // Check only surface voxels in nearby grid cells
            for (int gz = gridMinZ; gz <= gridMaxZ; gz++)
            {
                for (int gy = gridMinY; gy <= gridMaxY; gy++)
                {
                    for (int gx = gridMinX; gx <= gridMaxX; gx++)
                    {
                        int key = (gx & 0x3FF) | ((gy & 0x3FF) << 10) | ((gz & 0x3FF) << 20);
                        if (!_surfaceVoxelGrid.TryGetValue(key, out var cellVoxels))
                            continue;
                        
                        foreach (var (sx, sy, sz) in cellVoxels)
                        {
                            // Find closest point on node AABB to surface voxel
                            int closestX = Math.Max(minX, Math.Min(sx, minX + size - 1));
                            int closestY = Math.Max(minY, Math.Min(sy, minY + size - 1));
                            int closestZ = Math.Max(minZ, Math.Min(sz, minZ + size - 1));
                            
                            int dx = sx - closestX;
                            int dy = sy - closestY;
                            int dz = sz - closestZ;
                            float distSq = dx * dx + dy * dy + dz * dz;
                            
                            if (distSq < minDistSq)
                            {
                                minDistSq = distSq;
                                // Early exit if very close to surface
                                if (minDistSq <= thresholdSq)
                                    return 0f;
                            }
                        }
                    }
                }
            }
            
            return minDistSq == float.MaxValue ? _narrowBand * 10 : MathF.Sqrt(minDistSq) * _resolution;
        }

        public float GetDistanceAtIndex(int x, int y, int z)
        {
            return GetDistanceAtIndexInternal(_root, x, y, z);
        }

        private float GetDistanceAtIndexInternal(OctreeNode node, int x, int y, int z)
        {
            // With pre-computed SDF, directly read from the array instead of traversing octree
            if (_precomputedSDF != null)
            {
                if (x >= 0 && x < _sizeX && y >= 0 && y < _sizeY && z >= 0 && z < _sizeZ)
                {
                    return _precomputedSDF[x, y, z] * _resolution;
                }
                return _narrowBand;
            }
            
            // Legacy octree traversal
            if (node.IsLeaf) return node.Value;
            if (node.Children == null) return node.Value; // Safety check
            
            int half = node.Size / 2;
            int cx = (x >= node.MinX + half) ? 1 : 0;
            int cy = (y >= node.MinY + half) ? 1 : 0;
            int cz = (z >= node.MinZ + half) ? 1 : 0;
            int idx = cx | (cy << 1) | (cz << 2);
            
            if (idx < 0 || idx >= node.Children.Length) return node.Value; // Safety check
            var child = node.Children[idx];
            if (child == null) return node.Value;
            
            return GetDistanceAtIndexInternal(child, x, y, z);
        }

        // Rebuild nodes that intersect the specified voxel index region
        public void UpdateRegion(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            UpdateNodeRegion(_root, minX, minY, minZ, maxX, maxY, maxZ);
        }

        private void UpdateNodeRegion(OctreeNode node, int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            if (node == null) return;
            int nx0 = node.MinX;
            int ny0 = node.MinY;
            int nz0 = node.MinZ;
            int nx1 = node.MinX + node.Size - 1;
            int ny1 = node.MinY + node.Size - 1;
            int nz1 = node.MinZ + node.Size - 1;
            if (maxX < nx0 || minX > nx1 || maxY < ny0 || minY > ny1 || maxZ < nz0 || minZ > nz1)
            {
                return; // no overlap
            }
            // if fully contained, rebuild this node from scratch
            if (minX <= nx0 && minY <= ny0 && minZ <= nz0 && maxX >= nx1 && maxY >= ny1 && maxZ >= nz1)
            {
                // rebuild node
                BuildNode(node);
                return;
            }
            // else partially intersecting; if leaf, subdivide to update children
            if (node.IsLeaf)
            {
                BuildNode(node); // will subdivide as needed when necessary
                if (node.IsLeaf) return; // still leaf, done
            }
            // recursively update children
            foreach (var child in node.Children)
            {
                UpdateNodeRegion(child, minX, minY, minZ, maxX, maxY, maxZ);
            }
            // After possible child updates, attempt to collapse if all children are leaves with similar values
            bool canMerge = true;
            float refVal = node.Children[0].Value;
            foreach (var child in node.Children)
            {
                if (!child.IsLeaf || Math.Abs(child.Value - refVal) > _resolution) { canMerge = false; break; }
            }
            if (canMerge)
            {
                node.IsLeaf = true;
                node.Value = refVal;
                node.Children = null;
            }
        }
    }
}
