using System;
using System.Numerics;
using System.Threading.Tasks;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Represents a Signed Distance Field (SDF) grid for smooth surface representation.
    /// Stores signed distance values where negative indicates inside, positive outside, and zero on the surface.
    /// </summary>
    public class SDFGrid
    {
        private readonly object _sync = new object();
        private readonly float[][][] _distances;
        private readonly System.Collections.Concurrent.ConcurrentDictionary<int, float>? _sparseDistances;
        private readonly bool _useSparse;
        private readonly int _sizeX;
        private readonly int _sizeY;
        private readonly int _sizeZ;
        private readonly float _resolution;
        private readonly BoundingBox _bounds;
        private readonly float _narrowBandWidth;
        private readonly bool _fastMode;

        /// <summary>
        /// Gets the resolution (voxel size) in millimeters.
        /// </summary>
        public float Resolution => _resolution;

        /// <summary>
        /// Gets the bounding box of the field.
        /// </summary>
        public BoundingBox Bounds => _bounds;

        /// <summary>
        /// Gets the dimensions of the SDF grid.
        /// </summary>
        public (int X, int Y, int Z) Dimensions => (_sizeX, _sizeY, _sizeZ);

        /// <summary>
        /// Gets the narrow band width (maximum distance computed accurately).
        /// </summary>
        public float NarrowBandWidth => _narrowBandWidth;

        /// <summary>
        /// Creates a Signed Distance Field grid from a VoxelGrid.
        /// </summary>
        /// <param name="voxelGrid">The source voxel grid.</param>
        /// <param name="narrowBandWidth">Width of the narrow band in voxels (default: 10). 
        /// Only distances within this range are computed accurately, others are clamped.</param>
        /// <returns>A new SDFGrid instance.</returns>
        public static SDFGrid FromVoxelGrid(VoxelGrid voxelGrid, int narrowBandWidth = 10, bool useSparse = false, bool fastMode = false)
        {
            var dimensions = voxelGrid.Dimensions;
            // honor environment variable to force fast-mode for tests or CI runs if desired
            bool envFast = false;
            try { var v = Environment.GetEnvironmentVariable("MILLSIM_FAST_TESTS"); envFast = v == "1" || string.Equals(v, "true", StringComparison.OrdinalIgnoreCase); } catch { }
            bool chosenFastMode = fastMode || envFast;
            return new SDFGrid(voxelGrid, dimensions.X, dimensions.Y, dimensions.Z, 
                             voxelGrid.Resolution, voxelGrid.Bounds, narrowBandWidth, useSparse, chosenFastMode);
        }

        // Helper: Check whether an index is out of bounds of the underlying SDF grid
        private bool IsOutOfBoundsIndex(int x, int y, int z)
        {
            return x < 0 || x >= _sizeX || y < 0 || y >= _sizeY || z < 0 || z >= _sizeZ;
        }

        // Helper: Compute the distance from the voxel center at index (x,y,z) to the closest
        // point inside the bounding box. The result is clamped to the narrow band width.
        private float DistanceFromVoxelCenterToBounds(int x, int y, int z)
        {
            float cx = _bounds.Min.X + (x + 0.5f) * _resolution;
            float cy = _bounds.Min.Y + (y + 0.5f) * _resolution;
            float cz = _bounds.Min.Z + (z + 0.5f) * _resolution;
            float dx = 0, dy = 0, dz = 0;
            if (cx < _bounds.Min.X) dx = _bounds.Min.X - cx;
            else if (cx > _bounds.Max.X) dx = cx - _bounds.Max.X;
            if (cy < _bounds.Min.Y) dy = _bounds.Min.Y - cy;
            else if (cy > _bounds.Max.Y) dy = cy - _bounds.Max.Y;
            if (cz < _bounds.Min.Z) dz = _bounds.Min.Z - cz;
            else if (cz > _bounds.Max.Z) dz = cz - _bounds.Max.Z;
            float dist = MathF.Sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > _narrowBandWidth) dist = _narrowBandWidth;
            return dist;
        }

        private VoxelGrid? _boundVoxelGrid = null;

        /// <summary>
        /// Bind to a VoxelGrid so that we can react to its VoxelsChanged events and perform incremental SDF updates.
        /// </summary>
        public void BindToVoxelGrid(VoxelGrid grid)
        {
            if (_boundVoxelGrid != null)
                UnbindFromVoxelGrid();

            _boundVoxelGrid = grid;
            grid.VoxelsChanged += OnVoxelGridChanged;
        }

        /// <summary>
        /// Unbind from the VoxelGrid events.
        /// </summary>
        public void UnbindFromVoxelGrid()
        {
            if (_boundVoxelGrid == null) return;
            _boundVoxelGrid.VoxelsChanged -= OnVoxelGridChanged;
            _boundVoxelGrid = null;
        }

        private void OnVoxelGridChanged(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            // Expand by narrow band in voxel units
            int bandVoxels = Math.Max(1, (int)Math.Ceiling(_narrowBandWidth / _resolution));
            int nx0 = Math.Max(0, minX - bandVoxels);
            int ny0 = Math.Max(0, minY - bandVoxels);
            int nz0 = Math.Max(0, minZ - bandVoxels);
            int nx1 = Math.Min(_sizeX - 1, maxX + bandVoxels);
            int ny1 = Math.Min(_sizeY - 1, maxY + bandVoxels);
            int nz1 = Math.Min(_sizeZ - 1, maxZ + bandVoxels);

            // Recompute this region
            UpdateRegionFromVoxelGrid(_boundVoxelGrid!, nx0, ny0, nz0, nx1, ny1, nz1);
        }

        /// <summary>
        /// Private constructor used by FromVoxelGrid.
        /// </summary>
        private SDFGrid(VoxelGrid voxelGrid, int sizeX, int sizeY, int sizeZ, 
               float resolution, BoundingBox bounds, int narrowBandWidth, bool useSparse = false, bool fastMode = false)
                       
        {
            _sizeX = sizeX;
            _sizeY = sizeY;
            _sizeZ = sizeZ;
            _resolution = resolution;
            _bounds = bounds;
            _narrowBandWidth = narrowBandWidth * resolution;
            _fastMode = fastMode;

            // Initialize distance array
            _distances = new float[_sizeX][][];
            for (int x = 0; x < _sizeX; x++)
            {
                _distances[x] = new float[_sizeY][];
                for (int y = 0; y < _sizeY; y++)
                {
                    _distances[x][y] = new float[_sizeZ];
                }
            }

            _useSparse = useSparse;
            _sparseDistances = useSparse ? new System.Collections.Concurrent.ConcurrentDictionary<int, float>() : null;
            _boundVoxelGrid = voxelGrid; // remember original grid for on-demand distance queries

            // Compute the signed distance field (fast-mode uses simpler, approximate compute)
            if (_fastMode) ComputeSDFFast(voxelGrid, narrowBandWidth);
            else ComputeSDF(voxelGrid, narrowBandWidth);
        }

        /// <summary>
        /// Computes the signed distance field from the voxel grid.
        /// Uses a scan-based algorithm with narrow-band optimization.
        /// </summary>
        private void ComputeSDF(VoxelGrid voxelGrid, int narrowBandWidth)
        {
            // Parallel computation for each voxel
            Parallel.For(0, _sizeZ, z =>
            {
                for (int y = 0; y < _sizeY; y++)
                {
                    for (int x = 0; x < _sizeX; x++)
                    {
                        float distance = ComputeDistanceAt(voxelGrid, x, y, z, narrowBandWidth);
                        if (_useSparse)
                        {
                            int idx = x + y * _sizeX + z * _sizeX * _sizeY;
                            _sparseDistances![idx] = distance;
                        }
                        else
                        {
                            lock (_sync) { _distances[x][y][z] = distance; }
                        }
                    }
                }
            });
        }

        /// <summary>
        /// A faster, approximate SDF computation used for tests or low-cost runs.
        /// This reduces the search radius and uses a cheap axis-aligned scan to find
        /// a nearby surface voxel rather than exhaustive radius searching.
        /// Intended to preserve sign information but be much cheaper to compute.
        /// </summary>
        private void ComputeSDFFast(VoxelGrid voxelGrid, int narrowBandWidth)
        {
            int maxScan = Math.Max(1, Math.Min((int)Math.Ceiling(narrowBandWidth / _resolution), 12)); // cap for speed
            Parallel.For(0, _sizeZ, z =>
            {
                for (int y = 0; y < _sizeY; y++)
                {
                    for (int x = 0; x < _sizeX; x++)
                    {
                        float distance = ComputeDistanceAtFast(voxelGrid, x, y, z, maxScan);
                        if (_useSparse)
                        {
                            int idx = x + y * _sizeX + z * _sizeX * _sizeY;
                            _sparseDistances![idx] = distance;
                        }
                        else
                        {
                            lock (_sync) { _distances[x][y][z] = distance; }
                        }
                    }
                }
            });
        }

        private float ComputeDistanceAtFast(VoxelGrid voxelGrid, int x, int y, int z, int maxScan)
        {
            bool isMaterial = voxelGrid.GetVoxel(x, y, z);
            float minDistSq = float.MaxValue;
            // simple axis-aligned scan
            for (int dir = 0; dir < 6; dir++)
            {
                int dx = 0, dy = 0, dz = 0;
                switch (dir)
                {
                    case 0: dx = 1; break;
                    case 1: dx = -1; break;
                    case 2: dy = 1; break;
                    case 3: dy = -1; break;
                    case 4: dz = 1; break;
                    case 5: dz = -1; break;
                }
                for (int s = 1; s <= maxScan; s++)
                {
                    int sx = x + dx * s;
                    int sy = y + dy * s;
                    int sz = z + dz * s;
                    if (sx < 0 || sx >= _sizeX || sy < 0 || sy >= _sizeY || sz < 0 || sz >= _sizeZ)
                        break;
                    if (IsSurfaceVoxel(voxelGrid, sx, sy, sz))
                    {
                        float d = (s * _resolution);
                        float dsq = d * d;
                        if (dsq < minDistSq) minDistSq = dsq;
                        break; // stop scanning along this axis direction
                    }
                }
            }

            if (minDistSq == float.MaxValue)
            {
                // No surface found during fast axis-aligned scan — fallback to more accurate but
                // bounded computation to avoid returning the full narrow band value for interior points.
                int fallbackRadius = Math.Min((int)Math.Ceiling(_narrowBandWidth / _resolution), 64);
                return ComputeDistanceAt(voxelGrid, x, y, z, fallbackRadius);
            }
            float worldDist = MathF.Sqrt(minDistSq);
            float signed = isMaterial ? worldDist : -worldDist;
            if (Math.Abs(signed) > _narrowBandWidth) signed = MathF.Sign(signed) * _narrowBandWidth;
            return signed;
        }

        /// <summary>
        /// Incrementally recompute SDF in a voxel index region.
        /// </summary>
        public void UpdateRegionFromVoxelGrid(VoxelGrid voxelGrid, int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            int searchRadius = Math.Max(1, (int)Math.Ceiling(_narrowBandWidth / _resolution));
            minX = Math.Max(0, minX);
            minY = Math.Max(0, minY);
            minZ = Math.Max(0, minZ);
            maxX = Math.Min(_sizeX - 1, maxX);
            maxY = Math.Min(_sizeY - 1, maxY);
            maxZ = Math.Min(_sizeZ - 1, maxZ);

            // Optimization: collect surface voxel world positions in an expanded neighborhood
            int surfMinX = Math.Max(0, minX - searchRadius);
            int surfMinY = Math.Max(0, minY - searchRadius);
            int surfMinZ = Math.Max(0, minZ - searchRadius);
            int surfMaxX = Math.Min(_sizeX - 1, maxX + searchRadius);
            int surfMaxY = Math.Min(_sizeY - 1, maxY + searchRadius);
            int surfMaxZ = Math.Min(_sizeZ - 1, maxZ + searchRadius);

            var surfaceCenters = new System.Collections.Concurrent.ConcurrentBag<Vector3>();

            Parallel.For(surfMinZ, surfMaxZ + 1, z =>
            {
                for (int y = surfMinY; y <= surfMaxY; y++)
                {
                    for (int x = surfMinX; x <= surfMaxX; x++)
                    {
                        if (IsSurfaceVoxel(voxelGrid, x, y, z))
                        {
                            surfaceCenters.Add(VoxelToWorld(x, y, z));
                        }
                    }
                }
            });

            // If no surface voxels found, fall back to full compute
            Vector3[] surfaces = surfaceCenters.ToArray();
            // NOTE: Previously debug logging added here; removed for regular runs.
            if (surfaces.Length == 0)
            {
                Parallel.For(minZ, maxZ + 1, z =>
                {
                    for (int y = minY; y <= maxY; y++)
                    {
                        for (int x = minX; x <= maxX; x++)
                        {
                            float distance = _fastMode ? ComputeDistanceAtFast(voxelGrid, x, y, z, Math.Min(4, searchRadius)) : ComputeDistanceAt(voxelGrid, x, y, z, searchRadius);
                            if (_useSparse)
                            {
                                int idx = x + y * _sizeX + z * _sizeX * _sizeY;
                                _sparseDistances![idx] = distance;
                            }
                            else
                            {
                                lock (_sync) { _distances[x][y][z] = distance; }
                            }
                        }
                    }
                });
                return;
            }

            // Otherwise compute distances as minimum distance to any surface center in world coordinates
            Parallel.For(minZ, maxZ + 1, z =>
            {
                for (int y = minY; y <= maxY; y++)
                {
                    for (int x = minX; x <= maxX; x++)
                    {
                        var center = VoxelToWorld(x, y, z);
                        float minDistSq = float.MaxValue;
                        foreach (var sc in surfaces)
                        {
                            float d = Vector3.DistanceSquared(center, sc);
                            if (d < minDistSq) minDistSq = d;
                        }
                        float worldDist = MathF.Sqrt(minDistSq);
                        float signedDistance = voxelGrid.GetVoxel(x, y, z) ? worldDist : -worldDist;
                        // No debug logging here anymore.
                        // Clamp
                        if (Math.Abs(signedDistance) > _narrowBandWidth)
                            signedDistance = MathF.Sign(signedDistance) * _narrowBandWidth;
                        if (_useSparse)
                        {
                            int idx = x + y * _sizeX + z * _sizeX * _sizeY;
                            _sparseDistances![idx] = signedDistance;
                        }
                        else
                        {
                            lock (_sync) { _distances[x][y][z] = signedDistance; }
                        }
                    }
                }
            });
        }

        /// <summary>
        /// Computes the signed distance at a specific voxel location.
        /// </summary>
        private float ComputeDistanceAt(VoxelGrid voxelGrid, int x, int y, int z, int searchRadius)
        {
            bool isMaterial = voxelGrid.GetVoxel(x, y, z);
            
            // Find the closest surface voxel (boundary between material and empty)
            float minDistance = float.MaxValue;
            bool foundSurface = false;

            // Search in a cube around the current voxel
            int minX = Math.Max(0, x - searchRadius);
            int maxX = Math.Min(_sizeX - 1, x + searchRadius);
            int minY = Math.Max(0, y - searchRadius);
            int maxY = Math.Min(_sizeY - 1, y + searchRadius);
            int minZ = Math.Max(0, z - searchRadius);
            int maxZ = Math.Min(_sizeZ - 1, z + searchRadius);

            for (int sz = minZ; sz <= maxZ; sz++)
            {
                for (int sy = minY; sy <= maxY; sy++)
                {
                    for (int sx = minX; sx <= maxX; sx++)
                    {
                        // Check if this is a surface voxel (has a neighbor with different state)
                        if (IsSurfaceVoxel(voxelGrid, sx, sy, sz))
                        {
                            // Calculate distance in voxel space
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

            // Convert to world space distance
            float worldDistance = minDistance * _resolution;

            // Clamp to narrow band width
            if (worldDistance > _narrowBandWidth)
            {
                worldDistance = _narrowBandWidth;
            }

            // Apply sign: negative inside empty space (removed material), positive in material
            // isMaterial=false means empty (removed), which should be negative (inside the carved volume)
            float signedDistance = isMaterial ? worldDistance : -worldDistance;

            // If no surface found and we're at the boundary, clamp
            if (!foundSurface)
            {
                signedDistance = isMaterial ? _narrowBandWidth : -_narrowBandWidth;
            }

            return signedDistance;
        }

        /// <summary>
        /// Checks if a voxel is on the surface (has at least one neighbor with different material state).
        /// </summary>
        private bool IsSurfaceVoxel(VoxelGrid voxelGrid, int x, int y, int z)
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

                // Check bounds
                if (nx < 0 || nx >= _sizeX || ny < 0 || ny >= _sizeY || nz < 0 || nz >= _sizeZ)
                {
                    // If out of bounds, treat as empty (different from material)
                    if (current)
                        return true;
                    continue;
                }

                bool neighbor = voxelGrid.GetVoxel(nx, ny, nz);
                if (neighbor != current)
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Gets the signed distance at the specified voxel indices.
        /// </summary>
        /// <param name="x">X index</param>
        /// <param name="y">Y index</param>
        /// <param name="z">Z index</param>
        /// <returns>Signed distance value. Negative inside, positive outside, zero on surface.</returns>
        public float GetDistance(int x, int y, int z)
        {
            if (IsOutOfBoundsIndex(x, y, z))
            {
                return -DistanceFromVoxelCenterToBounds(x, y, z);
            }

            if (_useSparse)
            {
                int idx = x + y * _sizeX + z * _sizeX * _sizeY;
                if (_sparseDistances != null && _sparseDistances.TryGetValue(idx, out float value))
                    return value;
                // Fallback: if we have a bound voxel grid, use material to determine sign, but
                // still return a magnitude that matches the narrow band (we could compute
                // exact world distance if desired). If a value isn't found in the sparse store,
                // return the clamped narrow-band value with appropriate sign.
                if (_boundVoxelGrid != null)
                {
                    // If no computed SDF exists, compute it on-demand (bounded radius) and cache it.
                    int searchRadius = Math.Max(1, (int)Math.Ceiling(_narrowBandWidth / _resolution));
                    float computed = ComputeDistanceAt(_boundVoxelGrid, x, y, z, searchRadius);
                    int idx2 = x + y * _sizeX + z * _sizeX * _sizeY;
                    _sparseDistances![idx2] = computed;
                    return computed;
                }
                return _narrowBandWidth;
            }

            lock (_sync) { return _distances[x][y][z]; }
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
        /// Gets the signed distance at a world position using trilinear interpolation.
        /// </summary>
        /// <param name="worldPos">World position</param>
        /// <returns>Interpolated signed distance value</returns>
        public float GetDistance(Vector3 worldPos)
        {
            // Convert to voxel space
            Vector3 localPos = worldPos - _bounds.Min;
            float fx = localPos.X / _resolution;
            float fy = localPos.Y / _resolution;
            float fz = localPos.Z / _resolution;

            // Get integer and fractional parts
            int x0 = (int)Math.Floor(fx);
            int y0 = (int)Math.Floor(fy);
            int z0 = (int)Math.Floor(fz);
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;

            float tx = fx - x0;
            float ty = fy - y0;
            float tz = fz - z0;

            // Trilinear interpolation
            float c000 = GetDistance(x0, y0, z0);
            float c001 = GetDistance(x0, y0, z1);
            float c010 = GetDistance(x0, y1, z0);
            float c011 = GetDistance(x0, y1, z1);
            float c100 = GetDistance(x1, y0, z0);
            float c101 = GetDistance(x1, y0, z1);
            float c110 = GetDistance(x1, y1, z0);
            float c111 = GetDistance(x1, y1, z1);

            float c00 = c000 * (1 - tx) + c100 * tx;
            float c01 = c001 * (1 - tx) + c101 * tx;
            float c10 = c010 * (1 - tx) + c110 * tx;
            float c11 = c011 * (1 - tx) + c111 * tx;

            float c0 = c00 * (1 - ty) + c10 * ty;
            float c1 = c01 * (1 - ty) + c11 * ty;

            return c0 * (1 - tz) + c1 * tz;
        }

        /// <summary>
        /// Computes the gradient (approximate surface normal) at a world position using central differences.
        /// </summary>
        /// <param name="worldPos">World position</param>
        /// <returns>Normalized gradient vector (approximate surface normal)</returns>
        public Vector3 GetGradient(Vector3 worldPos)
        {
            float h = _resolution; // Step size for finite differences
            float dx = (GetDistance(worldPos + new Vector3(h, 0, 0)) - 
                       GetDistance(worldPos - new Vector3(h, 0, 0))) / (2 * h);
            float dy = (GetDistance(worldPos + new Vector3(0, h, 0)) - 
                       GetDistance(worldPos - new Vector3(0, h, 0))) / (2 * h);
            float dz = (GetDistance(worldPos + new Vector3(0, 0, h)) - 
                       GetDistance(worldPos - new Vector3(0, 0, h))) / (2 * h);

            Vector3 gradient = new Vector3(dx, dy, dz);
            float length = gradient.Length();
            
            if (length > 1e-6f)
            {
                return Vector3.Normalize(gradient);
            }
            
            return Vector3.UnitY; // Default normal if gradient is zero
        }
    }
}
