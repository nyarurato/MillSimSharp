using System;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Represents a Signed Distance Field (SDF) grid.
    /// <para>
    /// Sign convention (standard SDF): negative = material (solid), positive = empty
    /// (air / removed material), zero = surface. All stored values and distance APIs are in
    /// world-space millimeters.
    /// </para>
    /// <para>
    /// Samples live at voxel centers. Distances from voxel-based construction are computed with
    /// an exact Euclidean Distance Transform and corrected by half a voxel so the zero level set
    /// lies between adjacent voxel centers. Cells outside the grid bounds are treated as empty.
    /// </para>
    /// </summary>
    public class SDFGrid
    {
        private readonly object _sync = new object();
        private readonly int _sizeX;
        private readonly int _sizeY;
        private readonly int _sizeZ;
        private readonly float _resolution;
        private readonly BoundingBox _bounds;
        private readonly float _narrowBandWidth;
        private readonly float[,,] _distances;
        private VoxelGrid? _boundVoxelGrid;

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
        /// Gets the narrow band width in millimeters (maximum distance computed accurately).
        /// </summary>
        public float NarrowBandWidth => _narrowBandWidth;

        /// <summary>
        /// Creates an SDF grid with all material (negative distances).
        /// </summary>
        /// <param name="bounds">Bounding box of the SDF grid.</param>
        /// <param name="resolution">Voxel size in millimeters.</param>
        /// <param name="narrowBandWidth">Width of the narrow band in voxels (default: 10).</param>
        public SDFGrid(BoundingBox bounds, float resolution, int narrowBandWidth = 10)
        {
            if (bounds == null) throw new ArgumentNullException(nameof(bounds));
            if (resolution <= 0) throw new ArgumentException("Resolution must be positive.", nameof(resolution));
            if (narrowBandWidth <= 0) throw new ArgumentException("Narrow band width must be positive.", nameof(narrowBandWidth));

            var size = bounds.Max - bounds.Min;
            _sizeX = Math.Max(1, (int)Math.Ceiling(size.X / resolution));
            _sizeY = Math.Max(1, (int)Math.Ceiling(size.Y / resolution));
            _sizeZ = Math.Max(1, (int)Math.Ceiling(size.Z / resolution));
            _resolution = resolution;
            _bounds = bounds;
            _narrowBandWidth = narrowBandWidth * resolution;
            _distances = new float[_sizeX, _sizeY, _sizeZ];
            _boundVoxelGrid = null;

            FillMaterial();
        }

        /// <summary>
        /// Creates an SDF grid from an existing VoxelGrid.
        /// </summary>
        /// <param name="voxelGrid">Source voxel grid.</param>
        /// <param name="narrowBandWidth">Width of the narrow band in voxels (default: 10).</param>
        public static SDFGrid FromVoxelGrid(VoxelGrid voxelGrid, int narrowBandWidth = 10)
        {
            if (voxelGrid == null) throw new ArgumentNullException(nameof(voxelGrid));

            var dimensions = voxelGrid.Dimensions;
            var sdf = new SDFGrid(voxelGrid, dimensions.X, dimensions.Y, dimensions.Z,
                                  voxelGrid.Resolution, voxelGrid.Bounds, narrowBandWidth);

            SignedDistanceFieldBuilder.ComputeRegion(
                (x, y, z) => voxelGrid.GetVoxel(x, y, z),
                dimensions.X, dimensions.Y, dimensions.Z,
                voxelGrid.Resolution, sdf._narrowBandWidth,
                sdf._distances,
                0, 0, 0,
                dimensions.X - 1, dimensions.Y - 1, dimensions.Z - 1);

            return sdf;
        }

        /// <summary>
        /// Private constructor used by FromVoxelGrid. Distances are filled by the caller.
        /// </summary>
        private SDFGrid(VoxelGrid voxelGrid, int sizeX, int sizeY, int sizeZ,
               float resolution, BoundingBox bounds, int narrowBandWidth)
        {
            if (narrowBandWidth <= 0) throw new ArgumentException("Narrow band width must be positive.", nameof(narrowBandWidth));

            _sizeX = sizeX;
            _sizeY = sizeY;
            _sizeZ = sizeZ;
            _resolution = resolution;
            _bounds = bounds;
            _narrowBandWidth = narrowBandWidth * resolution;
            _distances = new float[_sizeX, _sizeY, _sizeZ];
            _boundVoxelGrid = voxelGrid;
        }

        private void FillMaterial()
        {
            for (int z = 0; z < _sizeZ; z++)
            for (int y = 0; y < _sizeY; y++)
            for (int x = 0; x < _sizeX; x++)
            {
                _distances[x, y, z] = -_narrowBandWidth;
            }
        }

        /// <summary>
        /// Bind to a VoxelGrid so that we can react to its VoxelsChanged events and perform incremental SDF updates.
        /// </summary>
        public void BindToVoxelGrid(VoxelGrid grid)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));
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
            if (_boundVoxelGrid == null) return;
            RebuildRegion(_boundVoxelGrid, minX, minY, minZ, maxX, maxY, maxZ);
        }

        /// <summary>
        /// Recomputes the SDF in the region affected by voxel changes.
        /// The write region is expanded by the narrow band, because distances up to one narrow
        /// band away from the changed voxels can change as well.
        /// </summary>
        /// <param name="voxelGrid">Source voxel grid.</param>
        /// <param name="minX">Minimum changed voxel index X.</param>
        /// <param name="minY">Minimum changed voxel index Y.</param>
        /// <param name="minZ">Minimum changed voxel index Z.</param>
        /// <param name="maxX">Maximum changed voxel index X.</param>
        /// <param name="maxY">Maximum changed voxel index Y.</param>
        /// <param name="maxZ">Maximum changed voxel index Z.</param>
        public void UpdateRegionFromVoxelGrid(VoxelGrid voxelGrid, int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            if (voxelGrid == null) throw new ArgumentNullException(nameof(voxelGrid));
            RebuildRegion(voxelGrid, minX, minY, minZ, maxX, maxY, maxZ);
        }

        private void RebuildRegion(VoxelGrid voxelGrid, int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            int band = Math.Max(1, (int)Math.Ceiling(_narrowBandWidth / _resolution));

            minX = Math.Max(0, minX - band);
            minY = Math.Max(0, minY - band);
            minZ = Math.Max(0, minZ - band);
            maxX = Math.Min(_sizeX - 1, maxX + band);
            maxY = Math.Min(_sizeY - 1, maxY + band);
            maxZ = Math.Min(_sizeZ - 1, maxZ + band);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            lock (_sync)
            {
                SignedDistanceFieldBuilder.ComputeRegion(
                    (x, y, z) => voxelGrid.GetVoxel(x, y, z),
                    _sizeX, _sizeY, _sizeZ,
                    _resolution, _narrowBandWidth,
                    _distances,
                    minX, minY, minZ,
                    maxX, maxY, maxZ);
            }
        }

        /// <summary>
        /// Checks whether an index is outside the SDF grid.
        /// </summary>
        private bool IsOutOfBoundsIndex(int x, int y, int z)
        {
            return x < 0 || x >= _sizeX || y < 0 || y >= _sizeY || z < 0 || z >= _sizeZ;
        }

        /// <summary>
        /// Computes the distance from a voxel center to the closest point inside the bounding box.
        /// The result is clamped to the narrow band width and is always positive (outside = empty).
        /// </summary>
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

        /// <summary>
        /// Gets the signed distance at the specified voxel indices.
        /// </summary>
        /// <param name="x">X index</param>
        /// <param name="y">Y index</param>
        /// <param name="z">Z index</param>
        /// <returns>Signed distance in millimeters. Negative = material, positive = empty.</returns>
        public float GetDistance(int x, int y, int z)
        {
            if (IsOutOfBoundsIndex(x, y, z))
            {
                // Outside the grid is empty space (positive distance).
                return DistanceFromVoxelCenterToBounds(x, y, z);
            }

            return _distances[x, y, z];
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
        /// Gets the signed distance at a world position using trilinear interpolation.
        /// Voxel samples are treated as living at voxel centers (half-voxel offset).
        /// </summary>
        /// <param name="worldPos">World position</param>
        /// <returns>Interpolated signed distance value in millimeters</returns>
        public float GetDistance(Vector3 worldPos)
        {
            // Convert to sample space (voxel centers live at integer + 0.5 in world/res units).
            float fx = (worldPos.X - _bounds.Min.X) / _resolution - 0.5f;
            float fy = (worldPos.Y - _bounds.Min.Y) / _resolution - 0.5f;
            float fz = (worldPos.Z - _bounds.Min.Z) / _resolution - 0.5f;

            int x0 = (int)MathF.Floor(fx);
            int y0 = (int)MathF.Floor(fy);
            int z0 = (int)MathF.Floor(fz);
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
        /// The gradient points from material toward empty space (outward from the solid).
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

        private void ClampRegion(ref int minX, ref int minY, ref int minZ, ref int maxX, ref int maxY, ref int maxZ)
        {
            minX = Math.Max(0, minX);
            minY = Math.Max(0, minY);
            minZ = Math.Max(0, minZ);
            maxX = Math.Min(_sizeX - 1, maxX);
            maxY = Math.Min(_sizeY - 1, maxY);
            maxZ = Math.Min(_sizeZ - 1, maxZ);
        }

        private void Carve(int x, int y, int z, float amount)
        {
            float clamped = amount;
            if (clamped > _narrowBandWidth) clamped = _narrowBandWidth;
            if (clamped > _distances[x, y, z]) _distances[x, y, z] = clamped;
        }

        /// <summary>
        /// Checks whether any material sample inside <paramref name="worldBounds"/> also lies inside
        /// the tool solid described by <paramref name="toolSignedDistance"/> (negative = inside).
        /// Read-only: does not modify the SDF.
        /// </summary>
        internal bool IntersectsToolSolid(BoundingBox worldBounds, Func<Vector3, float> toolSignedDistance)
        {
            if (worldBounds == null) throw new ArgumentNullException(nameof(worldBounds));
            if (toolSignedDistance == null) throw new ArgumentNullException(nameof(toolSignedDistance));

            var (minX, minY, minZ) = WorldToVoxel(worldBounds.Min);
            var (maxX, maxY, maxZ) = WorldToVoxel(worldBounds.Max);
            ClampRegion(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return false;

            for (int z = minZ; z <= maxZ; z++)
            for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                if (_distances[x, y, z] >= 0f) continue;
                if (toolSignedDistance(VoxelToWorld(x, y, z)) < 0f) return true;
            }

            return false;
        }

        /// <summary>
        /// Applies a CSG difference for the tool solid described by <paramref name="toolSignedDistance"/>
        /// (negative inside the tool) over the given world bounds.
        /// </summary>
        internal void CarveRegion(BoundingBox worldBounds, Func<Vector3, float> toolSignedDistance)
        {
            if (worldBounds == null) throw new ArgumentNullException(nameof(worldBounds));
            if (toolSignedDistance == null) throw new ArgumentNullException(nameof(toolSignedDistance));

            var (minX, minY, minZ) = WorldToVoxel(worldBounds.Min);
            var (maxX, maxY, maxZ) = WorldToVoxel(worldBounds.Max);
            ClampRegion(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            for (int z = minZ; z <= maxZ; z++)
            for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                float distance = toolSignedDistance(VoxelToWorld(x, y, z));
                Carve(x, y, z, -distance);
            }
        }

        /// <summary>
        /// Removes material in a spherical region using the CSG difference operation
        /// <c>d = max(d, radius - |p - center|)</c>.
        /// </summary>
        /// <param name="center">Center of the sphere in world coordinates.</param>
        /// <param name="radius">Radius of the sphere in millimeters.</param>
        public void RemoveSphere(Vector3 center, float radius)
        {
            if (radius <= 0) return;

            var (minX, minY, minZ) = WorldToVoxel(center - new Vector3(radius, radius, radius));
            var (maxX, maxY, maxZ) = WorldToVoxel(center + new Vector3(radius, radius, radius));
            ClampRegion(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            for (int z = minZ; z <= maxZ; z++)
            for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                float distToCenter = Vector3.Distance(VoxelToWorld(x, y, z), center);
                Carve(x, y, z, radius - distToCenter);
            }
        }

        /// <summary>
        /// Removes material in a capsule region (line segment with spherical ends) using the CSG
        /// difference operation <c>d = max(d, radius - distanceToSegment)</c>.
        /// </summary>
        /// <param name="start">Start point of the capsule axis in world coordinates.</param>
        /// <param name="end">End point of the capsule axis in world coordinates.</param>
        /// <param name="radius">Radius of the capsule in millimeters.</param>
        public void RemoveCapsule(Vector3 start, Vector3 end, float radius)
        {
            if (radius <= 0) return;

            Vector3 axis = end - start;
            float length = axis.Length();
            if (length < 1e-6f)
            {
                RemoveSphere(start, radius);
                return;
            }
            Vector3 axisDir = axis / length;

            Vector3 regionMin = Vector3.Min(start, end) - new Vector3(radius);
            Vector3 regionMax = Vector3.Max(start, end) + new Vector3(radius);
            var (minX, minY, minZ) = WorldToVoxel(regionMin);
            var (maxX, maxY, maxZ) = WorldToVoxel(regionMax);
            ClampRegion(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            for (int z = minZ; z <= maxZ; z++)
            for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                Vector3 toVoxel = VoxelToWorld(x, y, z) - start;
                float projection = Math.Clamp(Vector3.Dot(toVoxel, axisDir), 0f, length);
                float distToSegment = (toVoxel - axisDir * projection).Length();
                Carve(x, y, z, radius - distToSegment);
            }
        }

        /// <summary>
        /// Removes material in a finite flat-ended cylinder using the CSG difference operation.
        /// Unlike <see cref="RemoveCapsule"/>, the end faces are flat.
        /// </summary>
        /// <param name="start">Center of one end face in world coordinates.</param>
        /// <param name="end">Center of the other end face in world coordinates.</param>
        /// <param name="radius">Radius of the cylinder in millimeters.</param>
        public void RemoveFiniteCylinder(Vector3 start, Vector3 end, float radius)
        {
            if (radius <= 0) return;

            Vector3 axis = end - start;
            float length = axis.Length();
            if (length < 1e-6f) return; // Degenerate cylinder has no volume
            Vector3 axisDir = axis / length;
            Vector3 center = (start + end) * 0.5f;
            float halfLength = length * 0.5f;

            Vector3 regionMin = Vector3.Min(start, end) - new Vector3(radius);
            Vector3 regionMax = Vector3.Max(start, end) + new Vector3(radius);
            var (minX, minY, minZ) = WorldToVoxel(regionMin);
            var (maxX, maxY, maxZ) = WorldToVoxel(regionMax);
            ClampRegion(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            if (minX > maxX || minY > maxY || minZ > maxZ) return;

            for (int z = minZ; z <= maxZ; z++)
            for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                Vector3 toVoxel = VoxelToWorld(x, y, z) - center;
                float axial = Vector3.Dot(toVoxel, axisDir);
                float radial = (toVoxel - axisDir * axial).Length();
                float toolDistance = Math.Max(radial - radius, Math.Abs(axial) - halfLength);
                Carve(x, y, z, -toolDistance);
            }
        }
    }
}
