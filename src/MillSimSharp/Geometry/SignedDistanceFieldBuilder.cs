using System;
using System.Threading.Tasks;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Computes signed distance fields from voxel occupancy using an exact Euclidean Distance
    /// Transform (Felzenszwalb &amp; Huttenlocher).
    /// <para>
    /// Sign convention: negative = material (solid), positive = empty (air / removed material).
    /// Distances are stored in world-space millimeters.
    /// </para>
    /// <para>
    /// Voxel samples live at voxel centers. Distances are computed to the nearest opposite-state
    /// voxel center and corrected by half a voxel, so the zero level set lies between adjacent
    /// voxel centers. Cells outside the grid are treated as empty (air).
    /// </para>
    /// </summary>
    internal static class SignedDistanceFieldBuilder
    {
        private const float HugeValue = 1e20f;

        /// <summary>
        /// Recomputes signed distances for voxels inside the write region.
        /// The compute window is the write region expanded by the narrow band (to include every
        /// feature that can affect the region) and padded by one voxel (to represent air outside
        /// the grid). Results are written to <paramref name="target"/> in millimeters.
        /// </summary>
        public static void ComputeRegion(
            Func<int, int, int, bool> isMaterial,
            int sizeX, int sizeY, int sizeZ,
            float resolution, float narrowBand,
            float[,,] target,
            int writeMinX, int writeMinY, int writeMinZ,
            int writeMaxX, int writeMaxY, int writeMaxZ)
        {
            if (isMaterial == null) throw new ArgumentNullException(nameof(isMaterial));
            if (target == null) throw new ArgumentNullException(nameof(target));

            writeMinX = Math.Max(0, writeMinX);
            writeMinY = Math.Max(0, writeMinY);
            writeMinZ = Math.Max(0, writeMinZ);
            writeMaxX = Math.Min(sizeX - 1, writeMaxX);
            writeMaxY = Math.Min(sizeY - 1, writeMaxY);
            writeMaxZ = Math.Min(sizeZ - 1, writeMaxZ);
            if (writeMinX > writeMaxX || writeMinY > writeMaxY || writeMinZ > writeMaxZ)
                return;

            int band = Math.Max(1, (int)Math.Ceiling(narrowBand / resolution));

            int padMinX = Math.Max(0, writeMinX - band) - 1;
            int padMinY = Math.Max(0, writeMinY - band) - 1;
            int padMinZ = Math.Max(0, writeMinZ - band) - 1;
            int padMaxX = Math.Min(sizeX - 1, writeMaxX + band) + 1;
            int padMaxY = Math.Min(sizeY - 1, writeMaxY + band) + 1;
            int padMaxZ = Math.Min(sizeZ - 1, writeMaxZ + band) + 1;

            int nx = padMaxX - padMinX + 1;
            int ny = padMaxY - padMinY + 1;
            int nz = padMaxZ - padMinZ + 1;

            var work = new float[nx, ny, nz];

            // Pass 1: distance to the nearest empty voxel (seeds = empty, including outside the grid).
            Initialize(work, isMaterial, padMinX, padMinY, padMinZ, sizeX, sizeY, sizeZ, seedMaterial: false);
            DistanceTransform3D(work, nx, ny, nz);
            for (int z = writeMinZ; z <= writeMaxZ; z++)
            {
                for (int y = writeMinY; y <= writeMaxY; y++)
                {
                    for (int x = writeMinX; x <= writeMaxX; x++)
                    {
                        if (!isMaterial(x, y, z)) continue;
                        float d = MathF.Sqrt(work[x - padMinX, y - padMinY, z - padMinZ]) - 0.5f;
                        target[x, y, z] = Clamp(-d * resolution, narrowBand);
                    }
                }
            }

            // Pass 2: distance to the nearest material voxel (seeds = material).
            Initialize(work, isMaterial, padMinX, padMinY, padMinZ, sizeX, sizeY, sizeZ, seedMaterial: true);
            DistanceTransform3D(work, nx, ny, nz);
            for (int z = writeMinZ; z <= writeMaxZ; z++)
            {
                for (int y = writeMinY; y <= writeMaxY; y++)
                {
                    for (int x = writeMinX; x <= writeMaxX; x++)
                    {
                        if (isMaterial(x, y, z)) continue;
                        float d = MathF.Sqrt(work[x - padMinX, y - padMinY, z - padMinZ]) - 0.5f;
                        target[x, y, z] = Clamp(d * resolution, narrowBand);
                    }
                }
            }
        }

        private static float Clamp(float value, float narrowBand)
        {
            if (value < -narrowBand) return -narrowBand;
            if (value > narrowBand) return narrowBand;
            return value;
        }

        private static void Initialize(
            float[,,] work,
            Func<int, int, int, bool> isMaterial,
            int padMinX, int padMinY, int padMinZ,
            int sizeX, int sizeY, int sizeZ,
            bool seedMaterial)
        {
            int nx = work.GetLength(0);
            int ny = work.GetLength(1);
            int nz = work.GetLength(2);

            for (int wz = 0; wz < nz; wz++)
            {
                int gz = padMinZ + wz;
                for (int wy = 0; wy < ny; wy++)
                {
                    int gy = padMinY + wy;
                    for (int wx = 0; wx < nx; wx++)
                    {
                        int gx = padMinX + wx;
                        bool inside = gx >= 0 && gx < sizeX && gy >= 0 && gy < sizeY && gz >= 0 && gz < sizeZ;
                        bool material = inside && isMaterial(gx, gy, gz);
                        bool seed = seedMaterial ? material : !material;
                        work[wx, wy, wz] = seed ? 0f : HugeValue;
                    }
                }
            }
        }

        private sealed class TransformBuffers
        {
            public readonly float[] F;
            public readonly float[] D;
            public readonly int[] V;
            public readonly float[] Z;

            public TransformBuffers(int size)
            {
                F = new float[size];
                D = new float[size];
                V = new int[size];
                Z = new float[size + 1];
            }
        }

        private static void DistanceTransform3D(float[,,] work, int nx, int ny, int nz)
        {
            int maxDim = Math.Max(nx, Math.Max(ny, nz));

            // X pass: each (j, k) line is independent, so lines can run in parallel.
            Parallel.For(0, nz, () => new TransformBuffers(maxDim), (k, _, buffers) =>
            {
                float[] f = buffers.F, d = buffers.D;
                for (int j = 0; j < ny; j++)
                {
                    for (int i = 0; i < nx; i++) f[i] = work[i, j, k];
                    DistanceTransform1D(f, d, nx, buffers.V, buffers.Z);
                    for (int i = 0; i < nx; i++) work[i, j, k] = d[i];
                }
                return buffers;
            }, _ => { });

            // Y pass: each (i, k) line is independent.
            Parallel.For(0, nz, () => new TransformBuffers(maxDim), (k, _, buffers) =>
            {
                float[] f = buffers.F, d = buffers.D;
                for (int i = 0; i < nx; i++)
                {
                    for (int j = 0; j < ny; j++) f[j] = work[i, j, k];
                    DistanceTransform1D(f, d, ny, buffers.V, buffers.Z);
                    for (int j = 0; j < ny; j++) work[i, j, k] = d[j];
                }
                return buffers;
            }, _ => { });

            // Z pass: each (i, j) line is independent.
            Parallel.For(0, ny, () => new TransformBuffers(maxDim), (j, _, buffers) =>
            {
                float[] f = buffers.F, d = buffers.D;
                for (int i = 0; i < nx; i++)
                {
                    for (int k = 0; k < nz; k++) f[k] = work[i, j, k];
                    DistanceTransform1D(f, d, nz, buffers.V, buffers.Z);
                    for (int k = 0; k < nz; k++) work[i, j, k] = d[k];
                }
                return buffers;
            }, _ => { });
        }

        private static void DistanceTransform1D(float[] f, float[] d, int n, int[] v, float[] z)
        {
            int k = 0;
            v[0] = 0;
            z[0] = float.NegativeInfinity;
            z[1] = float.PositiveInfinity;

            for (int q = 1; q < n; q++)
            {
                float s = ((f[q] + (float)q * q) - (f[v[k]] + (float)v[k] * v[k])) / (2 * q - 2 * v[k]);
                while (s <= z[k])
                {
                    k--;
                    s = ((f[q] + (float)q * q) - (f[v[k]] + (float)v[k] * v[k])) / (2 * q - 2 * v[k]);
                }
                k++;
                v[k] = q;
                z[k] = s;
                z[k + 1] = float.PositiveInfinity;
            }

            k = 0;
            for (int q = 0; q < n; q++)
            {
                while (z[k + 1] < q) k++;
                float dq = q - v[k];
                d[q] = dq * dq + f[v[k]];
            }
        }
    }
}
