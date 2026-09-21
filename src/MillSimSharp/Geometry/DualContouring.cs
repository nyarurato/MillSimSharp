using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Dual Contouring implementation for mesh generation from SDF.
    /// Cell vertices are positioned with a normal-weighted quadratic error function (QEF) and
    /// quads are emitted only for grid edges with a sign change.
    /// </summary>
    internal static class DualContouring
    {
        // Edge table: which edges have sign changes for each cube configuration
        // Each bit position represents an edge (0-11)
        private static readonly int[] EdgeTable = new int[256];

        // Cube edge connections using the corner numbering of ComputeCellVertex:
        // corner i has offsets (bit0 = x, bit1 = y, bit2 = z):
        //   0 = (0,0,0), 1 = (1,0,0), 2 = (0,1,0), 3 = (1,1,0),
        //   4 = (0,0,1), 5 = (1,0,1), 6 = (0,1,1), 7 = (1,1,1)
        // Every entry must connect two corners that differ in exactly one component (a real cube
        // edge); face diagonals would put the QEF sample points off the cell edges.
        private static readonly int[][] EdgeConnections = new int[12][]
        {
            new int[] {0, 1}, new int[] {2, 3}, new int[] {4, 5}, new int[] {6, 7}, // X-aligned
            new int[] {0, 2}, new int[] {1, 3}, new int[] {4, 6}, new int[] {5, 7}, // Y-aligned
            new int[] {0, 4}, new int[] {1, 5}, new int[] {2, 6}, new int[] {3, 7}  // Z-aligned
        };

        static DualContouring()
        {
            // Initialize edge table (simplified - any edge with sign change is marked)
            for (int i = 0; i < 256; i++)
            {
                int edgeMask = 0;
                for (int e = 0; e < 12; e++)
                {
                    int v0 = EdgeConnections[e][0];
                    int v1 = EdgeConnections[e][1];
                    bool b0 = (i & (1 << v0)) != 0;
                    bool b1 = (i & (1 << v1)) != 0;
                    if (b0 != b1)
                    {
                        edgeMask |= (1 << e);
                    }
                }
                EdgeTable[i] = edgeMask;
            }
        }

        /// <summary>
        /// Check if a normal vector is valid (not NaN and has length)
        /// </summary>
        private static bool IsValidNormal(Vector3 normal)
        {
            return !float.IsNaN(normal.X) && !float.IsNaN(normal.Y) && !float.IsNaN(normal.Z)
                   && normal.LengthSquared() > 1e-8f;
        }

        /// <summary>
        /// Data for a cell vertex in Dual Contouring
        /// </summary>
        public struct CellVertex
        {
            /// <summary>Cell vertex position.</summary>
            public Vector3 Position;
            /// <summary>Cell vertex normal (averaged gradient).</summary>
            public Vector3 Normal;
            /// <summary>True when the cell crosses the surface.</summary>
            public bool IsValid;
            /// <summary>Bit i is set when corner i is material (negative SDF).</summary>
            public int MaterialMask;
        }

        /// <summary>
        /// Find the optimal vertex position inside a cell using a normal-weighted QEF,
        /// clamped to the cell bounds.
        /// </summary>
        private static Vector3 SolveQEF(List<Vector3> edgePoints, List<Vector3> edgeNormals, Vector3 cellCenter, float resolution)
        {
            // Mass point fallback (average of the edge intersections)
            Vector3 massPoint = Vector3.Zero;
            foreach (var p in edgePoints)
            {
                massPoint += p;
            }
            massPoint /= edgePoints.Count;

            // Accumulate A = sum(n n^T) and b = sum(n (n . p))
            float a00 = 0, a01 = 0, a02 = 0, a11 = 0, a12 = 0, a22 = 0;
            float b0 = 0, b1 = 0, b2 = 0;

            for (int i = 0; i < edgePoints.Count; i++)
            {
                Vector3 n = edgeNormals[i];
                Vector3 p = edgePoints[i];
                float nd = Vector3.Dot(n, p);

                a00 += n.X * n.X; a01 += n.X * n.Y; a02 += n.X * n.Z;
                a11 += n.Y * n.Y; a12 += n.Y * n.Z; a22 += n.Z * n.Z;

                b0 += n.X * nd; b1 += n.Y * nd; b2 += n.Z * nd;
            }

            // Solve the regularized QEF around the mass point:
            //   (A + lambda I) * delta = b - A * massPoint,  x = massPoint + delta
            // This keeps the unconstrained (tangential) directions at the mass point when A is
            // rank-deficient, instead of collapsing them toward the coordinate origin.
            Vector3 aMass = new Vector3(
                a00 * massPoint.X + a01 * massPoint.Y + a02 * massPoint.Z,
                a01 * massPoint.X + a11 * massPoint.Y + a12 * massPoint.Z,
                a02 * massPoint.X + a12 * massPoint.Y + a22 * massPoint.Z);
            float r0 = b0 - aMass.X;
            float r1 = b1 - aMass.Y;
            float r2 = b2 - aMass.Z;

            // Regularization keeps the system solvable for planar configurations.
            float trace = a00 + a11 + a22;
            float lambda = MathF.Max(trace * 1e-3f, 1e-6f);
            a00 += lambda;
            a11 += lambda;
            a22 += lambda;

            float det = a00 * (a11 * a22 - a12 * a12)
                      - a01 * (a01 * a22 - a12 * a02)
                      + a02 * (a01 * a12 - a11 * a02);

            Vector3 solution;
            if (MathF.Abs(det) < 1e-12f)
            {
                solution = massPoint;
            }
            else
            {
                float detX = r0 * (a11 * a22 - a12 * a12)
                           - a01 * (r1 * a22 - a12 * r2)
                           + a02 * (r1 * a12 - a11 * r2);
                float detY = a00 * (r1 * a22 - a12 * r2)
                           - r0 * (a01 * a22 - a12 * a02)
                           + a02 * (a01 * r2 - r1 * a02);
                float detZ = a00 * (a11 * r2 - r1 * a12)
                           - a01 * (a01 * r2 - r1 * a02)
                           + r0 * (a01 * a12 - a11 * a02);
                var delta = new Vector3(detX / det, detY / det, detZ / det);

                if (float.IsNaN(delta.X) || float.IsNaN(delta.Y) || float.IsNaN(delta.Z))
                {
                    solution = massPoint;
                }
                else
                {
                    solution = massPoint + delta;
                }
            }

            // Clamp the vertex to the cell bounds. A small inset avoids collapsing vertices of
            // adjacent cells onto the shared corner (which would create non-manifold edges).
            float half = resolution * 0.45f;
            solution.X = Math.Clamp(solution.X, cellCenter.X - half, cellCenter.X + half);
            solution.Y = Math.Clamp(solution.Y, cellCenter.Y - half, cellCenter.Y + half);
            solution.Z = Math.Clamp(solution.Z, cellCenter.Z - half, cellCenter.Z + half);

            return solution;
        }

        /// <summary>
        /// Compute vertex position and normal for a cell
        /// </summary>
        public static CellVertex ComputeCellVertex(SDFGrid sdf, int x, int y, int z, float resolution)
        {
            var result = new CellVertex { IsValid = false };

            // Get cell base position
            Vector3 basePos = sdf.Bounds.Min + new Vector3(x * resolution, y * resolution, z * resolution);

            // Sample 8 corners
            float[] cornerVals = new float[8];
            Vector3[] cornerPos = new Vector3[8];
            for (int i = 0; i < 8; i++)
            {
                int dx = (i & 1);
                int dy = (i >> 1) & 1;
                int dz = (i >> 2) & 1;
                cornerPos[i] = basePos + new Vector3(dx * resolution, dy * resolution, dz * resolution);
                cornerVals[i] = sdf.GetDistance(cornerPos[i]);
            }

            // Build the material mask (negative = material under the standard SDF convention)
            int materialMask = 0;
            for (int i = 0; i < 8; i++)
            {
                if (cornerVals[i] < 0) materialMask |= (1 << i);
            }
            result.MaterialMask = materialMask;

            // Skip if all inside or all outside
            if (materialMask == 0 || materialMask == 255)
                return result;

            // Find edge intersections
            var edgePoints = new List<Vector3>();
            var edgeNormals = new List<Vector3>();

            int edgeMask = EdgeTable[materialMask];
            for (int e = 0; e < 12; e++)
            {
                if ((edgeMask & (1 << e)) != 0)
                {
                    int v0 = EdgeConnections[e][0];
                    int v1 = EdgeConnections[e][1];
                    float val0 = cornerVals[v0];
                    float val1 = cornerVals[v1];

                    // Interpolate zero crossing
                    float t = val0 / (val0 - val1);
                    Vector3 edgePoint = cornerPos[v0] + (cornerPos[v1] - cornerPos[v0]) * t;
                    Vector3 edgeNormal = sdf.GetGradient(edgePoint); // Gradient points outward from material

                    edgePoints.Add(edgePoint);
                    edgeNormals.Add(edgeNormal);
                }
            }

            if (edgePoints.Count == 0)
                return result;

            // Solve for optimal vertex position
            Vector3 cellCenter = basePos + new Vector3(resolution * 0.5f);
            result.Position = SolveQEF(edgePoints, edgeNormals, cellCenter, resolution);

            // Compute average normal
            Vector3 normalSum = Vector3.Zero;
            foreach (var n in edgeNormals)
            {
                if (!IsValidNormal(n)) continue;
                normalSum += n;
            }

            if (normalSum.LengthSquared() < 1e-8f)
            {
                // Fallback: use direction from cell center to vertex position
                Vector3 fallbackNormal = result.Position - cellCenter;
                if (fallbackNormal.LengthSquared() > 1e-8f)
                    result.Normal = Vector3.Normalize(fallbackNormal);
                else
                    result.Normal = Vector3.UnitY;
            }
            else
            {
                result.Normal = Vector3.Normalize(normalSum);
            }

            result.IsValid = true;

            return result;
        }

        private static bool HasEdgeCrossing(int materialMask, int cornerA, int cornerB)
        {
            bool a = (materialMask & (1 << cornerA)) != 0;
            bool b = (materialMask & (1 << cornerB)) != 0;
            return a != b;
        }

        /// <summary>
        /// Emits a quad (two triangles) between four cell vertices.
        /// <para>
        /// The winding is derived from the SDF sign change along the corresponding grid edge:
        /// when material (negative) is at the positive end of the edge, the natural quad winding
        /// points into the material and must be reversed. This is globally consistent and does not
        /// depend on potentially cancelling cell normals.
        /// </para>
        /// </summary>
        private static void EmitQuad(
            CellVertex v0, CellVertex v1, CellVertex v2, CellVertex v3,
            bool materialAtEdgeEnd,
            Func<Vector3, Vector3, int> addVertex, List<int> indices)
        {
            // Check all vertices are valid
            if (!v0.IsValid || !v1.IsValid || !v2.IsValid || !v3.IsValid)
                return;

            // Compute the quad normal for shading. After the winding decision below this is the
            // normal of the emitted triangles, so vertex normals always agree with the geometry.
            Vector3 edge1 = v1.Position - v0.Position;
            Vector3 edge2 = v2.Position - v0.Position;
            Vector3 faceNormal = Vector3.Cross(edge1, edge2);
            if (faceNormal.LengthSquared() > 1e-12f)
            {
                faceNormal = Vector3.Normalize(faceNormal);
            }
            else
            {
                faceNormal = IsValidNormal(v0.Normal) ? Vector3.Normalize(v0.Normal) : Vector3.UnitY;
            }

            if (materialAtEdgeEnd)
            {
                faceNormal = -faceNormal; // the emitted triangles are reversed
            }

            int i0 = addVertex(v0.Position, faceNormal);
            int i1 = addVertex(v1.Position, faceNormal);
            int i2 = addVertex(v2.Position, faceNormal);
            int i3 = addVertex(v3.Position, faceNormal);

            if (materialAtEdgeEnd)
            {
                AddTriangle(indices, i0, i2, i1);
                AddTriangle(indices, i0, i3, i2);
            }
            else
            {
                AddTriangle(indices, i0, i1, i2);
                AddTriangle(indices, i0, i2, i3);
            }
        }

        /// <summary>
        /// Adds a triangle, skipping triangles that collapsed due to merged vertices.
        /// </summary>
        private static void AddTriangle(List<int> indices, int a, int b, int c)
        {
            if (a == b || b == c || c == a) return;
            indices.Add(a);
            indices.Add(b);
            indices.Add(c);
        }

        private sealed class Vector3Comparer : IEqualityComparer<Vector3>
        {
            private const float Epsilon = 1e-5f;

            public bool Equals(Vector3 a, Vector3 b)
            {
                return Quantize(a) == Quantize(b);
            }

            public int GetHashCode(Vector3 v)
            {
                return Quantize(v).GetHashCode();
            }

            private static (long X, long Y, long Z) Quantize(Vector3 v)
            {
                return (
                    (long)MathF.Round(v.X / Epsilon),
                    (long)MathF.Round(v.Y / Epsilon),
                    (long)MathF.Round(v.Z / Epsilon));
            }
        }

        /// <summary>
        /// Generate mesh using Dual Contouring algorithm with boundary handling
        /// </summary>
        public static Mesh Generate(SDFGrid sdf)
        {
            var (sizeX, sizeY, sizeZ) = sdf.Dimensions;
            float res = sdf.Resolution;

            // Compute cell vertices for all cells including boundary cells at -1
            // This allows generating outer shell of the mesh
            var cellVertices = new CellVertex[sizeX + 2, sizeY + 2, sizeZ + 2];

            Parallel.For(-1, sizeZ + 1, z =>
            {
                for (int y = -1; y <= sizeY; y++)
                {
                    for (int x = -1; x <= sizeX; x++)
                    {
                        cellVertices[x + 1, y + 1, z + 1] = ComputeCellVertex(sdf, x, y, z, res);
                    }
                }
            });

            var vertices = new List<Vector3>();
            var normalSums = new List<Vector3>();
            var indices = new List<int>();
            var vertexMap = new Dictionary<Vector3, int>(new Vector3Comparer());

            int AddVertex(Vector3 position, Vector3 normal)
            {
                if (vertexMap.TryGetValue(position, out int existing))
                {
                    normalSums[existing] += normal;
                    return existing;
                }

                int index = vertices.Count;
                vertices.Add(position);
                normalSums.Add(normal);
                vertexMap[position] = index;
                return index;
            }

            // Generate quads for every grid edge with a sign change.
            // Each edge is visited exactly once via its source cell.
            for (int z = -1; z < sizeZ; z++)
            {
                for (int y = -1; y < sizeY; y++)
                {
                    for (int x = -1; x < sizeX; x++)
                    {
                        int xi = x + 1, yi = y + 1, zi = z + 1;
                        CellVertex center = cellVertices[xi, yi, zi];
                        if (!center.IsValid) continue;

                        // All three edges end at the cell's (+,+,+) corner, so the sign there
                        // decides whether the natural quad winding must be reversed.
                        bool materialAtEdgeEnd = (center.MaterialMask & (1 << 7)) != 0;

                        // X-aligned edge at the cell's (+y, +z) corner
                        if (HasEdgeCrossing(center.MaterialMask, 6, 7))
                        {
                            EmitQuad(
                                cellVertices[xi, yi, zi],
                                cellVertices[xi, yi + 1, zi],
                                cellVertices[xi, yi + 1, zi + 1],
                                cellVertices[xi, yi, zi + 1],
                                materialAtEdgeEnd, AddVertex, indices);
                        }

                        // Y-aligned edge at the cell's (+x, +z) corner
                        if (HasEdgeCrossing(center.MaterialMask, 5, 7))
                        {
                            EmitQuad(
                                cellVertices[xi, yi, zi],
                                cellVertices[xi, yi, zi + 1],
                                cellVertices[xi + 1, yi, zi + 1],
                                cellVertices[xi + 1, yi, zi],
                                materialAtEdgeEnd, AddVertex, indices);
                        }

                        // Z-aligned edge at the cell's (+x, +y) corner
                        if (HasEdgeCrossing(center.MaterialMask, 3, 7))
                        {
                            EmitQuad(
                                cellVertices[xi, yi, zi],
                                cellVertices[xi + 1, yi, zi],
                                cellVertices[xi + 1, yi + 1, zi],
                                cellVertices[xi, yi + 1, zi],
                                materialAtEdgeEnd, AddVertex, indices);
                        }
                    }
                }
            }

            // Project vertices onto the zero level set with a few bounded Newton steps.
            // This removes small placement errors and reduces tucked/folded quads at sharp features
            // such as the junction of two crossing cuts.
            float maxStep = res * 0.5f;
            for (int i = 0; i < vertices.Count; i++)
            {
                Vector3 p = vertices[i];
                for (int iter = 0; iter < 3; iter++)
                {
                    float d = sdf.GetDistance(p);
                    if (MathF.Abs(d) < 1e-4f) break;

                    Vector3 gradient = sdf.GetGradient(p);
                    if (gradient.LengthSquared() < 1e-8f) break;

                    Vector3 step = gradient * d;
                    float length = step.Length();
                    if (length > maxStep)
                        step *= maxStep / length;

                    p -= step;
                }
                vertices[i] = p;
            }

            // Recompute normals from the projected triangles so they always match the geometry.
            var normalSums2 = new Vector3[vertices.Count];
            for (int i = 0; i + 2 < indices.Count; i += 3)
            {
                int a = indices[i], b = indices[i + 1], c = indices[i + 2];
                Vector3 normal = Vector3.Cross(vertices[b] - vertices[a], vertices[c] - vertices[a]);
                if (normal.LengthSquared() < 1e-12f) continue;
                normalSums2[a] += normal;
                normalSums2[b] += normal;
                normalSums2[c] += normal;
            }

            var normals = new List<Vector3>(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
            {
                Vector3 sum = normalSums2[i];
                if (sum.LengthSquared() > 1e-12f)
                    normals.Add(Vector3.Normalize(sum));
                else
                    normals.Add(Vector3.UnitY);
            }

            return new Mesh
            {
                Vertices = vertices.ToArray(),
                Normals = normals.ToArray(),
                Indices = indices.ToArray()
            };
        }
    }
}
