using System;
using System.Collections.Generic;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Dual Contouring implementation for mesh generation from SDF.
    /// This method preserves sharp features better than Marching Cubes.
    /// </summary>
    internal static class DualContouring
    {
        // Edge table: which edges have sign changes for each cube configuration
        // Each bit position represents an edge (0-11)
        private static readonly int[] EdgeTable = new int[256];
        
        // Edge connections (same as Marching Cubes)
        private static readonly int[][] EdgeConnections = new int[12][]
        {
            new int[] {0, 1}, new int[] {1, 2}, new int[] {2, 3}, new int[] {3, 0},
            new int[] {4, 5}, new int[] {5, 6}, new int[] {6, 7}, new int[] {7, 4},
            new int[] {0, 4}, new int[] {1, 5}, new int[] {2, 6}, new int[] {3, 7}
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
            public Vector3 Position;
            public Vector3 Normal;
            public bool IsValid;
        }

        /// <summary>
        /// Find the optimal vertex position inside a cell using QEF (Quadratic Error Function) minimization
        /// </summary>
        private static Vector3 SolveQEF(List<Vector3> edgePoints, List<Vector3> edgeNormals, Vector3 cellCenter)
        {
            if (edgePoints.Count == 0)
                return cellCenter;

            // Simplified QEF: use mass point (average of edge intersection points)
            // For production, implement proper SVD-based QEF solver
            Vector3 massPoint = Vector3.Zero;
            foreach (var p in edgePoints)
            {
                massPoint += p;
            }
            massPoint /= edgePoints.Count;

            // Clamp to cell bounds to avoid extreme positions
            // This is a simplified approach; proper implementation would solve the least squares problem
            return massPoint;
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

            // Build cube index
            // NOTE: Our SDF convention: negative = empty (removed), positive = material
            // Standard DC expects: negative = material, positive = empty
            // So we invert the sign check: positive values set the bit
            int cubeIndex = 0;
            for (int i = 0; i < 8; i++)
            {
                if (cornerVals[i] > 0) cubeIndex |= (1 << i);
            }

            // Skip if all inside or all outside
            if (cubeIndex == 0 || cubeIndex == 255)
                return result;

            // Find edge intersections
            var edgePoints = new List<Vector3>();
            var edgeNormals = new List<Vector3>();

            int edgeMask = EdgeTable[cubeIndex];
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
                    Vector3 edgeNormal = sdf.GetGradient(edgePoint); // Gradient points outward from surface

                    edgePoints.Add(edgePoint);
                    edgeNormals.Add(edgeNormal);
                }
            }

            if (edgePoints.Count == 0)
                return result;

            // Solve for optimal vertex position
            Vector3 cellCenter = basePos + new Vector3(resolution * 0.5f);
            result.Position = SolveQEF(edgePoints, edgeNormals, cellCenter);

            // Compute average normal
            Vector3 normalSum = Vector3.Zero;
            foreach (var n in edgeNormals)
            {
                // Skip invalid normals (NaN or zero length)
                if (float.IsNaN(n.X) || float.IsNaN(n.Y) || float.IsNaN(n.Z))
                    continue;
                if (n.LengthSquared() < 1e-8f)
                    continue;
                    
                normalSum += n;
            }
            
            // Ensure we have a valid normal
            if (normalSum.LengthSquared() < 1e-8f)
            {
                // Fallback: use direction from cell center to vertex position
                Vector3 fallbackNormal = result.Position - cellCenter;
                if (fallbackNormal.LengthSquared() > 1e-8f)
                    result.Normal = Vector3.Normalize(fallbackNormal);
                else
                    result.Normal = Vector3.UnitY; // Last resort default
            }
            else
            {
                result.Normal = Vector3.Normalize(normalSum);
            }
            
            result.IsValid = true;

            return result;
        }

        /// <summary>
        /// Generate quad (two triangles) between four cell vertices
        /// </summary>
        private static void EmitQuad(
            CellVertex v0, CellVertex v1, CellVertex v2, CellVertex v3,
            List<Vector3> vertices, List<Vector3> normals, List<int> indices)
        {
            // Check all vertices are valid
            if (!v0.IsValid || !v1.IsValid || !v2.IsValid || !v3.IsValid)
                return;

            int baseIdx = vertices.Count;
            vertices.Add(v0.Position);
            vertices.Add(v1.Position);
            vertices.Add(v2.Position);
            vertices.Add(v3.Position);

            // Compute face normal from geometry as fallback
            Vector3 fallbackEdge1 = v1.Position - v0.Position;
            Vector3 fallbackEdge2 = v2.Position - v0.Position;
            Vector3 faceNormalFromGeometry = Vector3.Cross(fallbackEdge1, fallbackEdge2);
            if (faceNormalFromGeometry.LengthSquared() > 1e-8f)
                faceNormalFromGeometry = Vector3.Normalize(faceNormalFromGeometry);
            else
                faceNormalFromGeometry = Vector3.UnitY;

            // Add normals with validation - use face normal as fallback
            normals.Add(IsValidNormal(v0.Normal) ? v0.Normal : faceNormalFromGeometry);
            normals.Add(IsValidNormal(v1.Normal) ? v1.Normal : faceNormalFromGeometry);
            normals.Add(IsValidNormal(v2.Normal) ? v2.Normal : faceNormalFromGeometry);
            normals.Add(IsValidNormal(v3.Normal) ? v3.Normal : faceNormalFromGeometry);

            // Compute face normal to determine winding order
            Vector3 edge1 = v1.Position - v0.Position;
            Vector3 edge2 = v2.Position - v0.Position;
            Vector3 faceNormal = Vector3.Cross(edge1, edge2);
            
            if (faceNormal.LengthSquared() < 1e-8f)
                return; // Degenerate quad
                
            faceNormal = Vector3.Normalize(faceNormal);
            Vector3 avgNormal = (v0.Normal + v1.Normal + v2.Normal + v3.Normal) / 4.0f;
            
            // Skip if average normal is invalid
            if (avgNormal.LengthSquared() < 1e-8f || 
                float.IsNaN(avgNormal.X) || float.IsNaN(avgNormal.Y) || float.IsNaN(avgNormal.Z))
                return;
                
            avgNormal = Vector3.Normalize(avgNormal);
            
            // Check if winding matches normal direction
            bool flipWinding = Vector3.Dot(faceNormal, avgNormal) < 0;
            
            if (flipWinding)
            {
                // Reversed winding
                indices.Add(baseIdx + 0);
                indices.Add(baseIdx + 2);
                indices.Add(baseIdx + 1);
                
                indices.Add(baseIdx + 0);
                indices.Add(baseIdx + 3);
                indices.Add(baseIdx + 2);
            }
            else
            {
                // Normal winding
                indices.Add(baseIdx + 0);
                indices.Add(baseIdx + 1);
                indices.Add(baseIdx + 2);
                
                indices.Add(baseIdx + 0);
                indices.Add(baseIdx + 2);
                indices.Add(baseIdx + 3);
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
            // Size is +2 to accommodate cells from -1 to sizeX-1 (need space for sizeX+1 when generating quads)
            var cellVertices = new CellVertex[sizeX + 2, sizeY + 2, sizeZ + 2];
            
            System.Threading.Tasks.Parallel.For(-1, sizeZ + 1, z =>
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
            var normals = new List<Vector3>();
            var indices = new List<int>();

            // Generate quads between adjacent cells
            // Include boundary cells to generate outer shell
            // EmitQuad will filter out quads with invalid vertices automatically
            for (int z = -1; z < sizeZ; z++)
            {
                for (int y = -1; y < sizeY; y++)
                {
                    for (int x = -1; x < sizeX; x++)
                    {
                        // Convert to array indices (add 1 to handle -1 offset)
                        int xi = x + 1, yi = y + 1, zi = z + 1;
                        
                        // X-aligned edge: cells (x,y,z), (x,y+1,z), (x,y+1,z+1), (x,y,z+1)
                        // Creates face in YZ plane
                        EmitQuad(
                            cellVertices[xi, yi, zi],
                            cellVertices[xi, yi + 1, zi],
                            cellVertices[xi, yi + 1, zi + 1],
                            cellVertices[xi, yi, zi + 1],
                            vertices, normals, indices);

                        // Y-aligned edge: cells (x,y,z), (x,y,z+1), (x+1,y,z+1), (x+1,y,z)
                        // Creates face in XZ plane
                        EmitQuad(
                            cellVertices[xi, yi, zi],
                            cellVertices[xi, yi, zi + 1],
                            cellVertices[xi + 1, yi, zi + 1],
                            cellVertices[xi + 1, yi, zi],
                            vertices, normals, indices);

                        // Z-aligned edge: cells (x,y,z), (x+1,y,z), (x+1,y+1,z), (x,y+1,z)
                        // Creates face in XY plane
                        EmitQuad(
                            cellVertices[xi, yi, zi],
                            cellVertices[xi + 1, yi, zi],
                            cellVertices[xi + 1, yi + 1, zi],
                            cellVertices[xi, yi + 1, zi],
                            vertices, normals, indices);
                    }
                }
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
