using System;
using System.Collections.Generic;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    public static class MeshConverter
    {
        // Convert VoxelGrid into a Mesh using marching cubes
        public static Mesh ConvertToMesh(VoxelGrid grid)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));

            var triangles = new List<(Vector3 v1, Vector3 v2, Vector3 v3, Vector3 normal)>();
            var (sizeX, sizeY, sizeZ) = grid.Dimensions;

            for (int z = 0; z < sizeZ - 1; z++)
            {
                for (int y = 0; y < sizeY - 1; y++)
                {
                    for (int x = 0; x < sizeX - 1; x++)
                    {
                        // Reuse logic from StlExporter.ProcessCube
                        bool[] corners = new bool[8];

                        float res = grid.Resolution;
                        Vector3 basePos = grid.Bounds.Min + new Vector3(x * res, y * res, z * res);

                        Vector3[] cornerPositions = new Vector3[8];
                        cornerPositions[0] = basePos;
                        cornerPositions[1] = basePos + new Vector3(res, 0, 0);
                        cornerPositions[2] = basePos + new Vector3(res, 0, res);
                        cornerPositions[3] = basePos + new Vector3(0, 0, res);
                        cornerPositions[4] = basePos + new Vector3(0, res, 0);
                        cornerPositions[5] = basePos + new Vector3(res, res, 0);
                        cornerPositions[6] = basePos + new Vector3(res, res, res);
                        cornerPositions[7] = basePos + new Vector3(0, res, res);

                        // Compute corner positions (world coordinates) and sample occupancy at those points
                        Vector3[] edgeVertices = new Vector3[12];
                        for (int i = 0; i < 12; i++)
                        {
                            int v1 = MarchingCubesTable.EdgeConnections[i, 0];
                            int v2 = MarchingCubesTable.EdgeConnections[i, 1];
                            edgeVertices[i] = (cornerPositions[v1] + cornerPositions[v2]) * 0.5f;
                        }
                        // Now sample corner occupancy using world coordinates
                        for (int i = 0; i < 8; i++)
                        {
                            corners[i] = grid.GetVoxelAtWorld(cornerPositions[i]);
                        }

                        int cubeIndex = 0;
                        for (int i = 0; i < 8; i++) if (corners[i]) cubeIndex |= (1 << i);
                        if (cubeIndex == 0 || cubeIndex == 255) continue;

                        for (int i = 0; MarchingCubesTable.TriangleTable[cubeIndex, i] != -1; i += 3)
                        {
                            int e1 = MarchingCubesTable.TriangleTable[cubeIndex, i];
                            int e2 = MarchingCubesTable.TriangleTable[cubeIndex, i + 1];
                            int e3 = MarchingCubesTable.TriangleTable[cubeIndex, i + 2];

                            Vector3 v1 = edgeVertices[e1];
                            Vector3 v2 = edgeVertices[e2];
                            Vector3 v3 = edgeVertices[e3];

                                Vector3 n = Vector3.Normalize(Vector3.Cross(v2 - v1, v3 - v1));
                                triangles.Add((v1, v2, v3, n));
                        }
                    }
                }
            }

            // Convert triangle list into mesh (expand vertices per triangle)
            var verts = new List<Vector3>();
            var norms = new List<Vector3>();
            var inds = new List<int>();
            var center = grid.Bounds.Min + grid.Bounds.Size / 2.0f;
            for (int i = 0; i < triangles.Count; i++)
            {
                var t = triangles[i];
                // Ensure triangle winding faces outward by sampling occupancy on both sides of triangle centroid.
                // This is more robust than only using the object's center for shapes with cavities or complex topology.
                var centroid = (t.v1 + t.v2 + t.v3) / 3.0f;
                Vector3 v1 = t.v1, v2 = t.v2, v3 = t.v3;
                Vector3 n = t.normal;

                // Sample distances along the normal; choose epsilon to move to adjacent voxel (75% of voxel size)
                float eps = Math.Max(1e-4f, grid.Resolution * 0.75f);
                var sampleInside = false;
                var sampleOutside = false;
                if (n.Length() > 1e-6f)
                {
                    sampleInside = grid.GetVoxelAtWorld(centroid + n * eps);
                    sampleOutside = grid.GetVoxelAtWorld(centroid - n * eps);
                }

                if (sampleInside && !sampleOutside)
                {
                    // The normal points into material; flip to get outward-facing normal
                    var tmp = v2; v2 = v3; v3 = tmp;
                    n = -n;
                }
                else if (sampleInside == sampleOutside)
                {
                    // Ambiguous; fall back to centroid-vs-center dot test
                    var dir = Vector3.Normalize(centroid - center);
                    var dot = Vector3.Dot(t.normal, dir);
                    if (dot < 0)
                    {
                        var tmp = v2; v2 = v3; v3 = tmp;
                        n = -n;
                    }
                }

                verts.Add(v1); norms.Add(n); inds.Add(i * 3 + 0);
                verts.Add(v2); norms.Add(n); inds.Add(i * 3 + 1);
                verts.Add(v3); norms.Add(n); inds.Add(i * 3 + 2);
            }

            // Add outer boundary faces for material voxels adjacent to empty space so the workpiece shell is visible
            for (int z = 0; z < sizeZ; z++)
            {
                for (int y = 0; y < sizeY; y++)
                {
                    for (int x = 0; x < sizeX; x++)
                    {
                        if (!grid.GetVoxel(x, y, z)) continue;

                        // center of this voxel
                        var centerVoxel = grid.Bounds.Min + new Vector3((x + 0.5f) * grid.Resolution, (y + 0.5f) * grid.Resolution, (z + 0.5f) * grid.Resolution);
                        float half = grid.Resolution * 0.5f;

                        // Directions and the corresponding face corner offsets
                        var faces = new (System.Func<int, int, int, bool> neighborCheck, Vector3[] corners, Vector3 outward)[ ]
                        {
                            ( (xi, yi, zi) => grid.GetVoxel(xi - 1, yi, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, -half, -half),
                                    centerVoxel + new Vector3(-half, +half, -half),
                                    centerVoxel + new Vector3(-half, +half, +half),
                                    centerVoxel + new Vector3(-half, -half, +half)
                                }, new Vector3(-1,0,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi + 1, yi, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(+half, -half, +half),
                                    centerVoxel + new Vector3(+half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, -half),
                                    centerVoxel + new Vector3(+half, -half, -half)
                                }, new Vector3(+1,0,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi - 1, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, -half, +half),
                                    centerVoxel + new Vector3(-half, -half, -half),
                                    centerVoxel + new Vector3(+half, -half, -half),
                                    centerVoxel + new Vector3(+half, -half, +half)
                                }, new Vector3(0,-1,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi + 1, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, +half, -half),
                                    centerVoxel + new Vector3(-half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, -half)
                                }, new Vector3(0,+1,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi, zi - 1), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, +half, -half),
                                    centerVoxel + new Vector3(-half, -half, -half),
                                    centerVoxel + new Vector3(+half, -half, -half),
                                    centerVoxel + new Vector3(+half, +half, -half)
                                }, new Vector3(0,0,-1) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi, zi + 1), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, -half, +half),
                                    centerVoxel + new Vector3(-half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, +half),
                                    centerVoxel + new Vector3(+half, -half, +half)
                                }, new Vector3(0,0,1) )
                        };

                        foreach (var face in faces)
                        {
                            bool neighborHasMaterial = false;
                            try
                            {
                                neighborHasMaterial = face.neighborCheck(x, y, z);
                            }
                            catch { neighborHasMaterial = false; }
                            if (neighborHasMaterial) continue; // neighbor is material, not an outer face

                            // add two triangles for this quad face
                            var fv = face.corners;
                            var tri1n = Vector3.Normalize(Vector3.Cross(fv[1] - fv[0], fv[2] - fv[0]));
                            var tri2n = Vector3.Normalize(Vector3.Cross(fv[2] - fv[0], fv[3] - fv[0]));
                            var triOut = face.outward;
                            // Ensure orientation is outward by flipping if necessary
                            if (Vector3.Dot(tri1n, triOut) < 0)
                            {
                                // flip winding for both triangles
                                verts.Add(fv[0]); norms.Add(-tri1n); inds.Add(inds.Count);
                                verts.Add(fv[2]); norms.Add(-tri1n); inds.Add(inds.Count);
                                verts.Add(fv[1]); norms.Add(-tri1n); inds.Add(inds.Count);

                                verts.Add(fv[0]); norms.Add(-tri2n); inds.Add(inds.Count);
                                verts.Add(fv[3]); norms.Add(-tri2n); inds.Add(inds.Count);
                                verts.Add(fv[2]); norms.Add(-tri2n); inds.Add(inds.Count);
                            }
                            else
                            {
                                verts.Add(fv[0]); norms.Add(tri1n); inds.Add(inds.Count);
                                verts.Add(fv[1]); norms.Add(tri1n); inds.Add(inds.Count);
                                verts.Add(fv[2]); norms.Add(tri1n); inds.Add(inds.Count);

                                verts.Add(fv[0]); norms.Add(tri2n); inds.Add(inds.Count);
                                verts.Add(fv[2]); norms.Add(tri2n); inds.Add(inds.Count);
                                verts.Add(fv[3]); norms.Add(tri2n); inds.Add(inds.Count);
                            }
                        }
                    }
                }
            }

            return new Mesh()
            {
                Vertices = verts.ToArray(),
                Normals = norms.ToArray(),
                Indices = inds.ToArray()
            };
        }
    }
}
