using System;
using System.Collections.Generic;
using System.Numerics;
using System.Reflection;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Contract tests for the mesh vertex comparers: <c>Equals(a, b)</c> must imply equal hash
    /// codes, otherwise dictionary-based vertex merging silently stops merging (or throws).
    /// The comparers are internal implementation details, so they are resolved by reflection.
    /// </summary>
    [TestFixture]
    public class MeshVertexComparerTest
    {
        private static IEqualityComparer<Vector3> CreateComparer(string typeName)
        {
            Assembly assembly = typeof(VoxelGrid).Assembly;
            Type? type = assembly.GetType(typeName);
            Assert.That(type, Is.Not.Null, $"{typeName} must exist");
            return (IEqualityComparer<Vector3>)Activator.CreateInstance(type!, nonPublic: true)!;
        }

        private static void AssertContract(IEqualityComparer<Vector3> comparer, float epsilon)
        {
            // Offsets that straddle quantization boundaries around multiples of epsilon.
            var candidates = new List<Vector3>();
            foreach (float factor in new[] { 0.4f, 0.9f, 1.1f, 1.6f, -0.4f, -0.9f, -1.1f, -1.6f })
            {
                candidates.Add(new Vector3(factor * epsilon, 0f, 0f));
                candidates.Add(new Vector3(0.3f * epsilon, factor * epsilon, -0.7f * epsilon));
            }

            foreach (Vector3 a in candidates)
            {
                Assert.That(comparer.Equals(a, a), Is.True, "A vertex must equal itself");

                foreach (Vector3 b in candidates)
                {
                    if (!comparer.Equals(a, b)) continue;

                    Assert.That(comparer.GetHashCode(a), Is.EqualTo(comparer.GetHashCode(b)),
                        $"Equals({a}, {b}) must imply the same hash code");
                }
            }

            // Points inside one quantization cell must still merge (no over-separation).
            Assert.That(
                comparer.Equals(
                    new Vector3(0.1f * epsilon, 0f, 0f),
                    new Vector3(0.4f * epsilon, 0f, 0f)),
                Is.True,
                "Near-identical coordinates must merge");

            var noisy = new Vector3(1f + 0.25f * epsilon, 2f, 3f);
            Assert.That(comparer.Equals(noisy, new Vector3(1f, 2f, 3f)), Is.True,
                "Sub-epsilon float noise must merge");
        }

        [Test]
        public void VoxelVertexComparer_EqualsImpliesSameHash()
        {
            AssertContract(CreateComparer("MillSimSharp.Geometry.VoxelVertexComparer"), 1e-3f);
        }

        [Test]
        public void Vector3Comparer_EqualsImpliesSameHash()
        {
            AssertContract(CreateComparer("MillSimSharp.Geometry.DualContouring+Vector3Comparer"), 1e-5f);
        }
    }
}
