using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Tests for shank/holder collision detection against remaining stock material.
    /// </summary>
    [TestFixture]
    public class ToolCollisionTest
    {
        private static BoundingBox StockBounds => BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));

        [Test]
        public void EndMill_ShankIsNotModelledByDefault()
        {
            Assert.That(new EndMill(10f, 30f, isBallEnd: false).GetShankGeometry(), Is.Null);
        }

        [Test]
        public void Shank_IntersectsMaterialAboveClearedBore()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            // Clear a bore from the bottom up to z = 0
            grid.RemoveVoxelsInCylinder(new Vector3(0, 0, -10), new Vector3(0, 0, 0), 3f, flatEnds: true);

            // Shank (radius 2, length 10) with its base at z = -5: the upper half is in untouched material
            var shank = new FlatEndMillGeometry(2f, 10f);
            bool collides = ToolCollisionDetector.IntersectsMaterial(grid, shank, new Vector3(0, 0, -5), Vector3.UnitZ);

            Assert.That(collides, Is.True, "The shank must collide with material above the cleared bore");
        }

        [Test]
        public void Shank_OutsideStock_NoCollision()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            var shank = new FlatEndMillGeometry(2f, 10f);

            // Shank fully above the stock (stock top is at z = 10)
            bool collides = ToolCollisionDetector.IntersectsMaterial(grid, shank, new Vector3(0, 0, 10), Vector3.UnitZ);

            Assert.That(collides, Is.False);
        }

        [Test]
        public void SdfShank_IntersectsMaterialAboveClearedBore()
        {
            var sdf = new SDFGrid(StockBounds, 1.0f, narrowBandWidth: 10);
            sdf.RemoveFiniteCylinder(new Vector3(0, 0, -10), new Vector3(0, 0, 0), 3f);

            var shank = new FlatEndMillGeometry(2f, 10f);
            bool collides = ToolCollisionDetector.IntersectsMaterial(sdf, shank, new Vector3(0, 0, -5), Vector3.UnitZ);

            Assert.That(collides, Is.True, "The shank must collide with material above the cleared bore");
        }

        [Test]
        public void SdfShank_OutsideStock_NoCollision()
        {
            var sdf = new SDFGrid(StockBounds, 1.0f, narrowBandWidth: 10);
            var shank = new FlatEndMillGeometry(2f, 10f);

            bool collides = ToolCollisionDetector.IntersectsMaterial(sdf, shank, new Vector3(0, 0, 10), Vector3.UnitZ);

            Assert.That(collides, Is.False);
        }
    }
}
