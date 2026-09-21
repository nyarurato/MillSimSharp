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

        // ---------------------------------------------------------------------
        // V4: axis vector contract
        // ---------------------------------------------------------------------

        [Test]
        public void IntersectsMaterial_ScaledAxis_MatchesUnitAxis()
        {
            // Material exists only near the top of the shank (z in [6, 9]), so scaling the axis to
            // (0,0,2) would push every center out of the solid unless the axis is normalized.
            var bbox = new BoundingBox(new Vector3(-2, -2, 6), new Vector3(2, 2, 9));
            var grid = new VoxelGrid(bbox, 1.0f);
            var shank = new FlatEndMillGeometry(2f, 10f);

            bool unit = ToolCollisionDetector.IntersectsMaterial(grid, shank, Vector3.Zero, Vector3.UnitZ);
            bool scaled = ToolCollisionDetector.IntersectsMaterial(grid, shank, Vector3.Zero, new Vector3(0, 0, 2));

            Assert.That(unit, Is.True);
            Assert.That(scaled, Is.EqualTo(unit),
                "A finite non-zero axis must behave like its normalized direction");
        }

        [Test]
        public void IntersectsMaterial_ZeroOrNonFiniteAxis_Throws()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            var shank = new FlatEndMillGeometry(2f, 10f);

            Assert.Throws<ArgumentException>(() =>
                ToolCollisionDetector.IntersectsMaterial(grid, shank, Vector3.Zero, Vector3.Zero));
            Assert.Throws<ArgumentException>(() =>
                ToolCollisionDetector.IntersectsMaterial(grid, shank, Vector3.Zero, new Vector3(float.NaN, 0, 1)));
            Assert.Throws<ArgumentException>(() =>
                ToolCollisionDetector.IntersectsMaterial(grid, shank, Vector3.Zero, new Vector3(0, 0, float.PositiveInfinity)));
        }

        // ---------------------------------------------------------------------
        // V5: LocalBounds.Min.Z contract
        // ---------------------------------------------------------------------

        [Test]
        public void IntersectsMaterial_GeometryExtendingBelowTip_IsDetected()
        {
            // A test-only solid spans local z in [-5, +5]; the stock material exists only below z = -1.
            var bbox = new BoundingBox(new Vector3(-2, -2, -4), new Vector3(2, 2, -1));
            var grid = new VoxelGrid(bbox, 1.0f);

            var geometry = new TestSpanningCapsuleGeometry(radius: 1f, halfLength: 5f);
            bool collides = ToolCollisionDetector.IntersectsMaterial(grid, geometry, Vector3.Zero, Vector3.UnitZ);

            Assert.That(collides, Is.True,
                "World bounds must cover negative local Z (solid extends below the physical tip)");
        }

        // ---------------------------------------------------------------------
        // V6: center-sampling limitation (documented behavior)
        // ---------------------------------------------------------------------

        [Test]
        public void Collision_SubResolutionTool_IsCenterSampled()
        {
            // Known limitation: only material voxel centers are tested. A 0.2mm tool at a voxel
            // corner is continuously inside the stock but misses every center.
            var grid = new VoxelGrid(StockBounds, 1.0f);
            var tiny = new FlatEndMillGeometry(0.1f, 5f);

            bool collides = ToolCollisionDetector.IntersectsMaterial(grid, tiny, Vector3.Zero, Vector3.UnitZ);

            Assert.That(grid.GetVoxelAtWorld(new Vector3(0.5f, 0.5f, 0.5f)), Is.True);
            Assert.That(collides, Is.False,
                "Known limitation: sub-resolution tools passing through a voxel corner are not detected");
        }

        [Test]
        public void Collision_CornerGrazingTool_IsCenterSampled()
        {
            // Radius 0.5 at the corner (0,0,0): the nearest voxel centers are 0.707mm away.
            var grid = new VoxelGrid(StockBounds, 1.0f);
            var graze = new FlatEndMillGeometry(0.5f, 4f);

            bool collides = ToolCollisionDetector.IntersectsMaterial(grid, graze, Vector3.Zero, Vector3.UnitZ);

            Assert.That(collides, Is.False,
                "Known limitation: a tool that only cuts voxel corners/edges is not detected by center sampling");
        }
    }
}
