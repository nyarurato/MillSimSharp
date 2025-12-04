using NUnit.Framework;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using System.Numerics;

namespace MillSimSharp.Tests.Simulation
{
    [TestFixture]
    public class CutterSimulatorTest
    {
        private VoxelGrid _grid;
        private CutterSimulator _simulator;
        private BoundingBox _bbox;

        [SetUp]
        public void Setup()
        {
            // 10x10x10mm grid, 1mm resolution
            _bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            _grid = new VoxelGrid(_bbox, resolution: 1.0f);
            _simulator = new CutterSimulator(_grid);
        }

        [Test]
        public void TestCutLinear_FlatEndMill()
        {
            // Tool: 2mm diameter (1mm radius)
            var tool = new EndMill(2.0f, 10.0f, isBallEnd: false);
            
            // Cut from (-2, 0, 0) to (2, 0, 0)
            var start = new Vector3(-2, 0, 0);
            var end = new Vector3(2, 0, 0);
            
            _simulator.CutLinear(start, end, tool);

            // Check center (should be removed)
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 0, 0)), Is.False);

            // Check start point (should be removed)
            Assert.That(_grid.GetVoxelAtWorld(start), Is.False);

            // Check near end point (should be removed)
            // Note: The voxel containing (2,0,0) has center at (2.5, 0.5, 0.5), which is outside the cylinder length (4.0).
            // So we check a point slightly inside the cut.
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(1.9f, 0, 0)), Is.False);

            // Check slightly beyond end point - in the current implementation the shaft
            // is removed as the tool passes, which includes vertical radius above the tip
            // so this voxel is now expected to be removed.
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(2.5f, 0, 0)), Is.False);
        }

        [Test]
        public void TestCutLinear_BallEndMill()
        {
            // Tool: 2mm diameter (1mm radius)
            var tool = new EndMill(2.0f, 10.0f, isBallEnd: true);
            
            // Cut from (-2, 0, 0) to (2, 0, 0)
            var start = new Vector3(-2, 0, 0);
            var end = new Vector3(2, 0, 0);
            
            _simulator.CutLinear(start, end, tool);

            // Check center (should be removed)
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 0, 0)), Is.False);

            // Check slightly beyond end point (should be removed for ball end)
            // End is at x=2. Radius is 1.
            // Point at x=2.5 is within radius (0.5 distance from end).
            // Should be removed.
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(2.5f, 0, 0)), Is.False);
            
            // Point at x=3.5 (1.5 distance) should be solid
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(3.5f, 0, 0)), Is.True);
        }
    }
}
