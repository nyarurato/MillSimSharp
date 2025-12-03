using NUnit.Framework;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using System.Collections.Generic;
using System.Numerics;

namespace MillSimSharp.Tests.Toolpath
{
    [TestFixture]
    public class ToolpathCommandTest
    {
        private VoxelGrid _grid;
        private CutterSimulator _simulator;
        private EndMill _tool;
        private BoundingBox _bbox;

        [SetUp]
        public void Setup()
        {
            // 20x20x20mm grid, 1mm resolution
            _bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            _grid = new VoxelGrid(_bbox, resolution: 1.0f);
            _simulator = new CutterSimulator(_grid);
            _tool = new EndMill(2.0f, 10.0f, isBallEnd: false);
        }

        [Test]
        public void TestG0Move_UpdatesPosition()
        {
            var start = new Vector3(0, 0, 0);
            var target = new Vector3(5, 5, 5);
            var command = new G0Move(target);

            command.Execute(_simulator, _tool, ref start);

            // Position should be updated
            Assert.That(start, Is.EqualTo(target));

            // No material should be removed (grid should still be solid everywhere)
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(2.5f, 2.5f, 2.5f)), Is.True);
        }

        [Test]
        public void TestG1Move_RemovesMaterial()
        {
            var start = new Vector3(-3, 0, 0);
            var target = new Vector3(3, 0, 0);
            var command = new G1Move(target);

            command.Execute(_simulator, _tool, ref start);

            // Position should be updated
            Assert.That(start, Is.EqualTo(target));

            // Material should be removed along the path
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 0, 0)), Is.False);
        }

        [Test]
        public void TestToolpathExecutor_SimpleSequence()
        {
            var executor = new ToolpathExecutor(_simulator, _tool, new Vector3(0, 0, 5));

            var commands = new List<IToolpathCommand>
            {
                new G0Move(new Vector3(-3, 0, 5)),  // Rapid to start position (above workpiece)
                new G0Move(new Vector3(-3, 0, 0)),  // Rapid down to cutting depth
                new G1Move(new Vector3(3, 0, 0)),   // Cut across
                new G0Move(new Vector3(3, 0, 5))    // Rapid up to safe Z
            };

            executor.ExecuteCommands(commands);

            // Final position should be at end of sequence
            Assert.That(executor.CurrentPosition, Is.EqualTo(new Vector3(3, 0, 5)));

            // Material should be removed along the cutting path (G1)
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 0, 0)), Is.False);

            // Material should NOT be removed along rapid moves (G0)
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 0, 5)), Is.True);
        }

        [Test]
        public void TestToolpathExecutor_MultipleCuts()
        {
            var executor = new ToolpathExecutor(_simulator, _tool, new Vector3(0, 0, 5));

            var commands = new List<IToolpathCommand>
            {
                // First cut
                new G0Move(new Vector3(-3, -2, 0)),
                new G1Move(new Vector3(3, -2, 0)),
                
                // Second cut
                new G0Move(new Vector3(-3, 2, 0)),
                new G1Move(new Vector3(3, 2, 0))
            };

            executor.ExecuteCommands(commands);

            // Both cuts should have removed material
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, -2, 0)), Is.False);
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 2, 0)), Is.False);

            // Material between cuts should still exist
            Assert.That(_grid.GetVoxelAtWorld(new Vector3(0, 0, 0)), Is.True);
        }
    }
}
