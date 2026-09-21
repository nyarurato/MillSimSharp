using NUnit.Framework;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using System.Numerics;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Tests for the tool reference point specification:
    /// the physical tool tip is canonical, and ball end mill centers are derived from it.
    /// </summary>
    [TestFixture]
    public class ToolPoseTest
    {
        private static (VoxelGrid grid, CutterSimulator simulator) CreateVoxelSetup()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, 1.0f);
            return (grid, new CutterSimulator(grid));
        }

        [Test]
        public void BallCenterOffsetFromTip_IsRadiusForBallOnly()
        {
            var ball = new EndMill(diameter: 10f, length: 50f, isBallEnd: true);
            var flat = new EndMill(diameter: 10f, length: 50f, isBallEnd: false);

            Assert.That(ball.BallCenterOffsetFromTip, Is.EqualTo(5f));
            Assert.That(flat.BallCenterOffsetFromTip, Is.EqualTo(0f));
        }

        [Test]
        public void ToolOrientation_AxisDirections_AreOpposite()
        {
            var def = ToolOrientation.Default;
            var cutting = def.GetCuttingAxisDirection();
            Assert.That(cutting.X, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(cutting.Y, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(cutting.Z, Is.EqualTo(-1f).Within(1e-5f));

            var towardSpindle = def.GetAxisTowardSpindle();
            Assert.That(towardSpindle.Z, Is.EqualTo(1f).Within(1e-5f));

            var tilt = new ToolOrientation(a_deg: 90);
            var tiltedCutting = tilt.GetCuttingAxisDirection();
            Assert.That(tiltedCutting.X, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(tiltedCutting.Y, Is.EqualTo(1f).Within(1e-5f));
            Assert.That(tiltedCutting.Z, Is.EqualTo(0f).Within(1e-5f));

            var tiltedToward = tilt.GetAxisTowardSpindle();
            Assert.That(tiltedToward.Y, Is.EqualTo(-1f).Within(1e-5f));
        }

        [Test]
        public void ToolPose_CuttingCenter_DerivesBallCenter()
        {
            var pose = new ToolPose(new Vector3(1, 2, 3), new ToolOrientation(a_deg: 90));
            var ball = new EndMill(10f, 50f, isBallEnd: true);
            var flat = new EndMill(10f, 50f, isBallEnd: false);

            // Position is always the physical tip
            Assert.That(pose.Position, Is.EqualTo(new Vector3(1, 2, 3)));
            Assert.That(pose.GetCuttingCenter(flat), Is.EqualTo(new Vector3(1, 2, 3)));

            // For A=90 the axis toward the spindle is (0,-1,0)
            var ballCenter = pose.GetCuttingCenter(ball);
            Assert.That(ballCenter.X, Is.EqualTo(1f).Within(1e-5f));
            Assert.That(ballCenter.Y, Is.EqualTo(-3f).Within(1e-5f));
            Assert.That(ballCenter.Z, Is.EqualTo(3f).Within(1e-5f));
        }

        [Test]
        public void BallEndMill_CutPoint_DoesNotGougeBelowTip()
        {
            var (grid, simulator) = CreateVoxelSetup();
            var ball = new EndMill(diameter: 10f, length: 30f, isBallEnd: true);

            simulator.CutPoint(Vector3.Zero, ball);

            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, -1f)), Is.True,
                "A ball end mill must not remove material below its physical tip");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, 1f)), Is.False,
                "The ball must remove material above the tip");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(4.5f, 0, 5f)), Is.False,
                "Material inside the ball equator must be removed");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(5.5f, 0, 5f)), Is.True,
                "Material outside the ball radius must remain");
        }

        [Test]
        public void BallEndMill_CutLinear_DoesNotGougeBelowTip()
        {
            var (grid, simulator) = CreateVoxelSetup();
            var ball = new EndMill(diameter: 10f, length: 30f, isBallEnd: true);

            simulator.CutLinear(new Vector3(-5, 0, 0), new Vector3(5, 0, 0), ball);

            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, -1f)), Is.True,
                "Ball sweep must not remove material below the tip path");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, 1f)), Is.False,
                "Ball sweep must remove material inside the ball");
        }

        [Test]
        public void FlatEndMill_CutPoint_HasFlatBottom()
        {
            var (grid, simulator) = CreateVoxelSetup();
            var flat = new EndMill(diameter: 10f, length: 30f, isBallEnd: false);

            simulator.CutPoint(Vector3.Zero, flat);

            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, -1f)), Is.True,
                "A flat end mill must not remove material below its tip plane");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(0, 0, 1f)), Is.False,
                "The flat bottom must remove material above the tip");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(4.5f, 0, 0.5f)), Is.False,
                "Material inside the tool radius must be removed");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(5.5f, 0, 0.5f)), Is.True,
                "Material outside the tool radius must remain");
        }

        [Test]
        public void SDFBallEndMill_CutPoint_DoesNotGougeBelowTip()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var sdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 10);
            var simulator = new SDFCutterSimulator(sdf);
            var ball = new EndMill(diameter: 10f, length: 30f, isBallEnd: true);

            simulator.CutPoint(Vector3.Zero, ball);

            Assert.That(sdf.GetDistance(new Vector3(0, 0, -2f)), Is.LessThan(0f),
                "Material below the physical tip must remain (negative = material)");
            Assert.That(sdf.GetDistance(new Vector3(0, 0, 2f)), Is.GreaterThan(0f),
                "The ball interior must be carved (positive = empty)");
        }

        [Test]
        public void OrientedBallEndMill_DoesNotGougeBeyondTip()
        {
            var (grid, simulator) = CreateVoxelSetup();
            var ball = new EndMill(diameter: 10f, length: 30f, isBallEnd: true);
            var tilt = new ToolOrientation(a_deg: 90); // cutting axis +Y, body toward -Y

            simulator.CutLinearWithOrientation(Vector3.Zero, new Vector3(10, 0, 0), ball, tilt, tilt);

            // (5,1,0) is beyond the tip along the cutting axis (+Y) -> must remain material
            Assert.That(grid.GetVoxelAtWorld(new Vector3(5, 1, 0)), Is.True,
                "Material beyond the tip along the cutting axis must remain");
            // (5,-2,0) is inside the ball (center at (5,-5,0), radius 5) -> removed
            Assert.That(grid.GetVoxelAtWorld(new Vector3(5, -2, 0)), Is.False,
                "Material inside the tilted ball must be removed");
        }

        [Test]
        public void OrientedFlatEndMill_RemovesAlongToolBodyOnly()
        {
            var (grid, simulator) = CreateVoxelSetup();
            var flat = new EndMill(diameter: 10f, length: 30f, isBallEnd: false);
            var tilt = new ToolOrientation(a_deg: 90); // cutting axis +Y, body toward -Y

            simulator.CutLinearWithOrientation(Vector3.Zero, new Vector3(10, 0, 0), flat, tilt, tilt);

            Assert.That(grid.GetVoxelAtWorld(new Vector3(5, -1, 3)), Is.False,
                "Material inside the tilted flat cylinder must be removed");
            Assert.That(grid.GetVoxelAtWorld(new Vector3(5, 1, 0)), Is.True,
                "Material beyond the tip along the cutting axis must remain");
        }
    }
}
