using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using MillSimSharp.Toolpath;
using MillSimSharp.Viewer;
using NUnit.Framework;

namespace MillSimSharp.Tests.Viewer
{
    /// <summary>
    /// Tests for the core G-code parser.
    /// </summary>
    [TestFixture]
    public class GCodeParserTest
    {
        [Test]
        public void Parse_StraightMoves_AndFeed()
        {
            var commands = GCodeParser.ParseText("G0 Z10\nG0 X0 Y0\nG1 Z0 F300\nG1 X10 Y0\n", Vector3.Zero);

            Assert.That(commands.Count, Is.EqualTo(4));
            Assert.That(commands[0], Is.InstanceOf<G0Move>());
            Assert.That(((G0Move)commands[0]).Target, Is.EqualTo(new Vector3(0, 0, 10)));

            Assert.That(commands[3], Is.InstanceOf<G1Move>());
            Assert.That(((G1Move)commands[3]).Target, Is.EqualTo(new Vector3(10, 0, 0)));
            Assert.That(((G1Move)commands[3]).FeedRate, Is.EqualTo(300f).Within(1e-4f));
        }

        [Test]
        public void Parse_InchUnits_ConvertsToMillimeters()
        {
            var commands = GCodeParser.ParseText("G20\nG1 X1 Y0.5 F10\n", Vector3.Zero);

            var move = (G1Move)commands[commands.Count - 1];
            Assert.That(move.Target.X, Is.EqualTo(25.4f).Within(1e-3f));
            Assert.That(move.Target.Y, Is.EqualTo(12.7f).Within(1e-3f));
        }

        [Test]
        public void Parse_IncrementalMode_AccumulatesPosition()
        {
            var commands = GCodeParser.ParseText("G91\nG1 X1 Y2\nG1 X1 Z1\n", Vector3.Zero);

            Assert.That(((G1Move)commands[0]).Target, Is.EqualTo(new Vector3(1, 2, 0)));
            Assert.That(((G1Move)commands[1]).Target, Is.EqualTo(new Vector3(2, 2, 1)));
        }

        [Test]
        public void Parse_Arc_EmitsChords()
        {
            // CCW half circle from (0,0) to (10,0) with center (5,0)
            var commands = GCodeParser.ParseText("G1 X0 Y0 F100\nG3 X10 Y0 I5 J0\n", Vector3.Zero, arcSegmentAngleDegrees: 5f);

            Assert.That(commands.Count, Is.GreaterThan(10));

            var last = (G1Move)commands[commands.Count - 1];
            Assert.That(last.Target.X, Is.EqualTo(10f).Within(1e-3f));
            Assert.That(last.Target.Y, Is.EqualTo(0f).Within(1e-3f));

            bool foundMidpoint = commands.OfType<G1Move>().Any(
                m => Math.Abs(m.Target.X - 5f) < 0.5f && Math.Abs(m.Target.Y + 5f) < 0.5f);
            Assert.That(foundMidpoint, Is.True, "The arc must pass through (5,-5)");
        }

        [Test]
        public void Parse_CommentsAndBlankLines_AreIgnored()
        {
            var commands = GCodeParser.ParseText("; header\n\nG1 X5 (inline comment)\n", Vector3.Zero);

            Assert.That(commands.Count, Is.EqualTo(1));
            Assert.That(((G1Move)commands[0]).Target, Is.EqualTo(new Vector3(5, 0, 0)));
        }

        // ---------------------------------------------------------------------
        // R-format arcs: positive R = minor arc, negative R = major arc
        // ---------------------------------------------------------------------

        private static List<Vector3> ParseArcTargets(string gcode, Vector3 start, float segmentAngleDegrees = 1f)
        {
            return GCodeParser.ParseText(gcode, start, segmentAngleDegrees)
                .OfType<G1Move>()
                .Select(m => m.Target)
                .ToList();
        }

        private static bool HasPointNear(List<Vector3> points, float x, float y, float tolerance = 0.05f)
        {
            return points.Any(p => Math.Abs(p.X - x) <= tolerance && Math.Abs(p.Y - y) <= tolerance);
        }

        [Test]
        public void Parse_G3_PositiveR_MinorArc()
        {
            // CCW minor arc from (1,0) to (0,1) is centered at (0,0).
            var points = ParseArcTargets("G3 X0 Y1 R1\n", new Vector3(1, 0, 0));

            Assert.That(points, Is.Not.Empty);
            Assert.That(points[^1].X, Is.EqualTo(0f).Within(1e-3f));
            Assert.That(points[^1].Y, Is.EqualTo(1f).Within(1e-3f));
            Assert.That(HasPointNear(points, 0.7071f, 0.7071f), Is.True,
                "The minor arc must pass through (0.707, 0.707)");
            Assert.That(HasPointNear(points, 1.7071f, 1.7071f), Is.False,
                "The minor arc must not take the major route");
        }

        [Test]
        public void Parse_G2_PositiveR_MinorArc()
        {
            // CW minor arc from (0,1) to (1,0) is centered at (0,0).
            var points = ParseArcTargets("G2 X1 Y0 R1\n", new Vector3(0, 1, 0));

            Assert.That(points, Is.Not.Empty);
            Assert.That(points[^1].X, Is.EqualTo(1f).Within(1e-3f));
            Assert.That(points[^1].Y, Is.EqualTo(0f).Within(1e-3f));
            Assert.That(HasPointNear(points, 0.7071f, 0.7071f), Is.True,
                "The minor arc must pass through (0.707, 0.707)");
            Assert.That(HasPointNear(points, 1.7071f, 1.7071f), Is.False,
                "The minor arc must not take the major route");
        }

        [Test]
        public void Parse_G3_NegativeR_MajorArc()
        {
            // Negative R: CCW major arc from (1,0) to (0,1), centered at (1,1).
            var points = ParseArcTargets("G3 X0 Y1 R-1\n", new Vector3(1, 0, 0));

            Assert.That(points, Is.Not.Empty);
            Assert.That(points[^1].X, Is.EqualTo(0f).Within(1e-3f));
            Assert.That(points[^1].Y, Is.EqualTo(1f).Within(1e-3f));
            Assert.That(HasPointNear(points, 1.7071f, 1.7071f), Is.True,
                "The major arc must pass through (1.707, 1.707)");
        }

        [Test]
        public void Parse_G2_NegativeR_MajorArc()
        {
            // Negative R: CW major arc from (0,1) to (1,0), centered at (1,1).
            var points = ParseArcTargets("G2 X1 Y0 R-1\n", new Vector3(0, 1, 0));

            Assert.That(points, Is.Not.Empty);
            Assert.That(points[^1].X, Is.EqualTo(1f).Within(1e-3f));
            Assert.That(points[^1].Y, Is.EqualTo(0f).Within(1e-3f));
            Assert.That(HasPointNear(points, 1.7071f, 1.7071f), Is.True,
                "The major arc must pass through (1.707, 1.707)");
        }

        [Test]
        public void Parse_ImpossibleRadius_IsHandledClearly()
        {
            // |R| < chord / 2: the line must not be turned into a circle.
            var commands = GCodeParser.ParseText("G3 X1 Y0 R0.4\n", Vector3.Zero);
            Assert.That(commands, Is.Empty, "An impossible radius must not emit moves");

            // The ignored line must not move the tool.
            var relative = GCodeParser.ParseText("G91\nG3 X1 Y0 R0.4\nG1 X1 Y0 F100\n", Vector3.Zero);
            Assert.That(relative.Count, Is.EqualTo(1));
            Assert.That(((G1Move)relative[0]).Target, Is.EqualTo(new Vector3(1, 0, 0)),
                "The ignored arc must leave the position unchanged");
        }

        [Test]
        public void Parse_ArcWithoutCenter_IsNotTreatedAsG1()
        {
            var commands = GCodeParser.ParseText("G3 X10 Y0\n", Vector3.Zero);
            Assert.That(commands, Is.Empty, "An arc without I/J/R must not be emitted as a linear move");

            var relative = GCodeParser.ParseText("G91\nG3 X10 Y0\nG1 X1 Y0 F100\n", Vector3.Zero);
            Assert.That(relative.Count, Is.EqualTo(1));
            Assert.That(((G1Move)relative[0]).Target, Is.EqualTo(new Vector3(1, 0, 0)),
                "The ignored arc must leave the position unchanged");
        }

        [Test]
        public void Parse_ModalWords_OrderIndependent()
        {
            // G91 before or after the axis word must produce the same target.
            var relativeFirst = GCodeParser.ParseText("G90\nG1 X5\nG91 G1 X1\n", Vector3.Zero);
            var relativeLast = GCodeParser.ParseText("G90\nG1 X5\nX1 G91 G1\n", Vector3.Zero);

            Assert.That(((G1Move)relativeLast[^1]).Target, Is.EqualTo(new Vector3(6, 0, 0)));
            Assert.That(((G1Move)relativeLast[^1]).Target, Is.EqualTo(((G1Move)relativeFirst[^1]).Target),
                "Relative mode must apply regardless of the word order in the block");

            // G20 before or after the axis word must produce the same (inch-scaled) target.
            var inchFirst = GCodeParser.ParseText("G21\nG20 G1 X1\n", Vector3.Zero);
            var inchLast = GCodeParser.ParseText("G21\nX1 G20 G1\n", Vector3.Zero);

            Assert.That(((G1Move)inchLast[^1]).Target.X, Is.EqualTo(25.4f).Within(1e-3f));
            Assert.That(((G1Move)inchLast[^1]).Target.X, Is.EqualTo(((G1Move)inchFirst[^1]).Target.X).Within(1e-3f),
                "Unit mode must apply regardless of the word order in the block");
        }

        [Test]
        public void Parse_ArcMode_IsModal()
        {
            // The second line continues the G3 arc without repeating the G code.
            var points = ParseArcTargets("G3 X0 Y1 R1\nX-1 Y0 R1\n", new Vector3(1, 0, 0));

            Assert.That(points, Is.Not.Empty);
            Assert.That(points[^1].X, Is.EqualTo(-1f).Within(1e-3f));
            Assert.That(points[^1].Y, Is.EqualTo(0f).Within(1e-3f));
            Assert.That(HasPointNear(points, -0.7071f, 0.7071f), Is.True,
                "The second arc (modal G3 from (0,1) to (-1,0)) must pass through (-0.707, 0.707)");
        }
    }
}
