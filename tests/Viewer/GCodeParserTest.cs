using System;
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
    }
}
