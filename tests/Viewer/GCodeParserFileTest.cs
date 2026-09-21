using System;
using System.IO;
using System.Linq;
using System.Numerics;
using MillSimSharp.Toolpath;
using MillSimSharp.Viewer;
using NUnit.Framework;

namespace MillSimSharp.Tests.Viewer
{
    /// <summary>
    /// Tests that the viewer G-code files parse correctly with the core parser.
    /// </summary>
    [TestFixture]
    public class GCodeParserFileTest
    {
        private static string ViewerGcodePath(string name)
        {
            return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "gcodes", name);
        }

        [Test]
        public void ParseFile_TestNc_ProducesExpectedCommands()
        {
            string path = ViewerGcodePath("test.nc");
            Assert.That(File.Exists(path), Is.True, "gcodes/test.nc must be copied to the test output");

            var commands = GCodeParser.ParseFile(path, Vector3.Zero);

            Assert.That(commands.Count, Is.EqualTo(12));
            Assert.That(commands[0], Is.InstanceOf<G0Move>());
            Assert.That(((G0Move)commands[0]).Target, Is.EqualTo(new Vector3(0, 0, 150)));

            // Modal feed rate is applied to following G1 moves
            Assert.That(commands[2], Is.InstanceOf<G1Move>());
            Assert.That(((G1Move)commands[2]).FeedRate, Is.EqualTo(300f).Within(1e-4f));
            Assert.That(((G1Move)commands[3]).FeedRate, Is.EqualTo(300f).Within(1e-4f));

            var last = (G1Move)commands[commands.Count - 1];
            Assert.That(last.Target, Is.EqualTo(new Vector3(-70, -70, 150)));
        }

        [Test]
        public void ParseFile_Test2Nc_ProducesManyMoves()
        {
            string path = ViewerGcodePath("test2.nc");
            Assert.That(File.Exists(path), Is.True, "gcodes/test2.nc must be copied to the test output");

            var commands = GCodeParser.ParseFile(path, Vector3.Zero);

            Assert.That(commands.Count, Is.GreaterThan(1000));
            Assert.That(commands.All(c => c is G0Move || c is G1Move), Is.True,
                "The demo file only contains G0/G1 moves");
        }
    }
}
