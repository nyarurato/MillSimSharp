using NUnit.Framework;
using System.IO;
using MillSimSharp.Viewer;
using MillSimSharp.Toolpath;
using System.Numerics;
using System.Linq;
using gs;

namespace MillSimSharp.Tests.Viewer
{
    [TestFixture]
    public class GcodeToPathTest
    {
        [Test]
        public void TestParseGcodeFileToToolpathCommands()
        {
            string gcode = "G0 Z10\nG0 X0 Y0\nG1 Z-5 F300\nG1 X10 Y10\nG1 Z10\n";
            var parser = new GenericGCodeParser();
            using (var sr = new StringReader(gcode))
            {
                var file = parser.Parse(sr);
                var commands = GcodeToPath.Parse(file, new Vector3(0, 0, 0));

                // We expect 5 commands
                Assert.That(commands.Count, Is.EqualTo(5));

                // First two should be G0 moves (rapid)
                Assert.That(commands[0], Is.InstanceOf<G0Move>());
                Assert.That(((G0Move)commands[0]).Target, Is.EqualTo(new Vector3(0, 0, 10)));

                Assert.That(commands[1], Is.InstanceOf<G0Move>());
                // this move should keep Z=10
                Assert.That(((G0Move)commands[1]).Target, Is.EqualTo(new Vector3(0, 0, 10)));

                // Third should be G1 move to Z=-5
                Assert.That(commands[2], Is.InstanceOf<G1Move>());
                Assert.That(((G1Move)commands[2]).Target, Is.EqualTo(new Vector3(0, 0, -5)));

                // Fourth G1 to X10 Y10 (Z remains -5)
                Assert.That(commands[3], Is.InstanceOf<G1Move>());
                Assert.That(((G1Move)commands[3]).Target, Is.EqualTo(new Vector3(10, 10, -5)));

                // Fifth G1 to Z=10
                Assert.That(commands[4], Is.InstanceOf<G1Move>());
                Assert.That(((G1Move)commands[4]).Target, Is.EqualTo(new Vector3(10, 10, 10)));
            }
        }
    }
}
