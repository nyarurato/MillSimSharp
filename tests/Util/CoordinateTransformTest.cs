using NUnit.Framework;
using MillSimSharp.Util;
using MillSimSharp.Geometry;
using System.Numerics;

namespace MillSimSharp.Tests.Util
{
    [TestFixture]
    public class CoordinateTransformTest
    {
        [Test]
        public void TestMachineToWork()
        {
            var machineCoords = new Vector3(100, 50, 25);
            var workOrigin = new Vector3(50, 50, 0);

            var workCoords = CoordinateTransform.MachineToWork(machineCoords, workOrigin);

            Assert.That(workCoords, Is.EqualTo(new Vector3(50, 0, 25)));
        }

        [Test]
        public void TestWorkToMachine()
        {
            var workCoords = new Vector3(50, 0, 25);
            var workOrigin = new Vector3(50, 50, 0);

            var machineCoords = CoordinateTransform.WorkToMachine(workCoords, workOrigin);

            Assert.That(machineCoords, Is.EqualTo(new Vector3(100, 50, 25)));
        }

        [Test]
        public void TestRoundTrip()
        {
            var original = new Vector3(123, 456, 789);
            var workOrigin = new Vector3(100, 200, 300);

            var work = CoordinateTransform.MachineToWork(original, workOrigin);
            var back = CoordinateTransform.WorkToMachine(work, workOrigin);

            Assert.That(back, Is.EqualTo(original));
        }

        [Test]
        public void TestTranslateBoundingBox()
        {
            var bbox = new BoundingBox(new Vector3(0, 0, 0), new Vector3(10, 10, 10));
            var offset = new Vector3(5, 5, 5);

            var translated = CoordinateTransform.TranslateBoundingBox(bbox, offset);

            Assert.That(translated.Min, Is.EqualTo(new Vector3(5, 5, 5)));
            Assert.That(translated.Max, Is.EqualTo(new Vector3(15, 15, 15)));
        }
    }
}
