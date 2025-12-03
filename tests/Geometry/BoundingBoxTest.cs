using NUnit.Framework;
using MillSimSharp.Geometry;
using System.Numerics;

namespace MillSimSharp.Tests.Geometry
{
    [TestFixture]
    public class BoundingBoxTest
    {
        [Test]
        public void TestConstruction()
        {
            var min = new Vector3(0, 0, 0);
            var max = new Vector3(10, 10, 10);
            var bbox = new BoundingBox(min, max);

            Assert.That(bbox.Min, Is.EqualTo(min));
            Assert.That(bbox.Max, Is.EqualTo(max));
        }

        [Test]
        public void TestInvalidConstruction()
        {
            var min = new Vector3(10, 10, 10);
            var max = new Vector3(0, 0, 0);
            Assert.Throws<ArgumentException>(() => new BoundingBox(min, max));
        }

        [Test]
        public void TestSize()
        {
            var min = new Vector3(0, 0, 0);
            var max = new Vector3(10, 20, 30);
            var bbox = new BoundingBox(min, max);

            Assert.That(bbox.Size, Is.EqualTo(new Vector3(10, 20, 30)));
        }

        [Test]
        public void TestCenter()
        {
            var min = new Vector3(0, 0, 0);
            var max = new Vector3(10, 20, 30);
            var bbox = new BoundingBox(min, max);

            Assert.That(bbox.Center, Is.EqualTo(new Vector3(5, 10, 15)));
        }

        [Test]
        public void TestFromCenterAndSize()
        {
            var center = new Vector3(5, 10, 15);
            var size = new Vector3(10, 20, 30);
            var bbox = BoundingBox.FromCenterAndSize(center, size);

            Assert.That(bbox.Min, Is.EqualTo(new Vector3(0, 0, 0)));
            Assert.That(bbox.Max, Is.EqualTo(new Vector3(10, 20, 30)));
        }

        [Test]
        public void TestContains()
        {
            var bbox = new BoundingBox(new Vector3(0, 0, 0), new Vector3(10, 10, 10));

            Assert.That(bbox.Contains(new Vector3(5, 5, 5)), Is.True);
            Assert.That(bbox.Contains(new Vector3(0, 0, 0)), Is.True);
            Assert.That(bbox.Contains(new Vector3(10, 10, 10)), Is.True);
            Assert.That(bbox.Contains(new Vector3(-1, 5, 5)), Is.False);
            Assert.That(bbox.Contains(new Vector3(11, 5, 5)), Is.False);
        }

        [Test]
        public void TestExpandToInclude()
        {
            var bbox = new BoundingBox(new Vector3(0, 0, 0), new Vector3(10, 10, 10));
            var expanded = bbox.ExpandToInclude(new Vector3(15, 5, 5));

            Assert.That(expanded.Min, Is.EqualTo(new Vector3(0, 0, 0)));
            Assert.That(expanded.Max, Is.EqualTo(new Vector3(15, 10, 10)));
        }
    }
}
