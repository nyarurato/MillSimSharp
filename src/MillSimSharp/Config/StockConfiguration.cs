using System;
using System.IO;
using System.Numerics;
using System.Xml.Serialization;
using MillSimSharp.Geometry;

namespace MillSimSharp.Config
{
    /// <summary>
    /// Stock configuration loaded from XML.
    /// </summary>
    [XmlRoot("stock")]
    public class StockConfiguration
    {
        /// <summary>
        /// Work origin (center or corner of stock).
        /// </summary>
        [XmlElement("WorkOrigin")]
        public Vector3Data WorkOrigin { get; set; }

        /// <summary>
        /// Work size (dimensions of stock).
        /// </summary>
        [XmlElement("WorkSize")]
        public Vector3Data WorkSize { get; set; }

        /// <summary>
        /// Loads stock configuration from XML file.
        /// </summary>
        /// <param name="path">Path to XML file.</param>
        /// <returns>Stock configuration.</returns>
        public static StockConfiguration LoadFromXml(string path)
        {
            var serializer = new XmlSerializer(typeof(StockConfiguration));
            using (var reader = new StreamReader(path))
            {
                return (StockConfiguration)serializer.Deserialize(reader);
            }
        }

        /// <summary>
        /// Creates a BoundingBox from this configuration.
        /// </summary>
        /// <returns>BoundingBox representing the stock.</returns>
        public BoundingBox GetBoundingBox()
        {
            var origin = WorkOrigin.ToVector3();
            var size = WorkSize.ToVector3();
            return BoundingBox.FromCenterAndSize(origin, size);
        }
    }

    /// <summary>
    /// Helper class for XML serialization of Vector3.
    /// </summary>
    public class Vector3Data
    {
        [XmlElement("X")]
        public float X { get; set; }

        [XmlElement("Y")]
        public float Y { get; set; }

        [XmlElement("Z")]
        public float Z { get; set; }

        public Vector3Data()
        {
        }

        public Vector3Data(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public Vector3 ToVector3()
        {
            return new Vector3(X, Y, Z);
        }
    }
}
