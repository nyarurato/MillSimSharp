using System;
using System.IO;
using System.Numerics;
using System.Xml.Serialization;
using MillSimSharp.Geometry;

namespace MillSimSharp.Config
{
    /// <summary>
    /// Defines where the WorkOrigin point is located on the stock.
    /// </summary>
    public enum StockOriginType
    {
        /// <summary>
        /// WorkOrigin is at the minimum corner (X-, Y-, Z-) of the stock.
        /// Stock extends in positive X, Y, Z directions.
        /// </summary>
        MinCorner,

        /// <summary>
        /// WorkOrigin is at the center of the stock.
        /// Stock extends equally in all directions.
        /// </summary>
        Center
    }

    /// <summary>
    /// Stock configuration loaded from XML.
    /// </summary>
    [XmlRoot("stock")]
    public class StockConfiguration
    {
        /// <summary>
        /// Work origin (reference point on stock).
        /// </summary>
        [XmlElement("WorkOrigin")]
        public Vector3Data WorkOrigin { get; set; }

        /// <summary>
        /// Work size (dimensions of stock).
        /// </summary>
        [XmlElement("WorkSize")]
        public Vector3Data WorkSize { get; set; }

        /// <summary>
        /// Defines where the WorkOrigin point is located on the stock.
        /// Default is MinCorner (legacy behavior).
        /// </summary>
        [XmlElement("OriginType")]
        public StockOriginType OriginType { get; set; } = StockOriginType.MinCorner;

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

            if (OriginType == StockOriginType.Center)
            {
                // Origin is at center, create bounding box from center and size
                return BoundingBox.FromCenterAndSize(origin, size);
            }
            else // MinCorner
            {
                // Origin is at minimum corner, create bounding box from min to max
                return new BoundingBox(origin, origin + size);
            }
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
