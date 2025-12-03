using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Serialization;
using MillSimSharp.Geometry;

namespace MillSimSharp.Config
{
    /// <summary>
    /// Machine configuration loaded from XML.
    /// </summary>
    [XmlRoot("machine")]
    public class MachineConfiguration
    {
        /// <summary>
        /// Machine name.
        /// </summary>
        [XmlElement("name")]
        public string Name { get; set; }

        /// <summary>
        /// List of machine axes.
        /// </summary>
        [XmlArray("axes")]
        [XmlArrayItem("axis")]
        public List<AxisConfig> Axes { get; set; }

        /// <summary>
        /// Loads machine configuration from XML file.
        /// </summary>
        /// <param name="path">Path to XML file.</param>
        /// <returns>Machine configuration.</returns>
        public static MachineConfiguration LoadFromXml(string path)
        {
            var serializer = new XmlSerializer(typeof(MachineConfiguration));
            using (var reader = new StreamReader(path))
            {
                return (MachineConfiguration)serializer.Deserialize(reader);
            }
        }

        /// <summary>
        /// Gets the work volume as a BoundingBox.
        /// </summary>
        /// <returns>BoundingBox representing machine limits.</returns>
        public BoundingBox GetWorkVolume()
        {
            if (Axes == null || Axes.Count < 3)
                throw new InvalidOperationException("Machine must have at least 3 axes (X, Y, Z).");

            var xAxis = Axes.FirstOrDefault(a => a.Name == "X");
            var yAxis = Axes.FirstOrDefault(a => a.Name == "Y");
            var zAxis = Axes.FirstOrDefault(a => a.Name == "Z");

            if (xAxis == null || yAxis == null || zAxis == null)
                throw new InvalidOperationException("Machine must have X, Y, and Z axes.");

            var min = new System.Numerics.Vector3(xAxis.Min, yAxis.Min, zAxis.Min);
            var max = new System.Numerics.Vector3(xAxis.Max, yAxis.Max, zAxis.Max);

            return new BoundingBox(min, max);
        }
    }

    /// <summary>
    /// Represents a single machine axis configuration.
    /// </summary>
    public class AxisConfig
    {
        /// <summary>
        /// Axis name (e.g., "X", "Y", "Z").
        /// </summary>
        [XmlElement("name")]
        public string Name { get; set; }

        /// <summary>
        /// Minimum axis position in millimeters.
        /// </summary>
        [XmlElement("min")]
        public float Min { get; set; }

        /// <summary>
        /// Maximum axis position in millimeters.
        /// </summary>
        [XmlElement("max")]
        public float Max { get; set; }
    }
}
