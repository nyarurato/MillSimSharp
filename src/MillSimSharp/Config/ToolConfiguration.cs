using System;
using System.IO;
using System.Xml.Serialization;
using MillSimSharp.Simulation;

namespace MillSimSharp.Config
{
    /// <summary>
    /// Tool configuration loaded from XML.
    /// </summary>
    [XmlRoot("tool")]
    public class ToolConfiguration
    {
        /// <summary>
        /// Tool diameter in millimeters.
        /// </summary>
        [XmlElement("diameter")]
        public float Diameter { get; set; }

        /// <summary>
        /// Tool cutting length in millimeters.
        /// </summary>
        [XmlElement("length")]
        public float Length { get; set; }

        /// <summary>
        /// Whether the tool is a ball end mill.
        /// </summary>
        [XmlElement("isBallEnd")]
        public bool IsBallEnd { get; set; }

        /// <summary>
        /// Loads tool configuration from XML file.
        /// </summary>
        /// <param name="path">Path to XML file.</param>
        /// <returns>Tool configuration.</returns>
        public static ToolConfiguration LoadFromXml(string path)
        {
            var serializer = new XmlSerializer(typeof(ToolConfiguration));
            using (var reader = new StreamReader(path))
            {
                return (ToolConfiguration)serializer.Deserialize(reader);
            }
        }

        /// <summary>
        /// Creates a Tool instance from this configuration.
        /// </summary>
        /// <returns>EndMill instance.</returns>
        public Tool CreateTool()
        {
            return new EndMill(Diameter, Length, IsBallEnd);
        }
    }
}
