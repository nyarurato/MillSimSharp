using System;
using System.IO;
using MillSimSharp.Config;

namespace MillSimSharp.Tests.Config
{
    /// <summary>
    /// Utility class for loading configurations from XML files.
    /// </summary>
    public static class ConfigurationLoader
    {
        /// <summary>
        /// Default path to tool configuration.
        /// </summary>
        public const string DefaultToolConfigPath = "configs/default_tool.xml";

        /// <summary>
        /// Default path to machine configuration.
        /// </summary>
        public const string DefaultMachineConfigPath = "configs/default_machine.xml";

        /// <summary>
        /// Default path to stock configuration.
        /// </summary>
        public const string DefaultStockConfigPath = "configs/default_stock.xml";

        /// <summary>
        /// Loads tool configuration from XML file.
        /// </summary>
        /// <param name="path">Path to XML file (default: configs/default_tool.xml).</param>
        /// <returns>Tool configuration.</returns>
        public static ToolConfiguration LoadToolConfig(string? path = null)
        {
            path = path ?? DefaultToolConfigPath;
            if (!File.Exists(path))
                throw new FileNotFoundException($"Tool configuration not found at: {Path.GetFullPath(path)}");

            return ToolConfiguration.LoadFromXml(path);
        }

        /// <summary>
        /// Loads machine configuration from XML file.
        /// </summary>
        /// <param name="path">Path to XML file (default: configs/default_machine.xml).</param>
        /// <returns>Machine configuration.</returns>
        public static MachineConfiguration LoadMachineConfig(string? path = null)
        {
            path = path ?? DefaultMachineConfigPath;
            if (!File.Exists(path))
                throw new FileNotFoundException($"Machine configuration not found at: {Path.GetFullPath(path)}");

            return MachineConfiguration.LoadFromXml(path);
        }

        /// <summary>
        /// Loads stock configuration from XML file.
        /// </summary>
        /// <param name="path">Path to XML file (default: configs/default_stock.xml).</param>
        /// <returns>Stock configuration.</returns>
        public static StockConfiguration LoadStockConfig(string? path = null)
        {
            path = path ?? DefaultStockConfigPath;
            if (!File.Exists(path))
                throw new FileNotFoundException($"Stock configuration not found at: {Path.GetFullPath(path)}");

            return StockConfiguration.LoadFromXml(path);
        }
    }
}
