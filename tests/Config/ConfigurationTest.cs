using NUnit.Framework;
using MillSimSharp.Config;
using MillSimSharp.Simulation;
using System;
using System.IO;
using System.Numerics;

namespace MillSimSharp.Tests.Config
{
    [TestFixture]
    public class ConfigurationTest
    {
        private string _configPath;

        [SetUp]
        public void Setup()
        {
            // Configs are copied to output directory by build
            _configPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "configs");
        }

        [Test]
        public void TestConfigFilesExist()
        {
            var toolPath = Path.Combine(_configPath, "default_tool.xml");
            var machinePath = Path.Combine(_configPath, "default_machine.xml");
            var stockPath = Path.Combine(_configPath, "default_stock.xml");

            Assert.That(File.Exists(toolPath), Is.True, $"Tool config not found at: {toolPath}");
            Assert.That(File.Exists(machinePath), Is.True, $"Machine config not found at: {machinePath}");
            Assert.That(File.Exists(stockPath), Is.True, $"Stock config not found at: {stockPath}");
        }

        [Test]
        public void TestLoadToolConfig()
        {
            var toolPath = Path.Combine(_configPath, "default_tool.xml");
            var config = ToolConfiguration.LoadFromXml(toolPath);

            Assert.That(config, Is.Not.Null);
            Assert.That(config.Diameter, Is.EqualTo(10.0f));
            Assert.That(config.Length, Is.EqualTo(50.0f));
            Assert.That(config.IsBallEnd, Is.False);
        }

        [Test]
        public void TestCreateToolFromConfig()
        {
            var toolPath = Path.Combine(_configPath, "default_tool.xml");
            var config = ToolConfiguration.LoadFromXml(toolPath);
            var tool = config.CreateTool();

            Assert.That(tool, Is.Not.Null);
            Assert.That(tool.Diameter, Is.EqualTo(10.0f));
            Assert.That(tool.Length, Is.EqualTo(50.0f));
            Assert.That(tool.Type, Is.EqualTo(ToolType.Flat));
        }

        [Test]
        public void TestLoadStockConfig()
        {
            var stockPath = Path.Combine(_configPath, "default_stock.xml");
            var config = StockConfiguration.LoadFromXml(stockPath);

            Assert.That(config, Is.Not.Null);
            Assert.That(config.WorkOrigin.ToVector3(), Is.EqualTo(Vector3.Zero));
            Assert.That(config.WorkSize.ToVector3(), Is.EqualTo(new Vector3(100, 100, 100)));
        }

        [Test]
        public void TestGetBoundingBoxFromStock()
        {
            var stockPath = Path.Combine(_configPath, "default_stock.xml");
            var config = StockConfiguration.LoadFromXml(stockPath);
            var bbox = config.GetBoundingBox();

            Assert.That(bbox, Is.Not.Null);
            Assert.That(bbox.Size, Is.EqualTo(new Vector3(100, 100, 100)));
        }

        [Test]
        public void TestLoadMachineConfig()
        {
            var machinePath = Path.Combine(_configPath, "default_machine.xml");
            var config = MachineConfiguration.LoadFromXml(machinePath);

            Assert.That(config, Is.Not.Null);
            Assert.That(config.Name, Is.EqualTo("Default Machine"));
            Assert.That(config.Axes, Is.Not.Null);
            Assert.That(config.Axes.Count, Is.EqualTo(3));
        }

        [Test]
        public void TestGetWorkVolumeFromMachine()
        {
            var machinePath = Path.Combine(_configPath, "default_machine.xml");
            var config = MachineConfiguration.LoadFromXml(machinePath);
            var volume = config.GetWorkVolume();

            Assert.That(volume, Is.Not.Null);
            Assert.That(volume.Min, Is.EqualTo(Vector3.Zero));
            Assert.That(volume.Max, Is.EqualTo(new Vector3(100, 100, 100)));
        }

        [Test]
        public void TestConfigurationLoader_AbsolutePaths()
        {
            var toolPath = Path.Combine(_configPath, "default_tool.xml");
            var machinePath = Path.Combine(_configPath, "default_machine.xml");
            var stockPath = Path.Combine(_configPath, "default_stock.xml");

            var toolConfig = ConfigurationLoader.LoadToolConfig(toolPath);
            var machineConfig = ConfigurationLoader.LoadMachineConfig(machinePath);
            var stockConfig = ConfigurationLoader.LoadStockConfig(stockPath);

            Assert.That(toolConfig, Is.Not.Null);
            Assert.That(machineConfig, Is.Not.Null);
            Assert.That(stockConfig, Is.Not.Null);
        }
    }
}
