using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using MillSimSharp.Toolpath;
using gs;
using MillSimSharp.Simulation;

namespace MillSimSharp.Viewer
{
    /// <summary>
    /// Simple helper: Convert a parsed G-Code file into a list of toolpath commands
    /// that the rest of the application understands.
    /// </summary>
    public static class GcodeToPath
    {
        /// <summary>
        /// Parse a GCodeFile into a list of IToolpathCommand objects.
        /// </summary>
        /// <param name="file">Parsed GCode file</param>
        /// <param name="initialPosition">Initial tool position</param>
        /// <returns>List of toolpath commands</returns>
        public static List<IToolpathCommand> Parse(GCodeFile file, Vector3 initialPosition)
        {
            if (file == null) throw new ArgumentNullException(nameof(file));

            var commands = new List<IToolpathCommand>();
            var curX = (double)initialPosition.X;
            var curY = (double)initialPosition.Y;
            var curZ = (double)initialPosition.Z;

            foreach (var line in file.AllLines())
            {
                if (line == null) continue;
                if (line.type != GCodeLine.LType.GCode) continue;

                switch (line.code)
                {
                    case 0: // G0 - rapid
                        ParseMove(line, ref curX, ref curY, ref curZ, commands, isRapid: true);
                        break;
                    case 1: // G1 - linear cut
                        ParseMove(line, ref curX, ref curY, ref curZ, commands, isRapid: false);
                        break;
                    default:
                        // ignore other codes for now
                        break;
                }
            }

            return commands;
        }

        /// <summary>
        /// Parse a file path (reads and parses with the included GenericGCodeParser)
        /// </summary>
        public static List<IToolpathCommand> ParseFromFile(string filePath, Vector3 initialPosition)
        {
            if (string.IsNullOrEmpty(filePath)) throw new ArgumentException("path required", nameof(filePath));
            if (!File.Exists(filePath)) throw new FileNotFoundException("G-code file not found", filePath);

            using (var sr = File.OpenText(filePath))
            {
                var parser = new GenericGCodeParser();
                var gfile = parser.Parse(sr);
                return Parse(gfile, initialPosition);
            }
        }

        private static void ParseMove(GCodeLine line, ref double curX, ref double curY, ref double curZ, List<IToolpathCommand> commands, bool isRapid)
        {
            double value = 0.0;
            double x = curX, y = curY, z = curZ;

            if (line.parameters != null)
            {
                if (GCodeUtil.TryFindParamNum(line.parameters, "X", ref value)) { x = value; }
                if (GCodeUtil.TryFindParamNum(line.parameters, "Y", ref value)) { y = value; }
                if (GCodeUtil.TryFindParamNum(line.parameters, "Z", ref value)) { z = value; }
            }

            var target = new Vector3((float)x, (float)y, (float)z);
            if (isRapid)
                commands.Add(new G0Move(target));
            else
            {
                // try to read feed (F), but default to 0
                float feed = 0f;
                if (line.parameters != null)
                {
                    if (GCodeUtil.TryFindParamNum(line.parameters, "F", ref value))
                        feed = (float)value;
                }
                commands.Add(new G1Move(target, feed));
            }

            curX = x;
            curY = y;
            curZ = z;
        }
    }
}