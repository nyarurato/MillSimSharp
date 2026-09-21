using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text.RegularExpressions;
using MillSimSharp.Toolpath;

namespace MillSimSharp.Viewer
{
    /// <summary>
    /// Minimal G-code parser that produces toolpath commands. This is a viewer-side helper; the
    /// core library intentionally does not include a G-code parser.
    /// <list type="bullet">
    /// <item>G0 rapid and G1 linear moves</item>
    /// <item>G2 / G3 arcs in the G17 (XY) plane, emitted as short G1 chords (helical Z supported)</item>
    /// <item>G20 / G21 units (inch / millimeter)</item>
    /// <item>G90 / G91 absolute / incremental distance mode</item>
    /// <item>F feed rate and M codes (M codes are ignored)</item>
    /// </list>
    /// Unsupported G codes (G18/G19 arcs, G28, canned cycles, ...) are ignored.
    /// </summary>
    public static class GCodeParser
    {
        private static readonly Regex WordPattern = new Regex(
            "([A-Za-z])\\s*([+-]?(?:[0-9]+\\.?[0-9]*|\\.?[0-9]+))",
            RegexOptions.Compiled);

        /// <summary>
        /// Parses G-code text into a list of toolpath commands.
        /// </summary>
        /// <param name="gcode">G-code text.</param>
        /// <param name="initialPosition">Initial tool position (physical tool tip).</param>
        /// <param name="arcSegmentAngleDegrees">Maximum angular step for arc chords in degrees.</param>
        /// <returns>List of toolpath commands.</returns>
        public static List<IToolpathCommand> ParseText(string gcode, Vector3 initialPosition, float arcSegmentAngleDegrees = 2f)
        {
            if (gcode == null) throw new ArgumentNullException(nameof(gcode));

            var commands = new List<IToolpathCommand>();

            double x = initialPosition.X, y = initialPosition.Y, z = initialPosition.Z;
            double feed = 0;
            double unitScale = 1.0;
            bool absolute = true;
            int motion = 0; // 0 = G0, 1 = G1, 2 = G2, 3 = G3

            string[] lines = gcode.Split('\n');
            foreach (string rawLine in lines)
            {
                string line = StripComments(rawLine).Trim();
                if (line.Length == 0) continue;

                var words = new List<(char letter, double value)>();
                foreach (Match match in WordPattern.Matches(line))
                {
                    char letter = char.ToUpperInvariant(match.Groups[1].Value[0]);
                    double value = double.Parse(match.Groups[2].Value, CultureInfo.InvariantCulture);
                    words.Add((letter, value));
                }
                if (words.Count == 0) continue;

                bool hasAxis = false;
                double nx = x, ny = y, nz = z;
                double i = 0, j = 0, r = 0;
                bool hasI = false, hasJ = false, hasR = false;
                int moveMotion = motion;

                foreach (var (letter, value) in words)
                {
                    switch (letter)
                    {
                        case 'G':
                            int g = (int)Math.Round(value);
                            switch (g)
                            {
                                case 0:
                                case 1:
                                case 2:
                                case 3:
                                    motion = g;
                                    moveMotion = g;
                                    break;
                                case 20:
                                    unitScale = 25.4;
                                    break;
                                case 21:
                                    unitScale = 1.0;
                                    break;
                                case 90:
                                    absolute = true;
                                    break;
                                case 91:
                                    absolute = false;
                                    break;
                                default:
                                    break; // unsupported G code (plane/offset/cycle/...)
                            }
                            break;

                        case 'X':
                            nx = SetAxis(x, value, unitScale, absolute);
                            hasAxis = true;
                            break;
                        case 'Y':
                            ny = SetAxis(y, value, unitScale, absolute);
                            hasAxis = true;
                            break;
                        case 'Z':
                            nz = SetAxis(z, value, unitScale, absolute);
                            hasAxis = true;
                            break;
                        case 'I':
                            i = value * unitScale;
                            hasI = true;
                            break;
                        case 'J':
                            j = value * unitScale;
                            hasJ = true;
                            break;
                        case 'R':
                            r = value * unitScale;
                            hasR = true;
                            break;
                        case 'F':
                            feed = value * unitScale;
                            break;
                        default:
                            break; // N, M, S, T, ...
                    }
                }

                bool isArc = moveMotion == 2 || moveMotion == 3;

                if (isArc)
                {
                    // An arc without I/J/R cannot be simulated (ignore it instead of turning it
                    // into a linear move), and an arc with an impossible radius is ignored as well.
                    bool valid = (hasI || hasJ || hasR)
                        && EmitArc(commands, moveMotion == 2,
                            new Vector3((float)x, (float)y, (float)z),
                            new Vector3((float)nx, (float)ny, (float)nz),
                            i, j, r, hasR, feed, arcSegmentAngleDegrees);

                    if (!valid)
                    {
                        // Ignored line: keep the current position.
                        nx = x;
                        ny = y;
                        nz = z;
                    }
                }
                else if (hasAxis)
                {
                    var target = new Vector3((float)nx, (float)ny, (float)nz);
                    if (moveMotion == 0)
                        commands.Add(new G0Move(target));
                    else
                        commands.Add(new G1Move(target, (float)feed));
                }

                x = nx;
                y = ny;
                z = nz;
            }

            return commands;
        }

        /// <summary>
        /// Parses a G-code file into a list of toolpath commands.
        /// </summary>
        /// <param name="path">Path to the G-code file.</param>
        /// <param name="initialPosition">Initial tool position (physical tool tip).</param>
        /// <param name="arcSegmentAngleDegrees">Maximum angular step for arc chords in degrees.</param>
        /// <returns>List of toolpath commands.</returns>
        public static List<IToolpathCommand> ParseFile(string path, Vector3 initialPosition, float arcSegmentAngleDegrees = 2f)
        {
            if (path == null) throw new ArgumentNullException(nameof(path));
            return ParseText(File.ReadAllText(path), initialPosition, arcSegmentAngleDegrees);
        }

        private static double SetAxis(double current, double value, double unitScale, bool absolute)
        {
            double scaled = value * unitScale;
            return absolute ? scaled : current + scaled;
        }

        private static bool EmitArc(
            List<IToolpathCommand> commands, bool clockwise,
            Vector3 start, Vector3 end,
            double i, double j, double r, bool useRadius, double feed, float arcSegmentAngleDegrees)
        {
            double cx, cy;

            if (useRadius)
            {
                double dx = end.X - start.X;
                double dy = end.Y - start.Y;
                double distance = Math.Sqrt(dx * dx + dy * dy);
                if (distance < 1e-9) return false;

                // Radius must reach at least half of the chord length.
                if (Math.Abs(r) < distance * 0.5 - 1e-9) return false;

                double hSquared = r * r - distance * distance / 4.0;
                double h = hSquared > 0 ? Math.Sqrt(hSquared) : 0;
                double mx = (start.X + end.X) / 2.0;
                double my = (start.Y + end.Y) / 2.0;

                // Left normal of the chord direction.
                double perpX = -dy / distance;
                double perpY = dx / distance;

                // Positive R selects the minor arc (below 180 degrees), negative R the major arc.
                // For CCW motion the center lies to the left of the chord, for CW to the right.
                double side = clockwise ? -1.0 : 1.0;
                if (r < 0) side = -side;

                cx = mx + side * perpX * h;
                cy = my + side * perpY * h;
            }
            else
            {
                cx = start.X + i;
                cy = start.Y + j;
            }

            double startAngle = Math.Atan2(start.Y - cy, start.X - cx);
            double endAngle = Math.Atan2(end.Y - cy, end.X - cx);
            double radius = Math.Sqrt((start.X - cx) * (start.X - cx) + (start.Y - cy) * (start.Y - cy));
            if (radius < 1e-9) return false;

            double sweep = endAngle - startAngle;
            const double twoPi = Math.PI * 2.0;
            if (clockwise)
            {
                while (sweep >= 0) sweep -= twoPi;
                if (sweep < -twoPi) sweep += twoPi;
            }
            else
            {
                while (sweep <= 0) sweep += twoPi;
                if (sweep > twoPi) sweep -= twoPi;
            }

            float stepRadians = MathF.Max(arcSegmentAngleDegrees, 0.1f) * MathF.PI / 180f;
            int segments = Math.Max(1, (int)Math.Ceiling(Math.Abs(sweep) / stepRadians));

            for (int n = 1; n <= segments; n++)
            {
                double t = (double)n / segments;
                double angle = startAngle + sweep * t;
                var target = new Vector3(
                    (float)(cx + radius * Math.Cos(angle)),
                    (float)(cy + radius * Math.Sin(angle)),
                    (float)(start.Z + (end.Z - start.Z) * t));
                commands.Add(new G1Move(target, (float)feed));
            }

            return true;
        }

        private static string StripComments(string line)
        {
            int semicolon = line.IndexOf(';');
            if (semicolon >= 0) line = line.Substring(0, semicolon);

            while (true)
            {
                int open = line.IndexOf('(');
                if (open < 0) break;
                int close = line.IndexOf(')', open);
                if (close < 0)
                {
                    line = line.Substring(0, open);
                    break;
                }
                line = line.Remove(open, close - open + 1);
            }

            return line;
        }
    }
}
