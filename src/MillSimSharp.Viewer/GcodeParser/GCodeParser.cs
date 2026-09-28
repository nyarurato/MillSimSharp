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
    /// <item>G2 / G3 arcs in the G17 (XY) plane, emitted as short G1 chords (helical Z supported);
    /// I/J arcs whose commanded end is inconsistent with the start radius are ignored. The active
    /// plane is tracked modally: arcs selected in G18/G19 are ignored</item>
    /// <item>G20 / G21 units (inch / millimeter)</item>
    /// <item>G90 / G91 absolute / incremental distance mode</item>
    /// <item>F feed rate and M codes (M codes are ignored)</item>
    /// </list>
    /// Unsupported G codes (G18/G19 arcs, G28, canned cycles, work offsets, ...) are ignored as
    /// blocks: no motion is generated from such a line and the position stays unchanged.
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
            int plane = 17; // G17 (XY) is the only plane whose arcs are implemented; 18 = XZ, 19 = YZ

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

                // Apply modal words (units / distance mode) before interpreting coordinates, so the
                // result does not depend on the word order inside the block: "G91 G1 X1" and
                // "X1 G91 G1" must be equivalent. A G code this parser does not implement makes the
                // whole block unsupported: its modal words still apply, but it must not generate a
                // motion (for example "G28 X0" after "G1 X10" is not a return-to-origin cut).
                bool hasUnsupportedG = false;
                foreach (var (letter, value) in words)
                {
                    if (letter != 'G') continue;

                    if (!TryGetExactGCode(value, out int g))
                    {
                        // Fractional codes (G90.1, G0.1, ...) are not supported: rounding them would
                        // execute a different command (absolute mode / rapid move).
                        hasUnsupportedG = true;
                        continue;
                    }

                    switch (g)
                    {
                        case 0:
                        case 1:
                        case 2:
                        case 3:
                            break; // motion: handled with the coordinates below
                        case 17:
                            plane = 17; // XY: arc support
                            break;
                        case 18:
                            plane = 18; // XZ: linear moves still work, arcs are ignored
                            break;
                        case 19:
                            plane = 19; // YZ: linear moves still work, arcs are ignored
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
                            hasUnsupportedG = true;
                            break;
                    }
                }

                if (hasUnsupportedG)
                {
                    // Unsupported block (G28, canned cycles, work offsets, ...): emit no motion and
                    // keep the position unchanged.
                    continue;
                }

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
                            if (!TryGetExactGCode(value, out int g))
                            {
                                // Already filtered by the modal scan above; keep the block inert.
                                break;
                            }

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
                                    break; // plane selection is handled in the modal scan
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
                    // Arcs are only implemented in the G17 (XY) plane. An arc without I/J/R cannot
                    // be simulated either (ignore it instead of turning it into a linear move), and
                    // an arc with an impossible radius is ignored as well.
                    bool valid = plane == 17
                        && (hasI || hasJ || hasR)
                        && EmitArc(commands, moveMotion == 2,
                            x, y, z, nx, ny, nz,
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

        /// <summary>
        /// Parses a G code word exactly: fractional codes (G90.1, G0.1, ...) must not be rounded to
        /// another command. Returns false for non-integer or out-of-range values.
        /// </summary>
        private static bool TryGetExactGCode(double value, out int code)
        {
            if (double.IsNaN(value) || value != Math.Floor(value) || value < int.MinValue || value > int.MaxValue)
            {
                code = 0;
                return false;
            }

            code = (int)value;
            return true;
        }

        private static double SetAxis(double current, double value, double unitScale, bool absolute)
        {
            double scaled = value * unitScale;
            return absolute ? scaled : current + scaled;
        }

        private static bool EmitArc(
            List<IToolpathCommand> commands, bool clockwise,
            double startX, double startY, double startZ,
            double endX, double endY, double endZ,
            double i, double j, double r, bool useRadius, double feed, float arcSegmentAngleDegrees)
        {
            double cx, cy;

            if (useRadius)
            {
                double dx = endX - startX;
                double dy = endY - startY;
                double distance = Math.Sqrt(dx * dx + dy * dy);
                if (distance < 1e-9) return false;

                // Radius must reach at least half of the chord length.
                if (Math.Abs(r) < distance * 0.5 - 1e-9) return false;

                double hSquared = r * r - distance * distance / 4.0;
                double h = hSquared > 0 ? Math.Sqrt(hSquared) : 0;
                double mx = (startX + endX) / 2.0;
                double my = (startY + endY) / 2.0;

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
                cx = startX + i;
                cy = startY + j;

                // I/J arcs use the start radius as the circle radius. The commanded end must lie
                // on that circle within the G-code rounding tolerance; otherwise the block is
                // ignored (no command emitted, position unchanged). The check uses the parser's
                // double-precision coordinates: converting to float first would misjudge valid
                // arcs far from the origin.
                double startRadius = Math.Sqrt(
                    (startX - cx) * (startX - cx) + (startY - cy) * (startY - cy));
                double endRadius = Math.Sqrt(
                    (endX - cx) * (endX - cx) + (endY - cy) * (endY - cy));
                double tolerance = Math.Max(1e-4, 1e-3 * startRadius);
                if (Math.Abs(endRadius - startRadius) > tolerance) return false;
            }

            double startAngle = Math.Atan2(startY - cy, startX - cx);
            double endAngle = Math.Atan2(endY - cy, endX - cx);
            double radius = Math.Sqrt((startX - cx) * (startX - cx) + (startY - cy) * (startY - cy));
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

            var end = new Vector3((float)endX, (float)endY, (float)endZ);

            for (int n = 1; n <= segments; n++)
            {
                // The final chord lands exactly on the commanded end point: for I/J arcs this
                // absorbs the small radial mismatch allowed above, and for R arcs it removes
                // floating-point noise.
                Vector3 target;
                if (n == segments)
                {
                    target = end;
                }
                else
                {
                    double t = (double)n / segments;
                    double angle = startAngle + sweep * t;
                    target = new Vector3(
                        (float)(cx + radius * Math.Cos(angle)),
                        (float)(cy + radius * Math.Sin(angle)),
                        (float)(startZ + (endZ - startZ) * t));
                }
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
