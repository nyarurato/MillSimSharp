using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using MillSimSharp.Viewer;
using System;

var nativeWindowSettings = new NativeWindowSettings()
{
    ClientSize = new Vector2i(1280, 720),
    Title = "MillSimSharp Voxel Viewer",
    Flags = ContextFlags.ForwardCompatible,
    Profile = ContextProfile.Core,
    API = ContextAPI.OpenGL,
    APIVersion = new Version(3, 3)
};

using (var window = new VoxelViewerWindow(GameWindowSettings.Default, nativeWindowSettings))
{
    window.Run();
}
