using System;

namespace MillSimSharp.Viewer
{
    /// <summary>
    /// Coordinates asynchronous mesh regeneration: at most one build runs at a time, at most one
    /// follow-up request is queued, and results from stale generations are rejected.
    /// <para>
    /// Usage: call <see cref="TryBegin"/> before starting a build. When it returns a generation,
    /// start the build and check <see cref="IsCurrent"/> with that generation when the result
    /// arrives. Call <see cref="Complete"/> when the build finishes; if it returns true, a request
    /// arrived while the build was running and another build must be started.
    /// </para>
    /// </summary>
    public sealed class MeshGenerationScheduler
    {
        private readonly object _sync = new object();
        private int _generation;
        private bool _running;
        private bool _rerunQueued;

        /// <summary>
        /// Current generation. Only the generation returned by the latest successful
        /// <see cref="TryBegin"/> may apply its result.
        /// </summary>
        public int CurrentGeneration
        {
            get
            {
                lock (_sync)
                {
                    return _generation;
                }
            }
        }

        /// <summary>
        /// Whether a build is currently running.
        /// </summary>
        public bool IsRunning
        {
            get
            {
                lock (_sync)
                {
                    return _running;
                }
            }
        }

        /// <summary>
        /// Tries to start a build.
        /// </summary>
        /// <returns>The new generation when a build should start, or null when one is already
        /// running (the request is recorded and <see cref="Complete"/> will report a follow-up).</returns>
        public int? TryBegin()
        {
            lock (_sync)
            {
                if (_running)
                {
                    _rerunQueued = true;
                    return null;
                }

                _running = true;
                _rerunQueued = false;
                return ++_generation;
            }
        }

        /// <summary>
        /// Marks the running build as finished.
        /// </summary>
        /// <returns>True when another build was queued while this one was running; the caller must
        /// start it. Only one follow-up is queued at a time.</returns>
        public bool Complete()
        {
            lock (_sync)
            {
                _running = false;
                bool rerun = _rerunQueued;
                _rerunQueued = false;
                return rerun;
            }
        }

        /// <summary>
        /// Returns true when the given generation may still apply its result (no newer build has
        /// started since).
        /// </summary>
        /// <param name="generation">Generation returned by <see cref="TryBegin"/>.</param>
        public bool IsCurrent(int generation)
        {
            lock (_sync)
            {
                return generation == _generation;
            }
        }
    }
}
