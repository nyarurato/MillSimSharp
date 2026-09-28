using MillSimSharp.Viewer;
using NUnit.Framework;

namespace MillSimSharp.Tests.Viewer
{
    /// <summary>
    /// Tests for the async mesh generation scheduler: requests that arrive while a build is running
    /// and results from stale generations (the race that previously left an old mesh on screen).
    /// </summary>
    [TestFixture]
    public class MeshGenerationSchedulerTest
    {
        [Test]
        public void TryBegin_WhileRunning_QueuesExactlyOneFollowUp()
        {
            var scheduler = new MeshGenerationScheduler();

            int? first = scheduler.TryBegin();
            Assert.That(first, Is.Not.Null);
            Assert.That(scheduler.IsRunning, Is.True);

            Assert.That(scheduler.TryBegin(), Is.Null, "a second build must not start while one runs");
            Assert.That(scheduler.TryBegin(), Is.Null, "further requests must stay queued as well");

            Assert.That(scheduler.Complete(), Is.True, "a follow-up was requested while running");
            Assert.That(scheduler.IsRunning, Is.False);

            int? second = scheduler.TryBegin();
            Assert.That(second, Is.Not.Null);
            Assert.That(scheduler.Complete(), Is.False, "no follow-up was requested during the second build");
        }

        [Test]
        public void IsCurrent_AfterANewerGenerationStarted_RejectsTheOldResult()
        {
            // Simulates the raced completion order: generation 1 finishes after generation 2 started.
            // The older result must be discarded so the newer mesh keeps the display.
            var scheduler = new MeshGenerationScheduler();

            int? first = scheduler.TryBegin();
            scheduler.Complete();
            int? second = scheduler.TryBegin();

            Assert.That(first, Is.Not.Null);
            Assert.That(second, Is.Not.Null);
            Assert.That(scheduler.IsCurrent(first!.Value), Is.False,
                "an older generation must not overwrite a newer mesh");
            Assert.That(scheduler.IsCurrent(second!.Value), Is.True);

            // After the newer build completes, its own result is still the current generation.
            scheduler.Complete();
            Assert.That(scheduler.IsCurrent(second.Value), Is.True);
        }
    }
}
