using MillSimSharp.Viewer;
using NUnit.Framework;

namespace MillSimSharp.Tests.Viewer
{
    /// <summary>
    /// Tests for the async mesh generation scheduler: requests that arrive while a build is running
    /// and results from stale generations (the race that previously left an old mesh on screen).
    /// The call order mirrors the viewer: the finishing build asks <c>TryApplyResult</c> first, then
    /// calls <c>Complete</c> and starts the queued follow-up.
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
        public void Completion_WithQueuedFollowUp_DiscardsTheOldResult()
        {
            // Real viewer order for a request that arrives while a build is running: the build
            // finishes, asks whether its result may be shown, and only then completes and starts the
            // queued follow-up. The outdated mesh must not be shown even briefly.
            var scheduler = new MeshGenerationScheduler();

            int first = scheduler.TryBegin()!.Value;
            Assert.That(scheduler.TryBegin(), Is.Null, "requests while running are queued");

            Assert.That(scheduler.TryApplyResult(first), Is.False,
                "a queued follow-up must not show the old mesh");
            Assert.That(scheduler.Complete(), Is.True, "the queued follow-up must run");

            int second = scheduler.TryBegin()!.Value;
            Assert.That(scheduler.TryApplyResult(second), Is.True,
                "the follow-up result is the current one");
            Assert.That(scheduler.Complete(), Is.False);
        }

        [Test]
        public void TryApplyResult_AfterANewerGenerationStarted_DiscardsTheOldResult()
        {
            // Safety net: if a caller completes generations out of order, only the latest generation
            // may apply its result.
            var scheduler = new MeshGenerationScheduler();

            int first = scheduler.TryBegin()!.Value;
            Assert.That(scheduler.TryApplyResult(first), Is.True, "the first result is current");
            scheduler.Complete();

            int second = scheduler.TryBegin()!.Value;
            Assert.That(scheduler.IsCurrent(first), Is.False);
            Assert.That(scheduler.TryApplyResult(first), Is.False,
                "an older generation must not overwrite a newer mesh");
            Assert.That(scheduler.TryApplyResult(second), Is.True);
            scheduler.Complete();
        }
    }
}
