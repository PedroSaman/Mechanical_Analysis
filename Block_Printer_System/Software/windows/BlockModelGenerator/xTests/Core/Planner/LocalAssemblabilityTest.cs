using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;
using Core.Planner;
using Xunit;

namespace xTests.Core.Planner
{
    public class PartialAssemblabilityTest
    {
        private static readonly Grid3 smallBlockSize = new Grid3(1, 1, 1);
        private static readonly Grid3 largeBlockSize = new Grid3(2, 2, 1);
        [Fact]
        public void AssemblableTest()
        {
            Assert.True(LocalAssemblabilityTag.HasStableBase.IsAssemblable());
            Assert.True(LocalAssemblabilityTag.Supported.IsAssemblable());
            Assert.True(LocalAssemblabilityTag.PartiallySupported.IsAssemblable());
            Assert.True(LocalAssemblabilityTag.SupportedByNeightbor.IsAssemblable());
            Assert.False(LocalAssemblabilityTag.CausesRotation.IsAssemblable());
            Assert.False(LocalAssemblabilityTag.Isolated.IsAssemblable());
            Assert.False(LocalAssemblabilityTag.Conflicts.IsAssemblable());
        }
        [Fact]
        public void IsAssemblableTest()
        {
            var tags = Enum.GetValues(typeof(LocalAssemblabilityTag)).Cast<LocalAssemblabilityTag>();
            foreach (var tag in tags)
            {
                var localAssemblability = new LocalAssemblability(tag, 1, 1, Grid3.Unit);
                Assert.Equal(tag.IsAssemblable(), localAssemblability.IsAssemblable);
            }
        }
        [Fact]
        public void StudAssemblabilityComparisonTest()
        {
            Assert.True(LocalAssemblabilityTag.Supported.CompareTo(LocalAssemblabilityTag.PartiallySupported) > 0);
            Assert.True(LocalAssemblabilityTag.PartiallySupported.CompareTo(LocalAssemblabilityTag.SupportedByNeightbor) > 0);
            Assert.True(LocalAssemblabilityTag.SupportedByNeightbor.CompareTo(LocalAssemblabilityTag.CausesRotation) > 0);
        }
        [Fact]
        void CompareTest()
        {
            {
                var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, smallBlockSize);
                var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.PartiallySupported, 2, 1, smallBlockSize);
                Assert.True(assemblability1.CompareTo(assemblability2) > 0);
            }
            {
                var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, smallBlockSize);
                var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.PartiallySupported, 1, 2, smallBlockSize);
                Assert.True(assemblability1.CompareTo(assemblability2) > 0);
            }
            {
                var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, smallBlockSize);
                var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.PartiallySupported, 1, 2, largeBlockSize);
                Assert.True(assemblability1.CompareTo(assemblability2) > 0);
            }
            {
                var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.PartiallySupported, 1, 1, smallBlockSize);
                var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.SupportedByNeightbor, 2, 1, smallBlockSize);
                Assert.True(assemblability1.CompareTo(assemblability2) > 0);
            }
            {
                var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.PartiallySupported, 1, 1, smallBlockSize);
                var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.SupportedByNeightbor, 1, 2, smallBlockSize);
                Assert.True(assemblability1.CompareTo(assemblability2) > 0);
            }
            {
                var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.PartiallySupported, 1, 1, smallBlockSize);
                var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.SupportedByNeightbor, 1, 1, largeBlockSize);
                Assert.True(assemblability1.CompareTo(assemblability2) > 0);
            }
        }
        [Fact]
        void ConnectedLowerBlockCountComparisonTest()
        {
            var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 2, 1, smallBlockSize);
            var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, smallBlockSize);
            Assert.True(assemblability1.CompareTo(assemblability1) == 0);
            Assert.True(assemblability1.CompareTo(assemblability2) > 0);
        }
        [Fact]
        void ConnectedLowerConvexCountComparisonTest()
        {
            var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 2, smallBlockSize);
            var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, smallBlockSize);
            Assert.True(assemblability1.CompareTo(assemblability1) == 0);
            Assert.True(assemblability1.CompareTo(assemblability2) > 0);
        }
        [Fact]
        void BlockSizeComparisonTest()
        {
            var assemblability1 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, largeBlockSize);
            var assemblability2 = new LocalAssemblability(LocalAssemblabilityTag.Supported, 1, 1, smallBlockSize);
            Assert.True(assemblability1.CompareTo(assemblability1) == 0);
            Assert.True(assemblability1.CompareTo(assemblability2) > 0);
        }
    }
}
