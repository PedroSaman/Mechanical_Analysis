using System;
using System.Linq;
using Core;
using Xunit;

namespace xTests.Core
{
    public class EnumerableExtensionTest
    {
        [Fact]
        public void FromToTest()
        {
            var sequence = EnumerableExtension.FromTo(1, 3).ToArray();
            Assert.Equal(3, sequence.Length);
            Assert.Equal(1, sequence[0]);
            Assert.Equal(2, sequence[1]);
            Assert.Equal(3, sequence[2]);
            var sequence2 = EnumerableExtension.FromTo(10, 10).ToArray();
            Assert.Single(sequence2);
            Assert.Contains(10, sequence2);
        }
        [Fact]
        public void FromToExceptionTest()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => EnumerableExtension.FromTo(1, 0));
        }
        [Fact]
        public void MinElementsTest()
        {
            var sequence = Enumerable.Range(-3, 7);
            double keySelector(int x) => Math.Abs(x * x - 4);
            var minElements = sequence.MinElements(keySelector).ToArray();
            Assert.Equal(2, minElements.Length);
            Assert.Contains(2, minElements);
            Assert.Contains(-2, minElements);
        }
        [Fact]
        public void MaxElementsTest()
        {
            var sequence = Enumerable.Range(-3, 7);
            double keySelector(int x) => -Math.Abs(x * x - 4);
            var maxElements = sequence.MaxElements(keySelector).ToArray();
            Assert.Equal(2, maxElements.Length);
            Assert.Contains(2, maxElements);
            Assert.Contains(-2, maxElements);
        }
        [Fact]
        public void WithIndexTest()
        {
            var sequence = Enumerable.Range(0, 3);
            var withIndex = sequence.WithIndex().ToArray();
            Assert.Equal(3, withIndex.Length);
            Assert.Equal(0, withIndex[0].item);
            Assert.Equal(0, withIndex[0].index);
            Assert.Equal(1, withIndex[1].item);
            Assert.Equal(1, withIndex[1].index);
            Assert.Equal(2, withIndex[2].item);
            Assert.Equal(2, withIndex[2].index);
        }
    }
}
