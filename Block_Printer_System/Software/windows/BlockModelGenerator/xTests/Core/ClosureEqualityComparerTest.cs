using System;
using System.Collections.Generic;
using Core;
using Xunit;

namespace xTests.Core
{
    public class ClosureEqualityComparerTest
    {
        [Theory]
        [MemberData(nameof(CreateTestData))]
        public void CompareTest(int x, int y)
        {
            ClosureEqualityComparer<int>.Comparer comparer = (left, right) => left.Equals(right);
            var equalityComparer = new ClosureEqualityComparer<int>(comparer);
            var equality1 = x.Equals(y);
            var equality2 = equalityComparer.Equals(x, y);
            Assert.Equal(equality1, equality2);
        }
        public static IEnumerable<object[]> CreateTestData()
        {
            yield return new object[] { 0, 0 };
            yield return new object[] { 0, 1 };
            yield return new object[] { 1, 0 };
        }
    }
}