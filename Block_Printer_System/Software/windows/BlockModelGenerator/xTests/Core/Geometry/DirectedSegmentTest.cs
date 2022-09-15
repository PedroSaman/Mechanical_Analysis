using System;
using System.Collections.Generic;
using System.Linq;
using Core.Geometry;
using Xunit;

namespace xTests.Core.Geometry
{
    public class DirectedSegmentTest
    {
        [Theory]
        [InlineData(1, 2, 3, 4)]
        void ConstructorTest(int x1, int y1, int x2, int y2)
        {
            var start = new Grid2(x1, y1);
            var end = new Grid2(x2, y2);
            var segment = new DirectedSegment(start, end);
            Assert.Equal(start, segment.Start);
            Assert.Equal(end, segment.End);
        }
        [Theory]
        [InlineData(0, 1)]
        void ContsructorExceptionTest(int x, int y)
        {
            var p = new Grid2(x, y);
            Assert.Throws<ArgumentException>(() => new DirectedSegment(p, p));
        }
        [Theory]
        [MemberData(nameof(CreateSideTestData))]
        void SideTest(Grid2 start, Grid2 end, Grid2 point, LineSide expectedLineSide)
        {
            var segment = new DirectedSegment(start, end);
            var actualSide = segment.SideOf(point);
            Assert.Equal(expectedLineSide, actualSide);
        }
        public static IEnumerable<object[]> CreateSideTestData()
        {
            {
                var start = new Grid2(-1, -1);
                var end = new Grid2(1, 1);
                var point = new Grid2(10, 1);
                var side = LineSide.Right;
                yield return new object[] { start, end, point, side };
            }
            {
                var start = new Grid2(-1, 1);
                var end = new Grid2(1, 0);
                var point = new Grid2(10, 1);
                var side = LineSide.Left;
                yield return new object[] { start, end, point, side };
            }
            {
                var start = new Grid2(0, 0);
                var end = new Grid2(1, 1);
                var point = new Grid2(5, 5);
                var side = LineSide.Center;
                yield return new object[] { start, end, point, side };
            }
            {
                var start = new Grid2(0, 0);
                var end = new Grid2(1, 0);
                var point = new Grid2(10, 1);
                var side = LineSide.Left;
                yield return new object[] { start, end, point, side };
            }
            {
                var start = new Grid2(0, 0);
                var end = new Grid2(0, 1);
                var point = new Grid2(1, 0);
                var side = LineSide.Right;
                yield return new object[] { start, end, point, side };
            }
        }
    }
}
