using System;
using System.Linq;
using Core.Geometry;
using Xunit;

namespace xTests.Core.Geometry
{
    public class ConvexHullTest
    {
        [Fact]
        public void ConstructorExceptionTest()
        {
            var point1 = new Grid2(0, 0);
            var point2 = new Grid2(1, 0);
            Assert.Throws<ArgumentException>(() => new ConvexHull(Enumerable.Empty<Grid2>()));
            Assert.Throws<ArgumentException>(() => new ConvexHull(new[] { point1 }));
            Assert.Throws<ArgumentException>(() => new ConvexHull(new[] { point1, point2 }));
            Assert.Throws<ArgumentException>(() => new ConvexHull(new[] { point1, point1, point1 }));
            Assert.Throws<ArgumentException>(() => new ConvexHull(new[] { point1, point1, point2 }));
        }
        [Fact]
        public void PointContainTest()
        {
            {
                var p1 = new Grid2(0, 0);
                var p2 = new Grid2(2, 0);
                var p3 = new Grid2(2, 2);
                var p4 = new Grid2(0, 2);
                var convexHull = new ConvexHull(new[] { p1, p2, p3, p4 });
                var containPoint = new Grid2(1, 1);
                var touchPoint = new Grid2(0, 1);
                var uncontainPoint1 = new Grid2(0, 3);
                var uncontainPoint2 = new Grid2(-1, 0);
                var uncontainPoint3 = new Grid2(0, -1);
                var uncontainPoint4 = new Grid2(-1, -1);
                //
                Assert.True(convexHull.Contains(p1));
                Assert.True(convexHull.Contains(p2));
                Assert.True(convexHull.Contains(p3));
                Assert.True(convexHull.Contains(p4));
                Assert.True(convexHull.Contains(containPoint));
                Assert.True(convexHull.Contains(touchPoint));
                Assert.False(convexHull.Contains(uncontainPoint1));
                Assert.False(convexHull.Contains(uncontainPoint2));
                Assert.False(convexHull.Contains(uncontainPoint3));
                Assert.False(convexHull.Contains(uncontainPoint4));
            }
            {

                var p1 = new Grid2(1, 0);
                var p2 = new Grid2(2, 0);
                var p3 = new Grid2(2, 1);
                var p4 = new Grid2(1, 2);
                var p5 = new Grid2(0, 2);
                var p6 = new Grid2(0, 1);
                var convexHull = new ConvexHull(new[] { p1, p2, p3, p4, p5, p6 });
                var containPoint = new Grid2(1, 1);
                var uncontainPoint1 = new Grid2(0, 0);
                var uncontainPoint2 = new Grid2(2, 2);
                //
                Assert.True(convexHull.Contains(p1));
                Assert.True(convexHull.Contains(p2));
                Assert.True(convexHull.Contains(p3));
                Assert.True(convexHull.Contains(p4));
                Assert.True(convexHull.Contains(p5));
                Assert.True(convexHull.Contains(p6));
                Assert.True(convexHull.Contains(containPoint));
                Assert.False(convexHull.Contains(uncontainPoint1));
                Assert.False(convexHull.Contains(uncontainPoint2));
            }
        }
        [Fact]
        public void PointContainInnerTest()
        {
            {
                var p1 = new Grid2(0, 0);
                var p2 = new Grid2(2, 0);
                var p3 = new Grid2(2, 2);
                var p4 = new Grid2(0, 2);
                var convexHull = new ConvexHull(new[] { p1, p2, p3, p4 });
                var containPoint = new Grid2(1, 1);
                var uncontainPoint1 = new Grid2(4, 4);
                var uncontainPoint2 = new Grid2(0, 1);
                var uncontainPoint3 = new Grid2(2, 2);
                //
                Assert.True(convexHull.InnerContains(containPoint));
                Assert.False(convexHull.InnerContains(uncontainPoint1));
                Assert.False(convexHull.InnerContains(uncontainPoint2));
                Assert.False(convexHull.InnerContains(uncontainPoint3));
            }
            {
                var p1 = new Grid2(1, 0);
                var p2 = new Grid2(2, 0);
                var p3 = new Grid2(2, 1);
                var p4 = new Grid2(1, 2);
                var p5 = new Grid2(0, 2);
                var p6 = new Grid2(0, 1);
                var convexHull = new ConvexHull(new[] { p1, p2, p3, p4, p5, p6 });
                var containPoint = new Grid2(1, 1);
                var uncontainPoint1 = new Grid2(0, 3);
                var uncontainPoint2 = new Grid2(0, 1);
                var uncontainPoint3 = new Grid2(2, 2);
                //
                Assert.True(convexHull.InnerContains(containPoint));
                Assert.False(convexHull.InnerContains(uncontainPoint1));
                Assert.False(convexHull.InnerContains(uncontainPoint2));
                Assert.False(convexHull.InnerContains(uncontainPoint3));
            }
        }
        [Fact]
        public void ConvexHullContainTest()
        {
            var p1 = new Grid2(0, 0);
            var p2 = new Grid2(1, 0);
            var p3 = new Grid2(1, 1);
            var p4 = new Grid2(0, 1);
            var smallConvexHull = new ConvexHull(new[] { p1, p2, p3, p4 });
            var p5 = new Grid2(0, 0);
            var p6 = new Grid2(2, 0);
            var p7 = new Grid2(2, 2);
            var p8 = new Grid2(0, 2);
            var largeConvexHull = new ConvexHull(new[] { p5, p6, p7, p8 });
            //
            Assert.True(smallConvexHull.Contains(smallConvexHull));
            //
            Assert.False(smallConvexHull.Contains(largeConvexHull));
            //
            Assert.True(largeConvexHull.Contains(smallConvexHull));
        }
        [Fact]
        public void HasCommonPointTest()
        {
            var p1 = new Grid2(0, 0);
            var p2 = new Grid2(1, 0);
            var p3 = new Grid2(1, 2);
            var p4 = new Grid2(0, 2);
            var convexHull1 = new ConvexHull(new[] { p1, p2, p3, p4 });
            var p5 = new Grid2(0, 0);
            var p6 = new Grid2(2, 0);
            var p7 = new Grid2(2, 1);
            var p8 = new Grid2(0, 1);
            var convexHull2 = new ConvexHull(new[] { p5, p6, p7, p8 });
            //
            Assert.True(convexHull1.HasCommonPoint(convexHull1));
            Assert.True(convexHull1.HasCommonPoint(convexHull2));
            Assert.True(convexHull2.HasCommonPoint(convexHull1));
        }
        [Fact]
        public void HasInnerCommonPointTest()
        {
            {
                var p1 = new Grid2(0, 0);
                var p2 = new Grid2(2, 0);
                var p3 = new Grid2(2, 2);
                var p4 = new Grid2(0, 2);
                var convexHull = new ConvexHull(new[] { p1, p2, p3, p4 });
                var q1 = new Grid2(0, 0);
                var q2 = new Grid2(1, 0);
                var q3 = new Grid2(1, 1);
                var q4 = new Grid2(0, 1);
                var otherConvexHull = new ConvexHull(new[] { q1, q2, q3, q4 });
                Assert.True(convexHull.HasInnerCommonPoint(otherConvexHull));
                Assert.False(otherConvexHull.HasInnerCommonPoint(convexHull));
            }
            {
                var p1 = new Grid2(1, 0);
                var p2 = new Grid2(2, 0);
                var p3 = new Grid2(2, 1);
                var p4 = new Grid2(1, 2);
                var p5 = new Grid2(0, 2);
                var p6 = new Grid2(0, 1);
                var convexHull = new ConvexHull(new[] { p1, p2, p3, p4, p5, p6 });
                var q1 = new Grid2(1, 1);
                var q2 = new Grid2(2, 1);
                var q3 = new Grid2(2, 2);
                var q4 = new Grid2(1, 2);
                var otherConvexHull = new ConvexHull(new[] { q1, q2, q3, q4 });
                Assert.True(convexHull.HasInnerCommonPoint(otherConvexHull));
                Assert.False(otherConvexHull.HasInnerCommonPoint(convexHull));
            }
        }
    }
}
