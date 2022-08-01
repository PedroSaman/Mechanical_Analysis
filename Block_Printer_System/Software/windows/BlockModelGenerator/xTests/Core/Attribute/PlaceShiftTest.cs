using Core.Attribute;
using Core.Geometry;
using Xunit;

namespace xTests.Core.Attribute
{
    public class PlaceShiftTest
    {
        [Fact]
        public void ConstructorTest()
        {
            var grid = new Grid2(1, 2);
            var shift = new PlaceShift(grid.X, grid.Y);
            Assert.Equal(grid, shift.Shift);
        }
        [Fact]
        public void EqualsTest()
        {
            var shift1 = new PlaceShift(1, 0);
            var shift2 = new PlaceShift(0, 1);
            Assert.True(shift1.Equals(shift1));
            Assert.False(shift1.Equals(shift2));
            Assert.False(shift1.Equals(null));
        }
    }
}
