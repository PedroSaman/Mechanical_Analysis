using Core.Geometry;
using Xunit;

namespace xTests.Core.Geometry
{
    public class Grid2Test
    {
        [Theory]
        [InlineData(1, 2)]
        public void ConstructorTest(int x, int y)
        {
            var grid = new Grid2(x, y);
            Assert.Equal(x, grid.X);
            Assert.Equal(y, grid.Y);
        }
        [Theory]
        [InlineData(1, 2)]
        [InlineData(1, -2)]
        [InlineData(-1, 2)]
        [InlineData(-1, -2)]
        public void EqualTest(int x,int y)
        {
            var grid1 = new Grid2(x, y);
            var grid2 = new Grid2(x, y);
            Assert.Equal(grid1, grid2);
        }
    }
}
