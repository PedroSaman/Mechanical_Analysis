using Core.Geometry;
using Xunit;

namespace xTests.Core.Geometry
{
    public class Grid3Test
    {
        [Theory]
        [InlineData(1, 2, 3)]
        public void ConstructorTest(int x, int y, int z)
        {
            var grid = new Grid3(x, y, z);
            Assert.Equal(x, grid.X);
            Assert.Equal(y, grid.Y);
            Assert.Equal(z, grid.Z);
        }
        [Theory]
        [InlineData(1, 2, 3)]
        public void EqualTest(int x, int y, int z)
        {
            var grid1 = new Grid3(x, y, z);
            var grid2 = new Grid3(x, y, z);
            Assert.Equal(grid1, grid2);
        }
        [Theory]
        [InlineData(1, 2, 3)]
        public void MinusTest(int x, int y, int z)
        {
            var grid1 = new Grid3(x, y, z);
            var grid2 = -grid1;
            Assert.Equal(-grid1.X, grid2.X);
            Assert.Equal(-grid1.Y, grid2.Y);
            Assert.Equal(-grid1.Z, grid2.Z);
        }
        [Theory]
        [InlineData(1, 2, 3, 4, 5, 6)]
        public void AdditionTest(int x1, int y1, int z1, int x2, int y2, int z2)
        {
            var grid1 = new Grid3(x1, y1, z1);
            var grid2 = new Grid3(x2, y2, z2);
            var addition = grid1 + grid2;
            Assert.Equal(grid1.X + grid2.X, addition.X);
            Assert.Equal(grid1.Y + grid2.Y, addition.Y);
            Assert.Equal(grid1.Z + grid2.Z, addition.Z);
        }
        [Theory]
        [InlineData(1, 2, 3, 4, 5, 6)]
        public void SubtractionTest(int x1, int y1, int z1, int x2, int y2, int z2)
        {
            var grid1 = new Grid3(x1, y1, z1);
            var grid2 = new Grid3(x2, y2, z2);
            var subtraction = grid1 - grid2;
            Assert.Equal(grid1.X - grid2.X, subtraction.X);
            Assert.Equal(grid1.Y - grid2.Y, subtraction.Y);
            Assert.Equal(grid1.Z - grid2.Z, subtraction.Z);
        }
    }
}
