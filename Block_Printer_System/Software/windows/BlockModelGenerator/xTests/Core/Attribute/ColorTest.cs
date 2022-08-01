using Core.Attribute;
using Xunit;

namespace xTests.Core.Attribute
{
    public class ColorTest
    {
        [Theory]
        [InlineData(1)]
        public void ConstructorTest(int index)
        {
            var color = new Color(index);
            Assert.Equal(index, color.Index);
        }
        [Fact]
        public void EqualsTest()
        {
            var color1 = new Color(0);
            var color2 = new Color(1);
            Assert.True(color1.Equals(color1));
            Assert.False(color1.Equals(color2));
            Assert.False(color1.Equals(null));
        }
    }
}
