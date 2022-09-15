using Core.Attribute;
using Xunit;

namespace xTests.Core.Attribute
{
    public class AssemblyAreaTest
    {
        [Fact]
        public void NextTest()
        {
            var area = AssemblyArea.Default;
            var next = area.Next();
            Assert.Equal(area.Index + 1, next.Index);
        }
    }
}
