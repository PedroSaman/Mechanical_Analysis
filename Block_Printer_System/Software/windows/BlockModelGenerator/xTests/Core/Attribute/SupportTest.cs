using Core.Attribute;
using Xunit;

namespace xTests.Core.Attribute
{
    public class SupportTest
    {
        [Fact]
        public void EqualsTest()
        {
            Assert.True(Support.Instance.Equals(Support.Instance));
            Assert.False(Support.Instance.Equals(null));
        }
    }
}
