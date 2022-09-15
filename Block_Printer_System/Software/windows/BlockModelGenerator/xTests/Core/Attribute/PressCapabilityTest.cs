using Core.Attribute;
using Xunit;

namespace xTests.Core.Attribute
{
    public class PressCapabilityTest
    {
        [Fact]
        public void CapabilityTest()
        {
            Assert.True(PressCapability.CanPress.Capability);
            Assert.False(PressCapability.CannotPress.Capability);
        }
        [Fact]
        public void EqualsTest()
        {
            var canPress = PressCapability.CanPress;
            var cannotPress = PressCapability.CannotPress;
            Assert.True(canPress.Equals(canPress));
            Assert.True(cannotPress.Equals(cannotPress));
            //
            Assert.False(canPress.Equals(cannotPress));
            Assert.False(cannotPress.Equals(canPress));
            //
            Assert.False(canPress.Equals(null));
            Assert.False(cannotPress.Equals(null));
        }
    }
}
