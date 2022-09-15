using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;
using Core.Attribute;
using Xunit;

namespace xTests.Core
{
    public class AsssemblyTest
    {
        [Fact]
        void EmptyConstructorTest()
        {
            var assembly = new Assembly();
            Assert.Empty(assembly);
            Assert.Throws<InvalidOperationException>(() => assembly.RightPosition);
            Assert.Throws<InvalidOperationException>(() => assembly.LeftPosition);
            Assert.Throws<InvalidOperationException>(() => assembly.FrontPosition);
            Assert.Throws<InvalidOperationException>(() => assembly.BackPosition);
            Assert.Throws<InvalidOperationException>(() => assembly.TopPosition);
            Assert.Throws<InvalidOperationException>(() => assembly.BottomPosition);
        }
        [Fact]
        void AddBlockTest()
        {
            var size = new Grid3(1, 1, 1);
            var position = new Grid3(0, 1, 2);
            var component = new AssemblyComponent(size, position);
            var assembly = new Assembly();
            assembly.AddComponent(component);
            Assert.Single(assembly);
            var containedBlock = assembly.Single();
            Assert.Equal(position, containedBlock.Position);
            Assert.Equal(size, containedBlock.Size);
        }
        [Fact]
        void AddBlockExceptionTest()
        {
            {
                var size = new Grid3(1, 1, 1);
                var position = new Grid3(0, 1, 2);
                var component = new AssemblyComponent(size, position);
                var assembly = new Assembly();
                assembly.AddComponent(component);
                Assert.Throws<InvalidOperationException>(() => assembly.AddComponent(component));
            }
            {
                var size1 = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 1);
                var component1 = new AssemblyComponent(size1, position1);
                var size2 = new Grid3(1, 1, 1);
                var position2 = new Grid3(1, 1, 1);
                var component2 = new AssemblyComponent(size1, position1);
                var assembly = new Assembly();
                assembly.AddComponent(component1);
                Assert.Throws<InvalidOperationException>(() => assembly.AddComponent(component2));
            }
        }
        [Fact]
        void OccupyTest()
        {
            var size = new Grid3(2, 2, 1);
            var position = new Grid3(0, 1, 2);
            var component = new AssemblyComponent(size, position);
            var assembly = new Assembly();
            assembly.AddComponent(component);
            Assert.True(assembly.Occupy(new Grid3(0, 1, 2)));
            Assert.True(assembly.Occupy(new Grid3(1, 1, 2)));
            Assert.True(assembly.Occupy(new Grid3(0, 2, 2)));
            Assert.True(assembly.Occupy(new Grid3(0, 1, 2)));
            Assert.False(assembly.Occupy(new Grid3(0, 0, 0)));
        }
        [Fact]
        void SidePositionTest()
        {
            var size = new Grid3(2, 2, 1);
            var position = new Grid3(0, 1, 2);
            var component = new AssemblyComponent(size, position);
            var assembly = new Assembly();
            assembly.AddComponent(component);
            Assert.Equal(1, assembly.RightPosition);
            Assert.Equal(0, assembly.LeftPosition);
            Assert.Equal(2, assembly.FrontPosition);
            Assert.Equal(1, assembly.BackPosition);
            Assert.Equal(2, assembly.TopPosition);
            Assert.Equal(2, assembly.BottomPosition);
        }
        [Fact]
        void CopyTest()
        {
            var size = new Grid3(2, 2, 1);
            var position = new Grid3(0, 1, 2);
            var component = new AssemblyComponent(size, position);
            var assembly = new Assembly();
            assembly.AddComponent(component);
            var copied = new Assembly(assembly);
            Assert.False(ReferenceEquals(assembly.First(), copied.First()));
        }
    }
}
