using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;
using Core.Attribute;
using Xunit;

namespace xTests.Core
{
    public class AssemblyComponentTest
    {
        [Theory]
        [MemberData(nameof(CreateAssemblyComponentSources))]
        void ConstructorTest(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributes)
        {
            var component = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            Assert.Equal(size, component.Size);
            Assert.Equal(size, component.Block.Size);
            Assert.Equal(blockAttributes, component.BlockAttributeCollection);
            Assert.Equal(blockAttributes, component.Block.BlockAttributeCollection);
            Assert.Equal(position, component.Position);
            Assert.Equal(assemblyAttributes, component.AssemblyAttributeCollection);
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponents))]
        void CopyTest(AssemblyComponent source)
        {
            var copiedAssemblyComponent = new AssemblyComponent(source.Size, source.BlockAttributeCollection, source.Position, source.AssemblyAttributeCollection);
            //
            Assert.Equal(source.Position, copiedAssemblyComponent.Position);
            Assert.Equal(source.Size, copiedAssemblyComponent.Size);
            Assert.Equal(source.BlockAttributeCollection, copiedAssemblyComponent.BlockAttributeCollection);
            Assert.Equal(source.AssemblyAttributeCollection, copiedAssemblyComponent.AssemblyAttributeCollection);
            Assert.Equal(source, copiedAssemblyComponent);
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponents))]
        void OccupyTest(AssemblyComponent block)
        {
            var size = block.Size;
            //
            Assert.Equal(size.X * size.Y * size.Z, block.OccupyingPositions.Count);
            var expectedPositions =
                from x in Enumerable.Range(0, size.X)
                from y in Enumerable.Range(0, size.Y)
                from z in Enumerable.Range(0, size.Z)
                select block.Position + new Grid3(x, y, z);
            Assert.All(expectedPositions, p => block.OccupyingPositions.Contains(p));
            Assert.All(expectedPositions, p => block.Occupy(p));
        }
        [Theory]
        [InlineData(0, 0, 0, 1, 1, 1)]
        [InlineData(0, 0, 1, 1, 2, 1)]
        [InlineData(0, 1, 0, 2, 1, 1)]
        [InlineData(1, 0, 0, 2, 2, 1)]
        void GetSidePositionTest(int x, int y, int z, int sizeX, int sizeY, int sizeZ)
        {
            var position = new Grid3(x, y, z);
            var size = new Grid3(sizeX, sizeY, sizeZ);
            var block = new AssemblyComponent(size, position);
            //
            var rightPosition = block.RightPosition;
            Assert.Equal(x + sizeX - 1, rightPosition);
            var leftPosition = block.LeftPosition;
            Assert.Equal(x, leftPosition);
            var frontPosition = block.FrontPosition;
            Assert.Equal(y + sizeY - 1, frontPosition);
            var backPosition = block.BackPosition;
            Assert.Equal(y, backPosition);
            var upPosition = block.TopPosition;
            Assert.Equal(z + sizeZ - 1, upPosition);
            var downPosition = block.BottomPosition;
            Assert.Equal(z, downPosition);
        }
        [Fact]
        void SidePositionsRightTest()
        {
            var position = new Grid3(1, 2, 3);
            var size = new Grid3(2, 2, 1);
            var block = new AssemblyComponent(size, position);
            Assert.Contains(new Grid3(2, 2, 3), block.RightPositions);
            Assert.Contains(new Grid3(2, 3, 3), block.RightPositions);
        }
        [Fact]
        void SidePositionsLeftTest()
        {
            var position = new Grid3(1, 2, 3);
            var size = new Grid3(2, 2, 1);
            var block = new AssemblyComponent(size, position);
            Assert.Contains(new Grid3(1, 2, 3), block.LeftPositions);
            Assert.Contains(new Grid3(1, 3, 3), block.LeftPositions);
        }
        [Fact]
        void SidePositionsFrontTest()
        {
            var position = new Grid3(1, 2, 3);
            var size = new Grid3(2, 2, 1);
            var block = new AssemblyComponent(size, position);
            Assert.Contains(new Grid3(1, 3, 3), block.FrontPositions);
            Assert.Contains(new Grid3(2, 3, 3), block.FrontPositions);
        }
        [Fact]
        void SidePositionsBackTest()
        {
            var position = new Grid3(1, 2, 3);
            var size = new Grid3(2, 2, 1);
            var block = new AssemblyComponent(size, position);
            Assert.Contains(new Grid3(1, 2, 3), block.BackPositions);
            Assert.Contains(new Grid3(2, 2, 3), block.BackPositions);
        }
        [Fact]
        void SidePositionsTopTest()
        {
            var position = new Grid3(1, 2, 3);
            var size = new Grid3(2, 2, 1);
            var block = new AssemblyComponent(size, position);
            Assert.Contains(new Grid3(1, 2, 3), block.TopPositions);
            Assert.Contains(new Grid3(2, 2, 3), block.TopPositions);
            Assert.Contains(new Grid3(1, 3, 3), block.TopPositions);
            Assert.Contains(new Grid3(2, 3, 3), block.TopPositions);
        }
        [Fact]
        void SidePositionsBottomTest()
        {
            var position = new Grid3(1, 2, 3);
            var size = new Grid3(2, 2, 1);
            var block = new AssemblyComponent(size, position);
            Assert.Contains(new Grid3(1, 2, 3), block.BottomPositions);
            Assert.Contains(new Grid3(2, 2, 3), block.BottomPositions);
            Assert.Contains(new Grid3(1, 3, 3), block.BottomPositions);
            Assert.Contains(new Grid3(2, 3, 3), block.BottomPositions);
        }
        [Fact]
        void TouchTest()
        {
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 0);
                var position2 = new Grid3(1, 2, 0);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                Assert.True(component1.Touches(component2));
                Assert.True(component2.Touches(component1));
            }
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 0);
                var position2 = new Grid3(2, 2, 0);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                Assert.False(component1.Touches(component2));
                Assert.False(component2.Touches(component1));
            }
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponentSources))]
        void EqualTest(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributes)
        {
            var component1 = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            var component2 = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            Assert.Equal(component1, component1);
            Assert.Equal(component1, component2);
            Assert.Equal(component2, component1);
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponentSources))]
        void NotEqualTest_Size(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributes)
        {
            var component1 = new AssemblyComponent(size + new Grid3(1, 0, 0), blockAttributes, position, assemblyAttributes);
            var component2 = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            Assert.NotEqual(component1, component2);
            Assert.NotEqual(component2, component1);
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponentSources))]
        void NotEqualTest_BlockAttributes(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributes)
        {
            var notEqualBlockAttributes = blockAttributes.Append(new Color(999));
            var component1 = new AssemblyComponent(size, notEqualBlockAttributes, position, assemblyAttributes);
            var component2 = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            Assert.NotEqual(component1, component2);
            Assert.NotEqual(component2, component1);
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponentSources))]
        void NotEqualTest_Position(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributes)
        {
            var component1 = new AssemblyComponent(size, blockAttributes, position + new Grid3(1, 0, 0), assemblyAttributes);
            var component2 = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            Assert.NotEqual(component1, component2);
            Assert.NotEqual(component2, component1);
        }
        [Theory]
        [MemberData(nameof(CreateAssemblyComponentSources))]
        void NotEqualTest_AssemblyAttributes(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributes)
        {
            var notEqualAssemblyAttributes = assemblyAttributes.Append(new PlaceShift(999, 999));
            var component1 = new AssemblyComponent(size, blockAttributes, position, notEqualAssemblyAttributes);
            var component2 = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes);
            Assert.NotEqual(component1, component2);
            Assert.NotEqual(component2, component1);
        }
        public static IEnumerable<object[]> CreateAssemblyComponents()
        {
            return
            from objects in CreateAssemblyComponentSources()
            let size = (Grid3)objects[0]
            let blockAttributes = (GenericCollection<IBlockAttribute>)objects[1]
            let position = (Grid3)objects[2]
            let assemblyAttributes = (GenericCollection<IAssemblyAttribute>)objects[3]
            let component = new AssemblyComponent(size, blockAttributes, position, assemblyAttributes)
            select new object[] { component };
        }
        public static IEnumerable<object[]> CreateAssemblyComponentSources()
        {
            {
                var size = new Grid3(1, 1, 1);
                var blockAttributes = GenericCollection.Empty<IBlockAttribute>();
                var position = new Grid3(1, 1, 1);
                var assemblyAttributes = GenericCollection.Empty<IAssemblyAttribute>();
                yield return new object[] { size, blockAttributes, position, assemblyAttributes };
            }
            {
                var size = new Grid3(2, 2, 1);
                var blockAttributes = GenericCollection.Empty<IBlockAttribute>();
                var position = new Grid3(1, 1, 1);
                var assemblyAttributes = GenericCollection.Empty<IAssemblyAttribute>();
                yield return new object[] { size, blockAttributes, position, assemblyAttributes };
            }
            {
                var size = new Grid3(1, 1, 1);
                var blockAttributes = GenericCollection.Empty<IBlockAttribute>();
                var position = new Grid3(5, 4, 3);
                var assemblyAttributes = GenericCollection.Empty<IAssemblyAttribute>();
                yield return new object[] { size, blockAttributes, position, assemblyAttributes };
            }
            {
                var size = new Grid3(2, 2, 1);
                var blockAttributes = GenericCollection.Empty<IBlockAttribute>();
                var position = new Grid3(5, 4, 3);
                var assemblyAttributes = GenericCollection.Empty<IAssemblyAttribute>();
                yield return new object[] { size, blockAttributes, position, assemblyAttributes };
            }
            {
                var size = new Grid3(1, 1, 1);
                var blockAttributes = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { Support.Instance, new Color(1) });
                var position = new Grid3(1, 1, 1);
                var assemblyAttributes = new GenericCollection<IAssemblyAttribute>(new IAssemblyAttribute[] { PressCapability.CanPress, AssemblyArea.Default });
                yield return new object[] { size, blockAttributes, position, assemblyAttributes };
            }
        }
    }
}
