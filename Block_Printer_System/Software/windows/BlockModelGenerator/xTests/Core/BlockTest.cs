using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;
using Core.Attribute;
using Xunit;

namespace xTests.Core
{
    public class BlockTest
    {
        [Theory]
        [MemberData(nameof(CreateBlockSources))]
        void ConstructorTest(Grid3 size, GenericCollection<IBlockAttribute> blockAttributes)
        {
            var block = new Block(size, blockAttributes);
            Assert.Equal(size, block.Size);
            Assert.Equal(blockAttributes, block.BlockAttributeCollection);
        }
        [Fact]
        void ConstructorExceptionTest()
        {
            var invalidSizes = new[] { new Grid3(1, 1, 0), new Grid3(1, 0, 1), new Grid3(0, 1, 1), new Grid3(1, 1, -1), new Grid3(1, -1, 1), new Grid3(-1, 1, 1) };
            foreach (var size in invalidSizes)
            {
                Assert.Throws<ArgumentException>(() => new Block(size));
            }
        }
        [Theory]
        [MemberData(nameof(CreateBlocks))]
        void CopyTest(Block source)
        {
            var copiedBlock = new Block(source.Size, source.BlockAttributeCollection);
            //
            Assert.Equal(source.Size, copiedBlock.Size);
            Assert.Equal(source.BlockAttributeCollection, copiedBlock.BlockAttributeCollection);
            Assert.Equal(source, copiedBlock);
        }
        [Fact]
        public void EqualTest()
        {
            {
                var size = new Grid3(1, 1, 1);
                var attributes = GenericCollection.Empty<IBlockAttribute>();
                var block1 = new Block(size, attributes);
                var block2 = new Block(size, attributes);
                Assert.Equal(block1, block2);
                Assert.Equal(block2, block1);
            }
            {
                var size = new Grid3(1, 1, 1);
                var attributes = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { Support.Instance });
                var block1 = new Block(size, attributes);
                var block2 = new Block(size, attributes);
                Assert.Equal(block1, block2);
                Assert.Equal(block2, block1);
            }
            {
                var size = new Grid3(1, 1, 1);
                var attributes = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { Support.Instance, new Color(0) });
                var block1 = new Block(size, attributes);
                var block2 = new Block(size, attributes);
                Assert.Equal(block1, block2);
                Assert.Equal(block2, block1);
            }
        }
        [Fact]
        public void NotEqualTest()
        {
            {
                var size1 = new Grid3(1, 1, 1);
                var size2 = new Grid3(2, 1, 1);
                var attributes = GenericCollection.Empty<IBlockAttribute>();
                var block1 = new Block(size1, attributes);
                var block2 = new Block(size2, attributes);
                Assert.NotEqual(block1, block2);
                Assert.NotEqual(block2, block1);
            }
            {
                var size = new Grid3(1, 1, 1);
                var attributes1 = GenericCollection.Empty<IBlockAttribute>();
                var attributes2 = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { Support.Instance });
                var block1 = new Block(size, attributes1);
                var block2 = new Block(size, attributes2);
                Assert.NotEqual(block1, block2);
                Assert.NotEqual(block2, block1);
            }
        }
        public static IEnumerable<object[]> CreateBlockSources()
        {
            {
                var size = new Grid3(1, 1, 1);
                yield return new object[] { size, GenericCollection.Empty<IBlockAttribute>() };
            }
            {
                var size = new Grid3(2, 1, 1);
                yield return new object[] { size, GenericCollection.Empty<IBlockAttribute>() };
            }
            {
                var size = new Grid3(2, 1, 1);
                var attribute = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { Support.Instance });
                yield return new object[] { size, attribute };
            }
            {
                var size = new Grid3(2, 1, 1);
                var attribute = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { new Color(1) });
                yield return new object[] { size, attribute };
            }
            {
                var size = new Grid3(2, 1, 1);
                var attribute = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { new Color(1), Support.Instance });
                yield return new object[] { size, attribute };
            }
        }
        public static IEnumerable<object[]> CreateBlocks()
        {
            return
            from objects in CreateBlockSources()
            let size = (Grid3)objects[0]
            let blockAttributes = (GenericCollection<IBlockAttribute>)objects[1]
            let block = new Block(size, blockAttributes)
            select new object[] { block };
        }
    }
}
