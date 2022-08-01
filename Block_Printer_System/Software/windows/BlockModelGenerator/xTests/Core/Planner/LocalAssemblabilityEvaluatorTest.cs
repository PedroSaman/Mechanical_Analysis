using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;
using Core.Planner;
using Xunit;

namespace xTests.Core.Planner
{
    public class PartialAssemblabilityEvaluatorTest
    {
        [Fact]
        void AssemblyConflictTest()
        {
            var component = new AssemblyComponent(Grid3.Unit, Grid3.Origin);
            var assembly = new Assembly();
            assembly.AddComponent(component);
            Assert.Throws<InvalidOperationException>(() => assembly.AddComponent(component));
        }
        [Fact]
        void GetConnectedLowerComponentsTest()
        {
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(2, 0, 0);
            var position3 = new Grid3(1, 1, 1);
            var size = new Grid3(2, 2, 1);
            var assembly = new Assembly();
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            //
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component1));
            //
            assembly.AddComponent(component1);
            //
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component1));
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component2));
            Assert.Contains(assembly.GetConnectedLowerComponentsTo(component3),
                b => b.Position.Equals(component1.Position) && b.Size.Equals(component1.Size));
            //
            assembly.AddComponent(component2);
            //
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component1));
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component2));
            Assert.Contains(assembly.GetConnectedLowerComponentsTo(component3),
                b => b.Position.Equals(component1.Position) && b.Size.Equals(component1.Size));
            Assert.Contains(assembly.GetConnectedLowerComponentsTo(component3),
                b => b.Position.Equals(component2.Position) && b.Size.Equals(component2.Size));
            //
            assembly.AddComponent(component3);
            //
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component1));
            Assert.Empty(assembly.GetConnectedLowerComponentsTo(component2));
        }
        [Fact]
        void GetConnectedUpperComponentsTest()
        {
            var position1 = new Grid3(1, 1, 0);
            var position2 = new Grid3(0, 0, 1);
            var position3 = new Grid3(2, 0, 1);
            var size = new Grid3(2, 2, 1);
            var assembly = new Assembly();
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            //
            Assert.Empty(assembly.GetConnectedUpperComponentsTo(component1));
            //
            assembly.AddComponent(component1);
            //
            Assert.Empty(assembly.GetConnectedUpperComponentsTo(component1));
            Assert.Empty(assembly.GetConnectedUpperComponentsTo(component2));
            //
            assembly.AddComponent(component2);
            //
            Assert.Contains(assembly.GetConnectedUpperComponentsTo(component1),
                b => b.Position.Equals(component2.Position) && b.Size.Equals(component2.Size));
            Assert.Empty(assembly.GetConnectedUpperComponentsTo(component2));
            //
            assembly.AddComponent(component3);
            //
            Assert.Contains(assembly.GetConnectedUpperComponentsTo(component1),
                b => b.Position.Equals(component2.Position) && b.Size.Equals(component2.Size));
            Assert.Contains(assembly.GetConnectedUpperComponentsTo(component1),
                b => b.Position.Equals(component3.Position) && b.Size.Equals(component3.Size));
            Assert.Empty(assembly.GetConnectedUpperComponentsTo(component2));
            Assert.Empty(assembly.GetConnectedUpperComponentsTo(component3));
        }
        [Fact]
        void GetConnectedLowerConvexesTest()
        {
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(2, 0, 0);
            var position3 = new Grid3(1, 1, 1);
            var size = new Grid3(2, 2, 1);
            var assembly = new Assembly();
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            //
            Assert.Empty(assembly.GetConnectedLowerConvexesTo(component1));
            //
            assembly.AddComponent(component1);
            assembly.AddComponent(component2);
            assembly.AddComponent(component3);
            Assert.Empty(assembly.GetConnectedLowerConvexesTo(component1));
            Assert.Empty(assembly.GetConnectedLowerConvexesTo(component2));
            Assert.Equal(2, assembly.GetConnectedLowerConvexesTo(component3).Count());
            Assert.Contains(new Grid3(1, 1, 0), assembly.GetConnectedLowerConvexesTo(component3));
            Assert.Contains(new Grid3(2, 1, 0), assembly.GetConnectedLowerConvexesTo(component3));
        }
        [Fact]
        public void GetSupportingConvexHullPairsTest()
        {
            {
                var size = new Grid3(2, 2, 1);
                var bottomPosition1 = new Grid3(0, 0, 0);
                var bottomPosition2 = new Grid3(2, 1, 0);
                var middlePosition = new Grid3(1, 0, 1);
                var topPosition = new Grid3(1, 0, 2);
                var bottomBlock1 = new AssemblyComponent(size, bottomPosition1);
                var bottomBlock2 = new AssemblyComponent(size, bottomPosition2);
                var middleBlock = new AssemblyComponent(size, middlePosition);
                var topBlock = new AssemblyComponent(size, topPosition);
                var assembly = new Assembly(new[] { bottomBlock1, bottomBlock2, middleBlock });
                var pairs = assembly.GetSupportingConvexHullPairs(topBlock).ToArray();
                Assert.Single(pairs);
            }
            {
                var size = new Grid3(2, 2, 1);
                var bottomPosition = new Grid3(1, 1, 0);
                var middlePosition1 = new Grid3(0, 0, 1);
                var middlePosition2 = new Grid3(2, 0, 1);
                var topPosition = new Grid3(1, 0, 2);
                var bottomBlock = new AssemblyComponent(size, bottomPosition);
                var middleBlock1 = new AssemblyComponent(size, middlePosition1);
                var middleBlock2 = new AssemblyComponent(size, middlePosition2);
                var topBlock = new AssemblyComponent(size, topPosition);
                var assembly = new Assembly(new[] { bottomBlock, middleBlock1, middleBlock2 });
                var pairs = assembly.GetSupportingConvexHullPairs(topBlock).ToArray();
                Assert.Equal(2, pairs.Count());
            }
            {
                var size = new Grid3(2, 2, 1);
                var bottomPosition = new Grid3(1, 1, 0);
                var middlePosition1 = new Grid3(0, 0, 1);
                var middlePosition2 = new Grid3(0, 2, 1);
                var middlePosition3 = new Grid3(2, 0, 1);
                var topPosition = new Grid3(1, 1, 2);
                var bottomBlock = new AssemblyComponent(size, bottomPosition);
                var middleBlock1 = new AssemblyComponent(size, middlePosition1);
                var middleBlock2 = new AssemblyComponent(size, middlePosition2);
                var middleBlock3 = new AssemblyComponent(size, middlePosition3);
                var topBlock = new AssemblyComponent(size, topPosition);
                var assembly = new Assembly(new[] { bottomBlock, middleBlock1, middleBlock2, middleBlock3 });
                var pairs = assembly.GetSupportingConvexHullPairs(topBlock).ToArray();
                Assert.Equal(3, pairs.Count());
            }
            {
                var size = new Grid3(2, 2, 1);
                var bottomPosition = new Grid3(1, 1, 0);
                var middlePosition1 = new Grid3(0, 0, 1);
                var middlePosition2 = new Grid3(0, 2, 1);
                var middlePosition3 = new Grid3(2, 0, 1);
                var middlePosition4 = new Grid3(2, 2, 1);
                var topPosition = new Grid3(1, 1, 2);
                var bottomBlock = new AssemblyComponent(size, bottomPosition);
                var middleBlock1 = new AssemblyComponent(size, middlePosition1);
                var middleBlock2 = new AssemblyComponent(size, middlePosition2);
                var middleBlock3 = new AssemblyComponent(size, middlePosition3);
                var middleBlock4 = new AssemblyComponent(size, middlePosition4);
                var topBlock = new AssemblyComponent(size, topPosition);
                var assembly = new Assembly(new[] { bottomBlock, middleBlock1, middleBlock2, middleBlock3, middleBlock4 });
                var pairs = assembly.GetSupportingConvexHullPairs(topBlock).ToArray();
                Assert.Equal(4, pairs.Count());
            }
        }
        [Fact]
        void ConflictTest()
        {
            var size = new Grid3(2, 2, 1);
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(1, 0, 0);
            var position3 = new Grid3(1, 0, -1);
            var unconflictPosition = new Grid3(0, 2, 0);
            var assembly = new Assembly();
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            var unconflictComponent = new AssemblyComponent(size, unconflictPosition);
            //
            Assert.False(assembly.ConflictWith(unconflictComponent));
            //
            assembly.AddComponent(component1);
            Assert.True(assembly.ConflictWith(component1));
            Assert.True(assembly.ConflictWith(component2));
            Assert.True(assembly.ConflictWith(component3));
            Assert.False(assembly.ConflictWith(unconflictComponent));
        }
        [Fact]
        void FloatAndIsolatesTest()
        {
            var size = new Grid3(2, 2, 1);
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(2, 0, 1);
            var assembly = new Assembly();
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            assembly.AddComponent(component1);
            Assert.False(assembly.FloatAndIsolates(component1));
            Assert.True(assembly.FloatAndIsolates(component2));
        }
        [Fact]
        void HasStableBaseTest()
        {
            var size = new Grid3(2, 2, 1);
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(2, 0, 0);
            var position3 = new Grid3(1, 0, 1);
            var position4 = new Grid3(3, 0, 1);
            var position5 = new Grid3(2, 0, 2);
            var position6 = new Grid3(3, 0, 2);
            var assembly = new Assembly();
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            var component4 = new AssemblyComponent(size, position4);
            var component5 = new AssemblyComponent(size, position5);
            var component6 = new AssemblyComponent(size, position6);
            //
            Assert.True(assembly.HasStableBaseFor(component1));
            Assert.True(assembly.HasStableBaseFor(component2));
            Assert.True(assembly.HasStableBaseFor(component3));
            Assert.True(assembly.HasStableBaseFor(component4));
            Assert.True(assembly.HasStableBaseFor(component5));
            Assert.True(assembly.HasStableBaseFor(component6));
            //
            assembly.AddComponent(component1);
            assembly.AddComponent(component2);
            assembly.AddComponent(component3);
            assembly.AddComponent(component4);
            Assert.True(assembly.HasStableBaseFor(component5));
            Assert.False(assembly.HasStableBaseFor(component6));
        }
        [Fact]
        public void SupportByNeighborComponentTest_true()
        {
            var size = new Grid3(2, 2, 1);
            var position1 = new Grid3(1, 0, 0);
            var position2 = new Grid3(0, 0, 1);
            var position3 = new Grid3(2, 0, 1);
            var position4 = new Grid3(1, 0, 2);
            var position5 = new Grid3(3, 0, 2);
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            var component4 = new AssemblyComponent(size, position4);
            var component5 = new AssemblyComponent(size, position5);
            var assembly = new Assembly(new[] { component1, component2, component3, component4 });
            Assert.True(assembly.SupportByNeighborComponent(component5));
        }
        [Fact]
        public void SupportByNeighborComponentTest_false()
        {
            var size = new Grid3(2, 2, 1);
            var smallerSize = new Grid3(2, 1, 1);
            var position1 = new Grid3(1, 0, 0);
            var position2 = new Grid3(0, 0, 1);
            var position3 = new Grid3(2, 0, 1);
            var position4 = new Grid3(2, 0, 2);
            var position5 = new Grid3(3, 0, 2);
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            var component4 = new AssemblyComponent(smallerSize, position4);
            var component5 = new AssemblyComponent(size, position5);
            var assembly = new Assembly(new[] { component1, component2, component3, component4 });
            Assert.False(assembly.SupportByNeighborComponent(component5));
        }
        [Fact]
        public void GetPartialAssemblabilityTest_HasStableBase()
        {
            var size = new Grid3(2, 2, 1);
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(0, 0, 1);
            var position3 = new Grid3(0, 0, 2);
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            var assembly = new Assembly(new[] { component1, component2 });
            var assemblability = assembly.GetPartialAssemblability(component3);
            Assert.Equal(LocalAssemblabilityTag.HasStableBase, assemblability.Tag);
        }
        [Fact]
        public void GetPartialAssemblabilityTest_Isolated()
        {
            var size = new Grid3(2, 2, 1);
            var position1 = new Grid3(0, 0, 0);
            var position2 = new Grid3(0, 0, 1);
            var position3 = new Grid3(2, 0, 2);
            var component1 = new AssemblyComponent(size, position1);
            var component2 = new AssemblyComponent(size, position2);
            var component3 = new AssemblyComponent(size, position3);
            var assembly = new Assembly(new[] { component1, component2 });
            var assemblability = assembly.GetPartialAssemblability(component3);
            Assert.Equal(LocalAssemblabilityTag.Isolated, assemblability.Tag);
        }
        [Fact]
        public void GetPartialAssemblabilityTest_ContainsConvexHull()
        {
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 0);
                var position2 = new Grid3(1, 1, 1);
                var position3 = new Grid3(0, 0, 2);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var assembly = new Assembly(new[] { component1, component2 });
                var assemblability = assembly.GetPartialAssemblability(component3);
                Assert.Equal(LocalAssemblabilityTag.Supported, assemblability.Tag);
                Assert.Equal(1, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(1, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component3.Size, assemblability.BlockSize);
            }
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 0);
                var position2 = new Grid3(1, 0, 1);
                var position3 = new Grid3(0, 0, 2);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var assembly = new Assembly(new[] { component1, component2 });
                var assemblability = assembly.GetPartialAssemblability(component3);
                Assert.Equal(LocalAssemblabilityTag.Supported, assemblability.Tag);
                Assert.Equal(1, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(2, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component3.Size, assemblability.BlockSize);
            }
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 1, 0);
                var position2 = new Grid3(1, 0, 1);
                var position3 = new Grid3(1, 2, 1);
                var position4 = new Grid3(0, 1, 2);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var component4 = new AssemblyComponent(size, position4);
                var assembly = new Assembly(new[] { component1, component2, component3 });
                var assemblability = assembly.GetPartialAssemblability(component4);
                Assert.Equal(LocalAssemblabilityTag.Supported, assemblability.Tag);
                Assert.Equal(2, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(2, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component4.Size, assemblability.BlockSize);
            }
        }
        [Fact]
        public void GetPartialAssemblabilityTest_PartiallyContainsConvexHull()
        {
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 0);
                var position2 = new Grid3(1, 1, 1);
                var position3 = new Grid3(1, 0, 2);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var assembly = new Assembly(new[] { component1, component2 });
                var assemblability = assembly.GetPartialAssemblability(component3);
                Assert.Equal(LocalAssemblabilityTag.PartiallySupported, assemblability.Tag);
                Assert.Equal(1, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(2, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component3.Size, assemblability.BlockSize);
            }
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 0, 0);
                var position2 = new Grid3(1, 1, 1);
                var position3 = new Grid3(0, 0, 2);
                var position4 = new Grid3(1, 0, 3);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var component4 = new AssemblyComponent(size, position4);
                var assembly = new Assembly(new[] { component1, component2, component3 });
                var assemblability = assembly.GetPartialAssemblability(component4);
                Assert.Equal(LocalAssemblabilityTag.PartiallySupported, assemblability.Tag);
                Assert.Equal(1, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(2, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component4.Size, assemblability.BlockSize);
            }
        }
        [Fact]
        public void GetPartialAssemblability_SupportedByNeighbor()
        {
            {
                var size = new Grid3(2, 2, 1);
                var position1 = new Grid3(0, 1, 0);
                var position2 = new Grid3(1, 0, 1);
                var position3 = new Grid3(1, 2, 1);
                var position4 = new Grid3(0, 1, 2);
                var position5 = new Grid3(2, 1, 2);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var component4 = new AssemblyComponent(size, position4);
                var component5 = new AssemblyComponent(size, position5);
                var assembly = new Assembly(new[] { component1, component2, component3, component4 });
                var assemblability = assembly.GetPartialAssemblability(component5);
                Assert.Equal(LocalAssemblabilityTag.SupportedByNeightbor, assemblability.Tag);
                Assert.Equal(2, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(2, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component5.Size, assemblability.BlockSize);
            }
            {
                var size = new Grid3(2, 2, 1);
                var smallerSize = new Grid3(1, 2, 1);
                var position1 = new Grid3(0, 1, 0);
                var position2 = new Grid3(1, 0, 1);
                var position3 = new Grid3(1, 2, 1);
                var position4 = new Grid3(1, 1, 2);
                var position5 = new Grid3(2, 2, 2);
                var component1 = new AssemblyComponent(size, position1);
                var component2 = new AssemblyComponent(size, position2);
                var component3 = new AssemblyComponent(size, position3);
                var component4 = new AssemblyComponent(smallerSize, position4);
                var component5 = new AssemblyComponent(smallerSize, position5);
                var assembly = new Assembly(new[] { component1, component2, component3, component4 });
                var assemblability = assembly.GetPartialAssemblability(component5);
                Assert.Equal(LocalAssemblabilityTag.SupportedByNeightbor, assemblability.Tag);
                Assert.Equal(1, assemblability.ConnectedLowerBlockCount);
                Assert.Equal(2, assemblability.ConnectedLowerConvexCount);
                Assert.Equal(component5.Size, assemblability.BlockSize);
            }
        }
    }
}
