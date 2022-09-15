using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;
using Core.Attribute;
using Xunit;

namespace xTests.Core
{
    public class GenericCollectionTest
    {
        interface IBase : IEquatable<IBase> { }
        class DerivedClass : IBase
        {
            private readonly int value;
            public DerivedClass(int v) => this.value = v;
            public bool Equals(IBase other) => other is DerivedClass d && this.value == d.value;
            public override int GetHashCode() => this.value.GetHashCode();
        }
        struct DerivedStruct : IBase
        {
            private readonly int value;
            public DerivedStruct(int v) => this.value = v;
            public bool Equals(IBase other) => other is DerivedStruct d && this.value == d.value;
            public override int GetHashCode() => this.value.GetHashCode();
        }
        [Fact]
        public void ExistsTest()
        {
            {
                var collection = GenericCollection.Empty<IBase>();
                Assert.False(collection.Exists<DerivedClass>());
                Assert.False(collection.Exists<DerivedStruct>());
            }
            {
                var items = new IBase[] { new DerivedClass(1) };
                var collection = new GenericCollection<IBase>(items);
                Assert.True(collection.Exists<DerivedClass>());
                Assert.False(collection.Exists<DerivedStruct>());
            }
            {
                var items = new IBase[] { new DerivedStruct(1) };
                var collection = new GenericCollection<IBase>(items);
                Assert.False(collection.Exists<DerivedClass>());
                Assert.True(collection.Exists<DerivedStruct>());
            }
            {
                var items = new IBase[] { new DerivedClass(1), new DerivedStruct(1) };
                var collection = new GenericCollection<IBase>(items);
                Assert.True(collection.Exists<DerivedClass>());
                Assert.True(collection.Exists<DerivedStruct>());
            }
        }
        [Fact]
        public void EqualTest()
        {
            {
                var collection1 = GenericCollection.Empty<IBase>();
                Assert.Equal(collection1, collection1);
            }
            {
                var items = new IBase[0];
                var collection1 = GenericCollection.Empty<IBase>();
                var collection2 = new GenericCollection<IBase>(items);
                Assert.Equal(collection1, collection2);
                Assert.Equal(collection2, collection1);
            }
            {
                var items = new IBase[] { new DerivedClass(1) };
                var collection1 = GenericCollection.Empty<IBase>();
                var collection2 = new GenericCollection<IBase>(items);
                Assert.Equal(collection1, collection1);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
            {
                var items1 = new IBase[] { new DerivedClass(1) };
                var items2 = new IBase[] { new DerivedClass(2) };
                var collection1 = new GenericCollection<IBase>(items1);
                var collection2 = new GenericCollection<IBase>(items2);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
            {
                var items = new IBase[] { new DerivedStruct(1) };
                var collection1 = GenericCollection.Empty<IBase>();
                var collection2 = new GenericCollection<IBase>(items);
                Assert.Equal(collection1, collection1);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
            {
                var items1 = new IBase[] { new DerivedStruct(1) };
                var items2 = new IBase[] { new DerivedStruct(2) };
                var collection1 = new GenericCollection<IBase>(items1);
                var collection2 = new GenericCollection<IBase>(items2);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
            {
                var items1 = new IBase[] { new DerivedClass(1) };
                var items2 = new IBase[] { new DerivedStruct(1) };
                var collection1 = new GenericCollection<IBase>(items1);
                var collection2 = new GenericCollection<IBase>(items2);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
            {
                var items1 = new IBase[] { new DerivedClass(1), new DerivedStruct(2) };
                var items2 = new IBase[] { new DerivedStruct(2), new DerivedClass(1) };
                var collection1 = new GenericCollection<IBase>(items1);
                var collection2 = new GenericCollection<IBase>(items2);
                Assert.Equal(collection1, collection1);
                Assert.Equal(collection1, collection2);
                Assert.Equal(collection2, collection1);
            }
            {
                var items1 = new IBase[] { new DerivedClass(1), new DerivedStruct(2) };
                var items2 = new IBase[] { new DerivedStruct(3), new DerivedClass(1) };
                var collection1 = new GenericCollection<IBase>(items1);
                var collection2 = new GenericCollection<IBase>(items2);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
            {
                var items1 = new IBase[] { new DerivedClass(1), new DerivedStruct(2) };
                var items2 = new IBase[] { new DerivedStruct(2), new DerivedClass(3) };
                var collection1 = new GenericCollection<IBase>(items1);
                var collection2 = new GenericCollection<IBase>(items2);
                Assert.NotEqual(collection1, collection2);
                Assert.NotEqual(collection2, collection1);
            }
        }
    }
}