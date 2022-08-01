using System;
using System.Collections.Generic;
using System.Linq;
using Core;

namespace Core
{
    class GenericCollection<TBaseType> : IEquatable<GenericCollection<TBaseType>> where TBaseType : class, IEquatable<TBaseType>
    {
        private readonly IReadOnlyDictionary<Type, TBaseType> items;
        public GenericCollection(IEnumerable<TBaseType> items) => this.items = items.ToDictionary(item => item.GetType());
        public GenericCollection<TBaseType> Append<TDerivedType>(TDerivedType item) where TDerivedType : TBaseType
        {
            var alreadyExists = this.Exists<TDerivedType>();
            var newItems = this.items.Values.Where(i => !(i is TDerivedType)).Append(item);
            return new GenericCollection<TBaseType>(newItems);
        }
        public bool Exists<TDerivedType>() where TDerivedType : TBaseType => this.items.ContainsKey(typeof(TDerivedType));
        public TDerivedType Single<TDerivedType>() where TDerivedType : TBaseType => (TDerivedType)this.items[typeof(TDerivedType)];
        public bool Equals(GenericCollection<TBaseType> other)
        {
            if (this.items.Count != other.items.Count) return false;
            foreach (var (key, value) in this.items)
            {
                if (!other.items.ContainsKey(key)) return false;
                var otherValue = other.items[key];
                if (!value.Equals(otherValue)) return false;
            }
            return true;
        }
        public override int GetHashCode() => this.items.Aggregate(0, (current, next) => current ^ next.GetHashCode());
    }
    class GenericCollection
    {
        public static GenericCollection<TBaseType> Empty<TBaseType>() where TBaseType : class, IEquatable<TBaseType> => new GenericCollection<TBaseType>(Enumerable.Empty<TBaseType>());
    }
}