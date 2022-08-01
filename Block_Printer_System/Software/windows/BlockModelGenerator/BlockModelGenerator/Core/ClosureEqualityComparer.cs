using System;
using System.Collections.Generic;

namespace Core
{
    class ClosureEqualityComparer<T> : IEqualityComparer<T>
    {
        public delegate bool Comparer(T x, T y);
        private readonly Comparer comparer;
        public ClosureEqualityComparer(Comparer comparer) => this.comparer = comparer;
        public bool Equals(T x, T y) => this.comparer(x, y);
        public int GetHashCode(T obj) => obj?.GetHashCode() ?? 0;
    }
}