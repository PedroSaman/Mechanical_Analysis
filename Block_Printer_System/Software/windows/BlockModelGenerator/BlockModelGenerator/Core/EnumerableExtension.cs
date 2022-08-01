using System;
using System.Collections.Generic;
using System.Linq;

namespace Core
{
    public static class EnumerableExtension
    {
        /// <summary>
        /// �w�肵���͈͓�̐�����ԂɕԂ��D
        /// </summary>
        /// <param name="from">�V�[�P���X�̍ŏ��̗v�f�D</param>
        /// <param name="to">�V�[�P���X�̍Ō�̗v�f�D</param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static IEnumerable<int> FromTo(int from, int to)
        {
            if (from > to) throw new ArgumentOutOfRangeException($"{to} must be equal or larger than {from}");
            return Enumerable.Range(from, to - from + 1);
        }
        /// <summary>
        /// �V�[�P���X�̒��ŁC�w�肵������ŏ��l�ɂ���悤�ȗv�f��ׂĕԂ��D
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="U"></typeparam>
        /// <param name="enumerable"></param>
        /// <param name="keySelector"></param>
        /// <returns></returns>
        public static IEnumerable<T> MinElements<T, U>(this IEnumerable<T> enumerable, Func<T, U> keySelector)
        {
            var min = enumerable.Min(keySelector);
            return enumerable.Where(e => keySelector(e).Equals(min));
        }
        /// <summary>
        /// �V�[�P���X�̒��ŁC�w�肵������ő�l�ɂ���悤�ȗv�f��ׂĕԂ��D
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="U"></typeparam>
        /// <param name="enumerable"></param>
        /// <param name="keySelector"></param>
        /// <returns></returns>
        public static IEnumerable<T> MaxElements<T, U>(this IEnumerable<T> enumerable, Func<T, U> keySelector)
        {
            var max = enumerable.Max(keySelector);
            return enumerable.Where(e => keySelector(e).Equals(max));
        }
        /// <summary>
        /// �V�[�P���X�̊e�v�f��C0����n�܂�C���f�b�N�X�̃y�A�Ƃ��ĕԂ��D
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="source"></param>
        /// <returns></returns>
        public static IEnumerable<(T item, int index)> WithIndex<T>(this IEnumerable<T> source)
        {
            int index = 0;
            foreach (var item in source)
            {
                yield return (item, index);
                index++;
            }
        }
        public static IEnumerable<(TSource source, TMap map)> SelectWithSource<TSource, TMap>(this IEnumerable<TSource> source, Func<TSource, TMap> selector)
        {
            foreach (var item in source) yield return (item, selector(item));
        }
    }
}