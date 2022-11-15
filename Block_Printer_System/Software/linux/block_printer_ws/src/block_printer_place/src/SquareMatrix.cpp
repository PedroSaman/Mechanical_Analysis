#include <block_printer_place/SquareMatrix.h>

using namespace block_printer_place;

SquareMatrix::SquareMatrix(int size) : Matrix(size, size)
{
}
SquareMatrix::SquareMatrix(const SquareMatrix &squareMatrix) : Matrix(squareMatrix)
{
}
SquareMatrix::SquareMatrix(const std::initializer_list<ColumnVector> &columnVectors) : Matrix(columnVectors)
{
    if (this->getLine() != this->getColumn())
        throw std::exception();
}
int SquareMatrix::size() const
{
    return this->getLine();
}
SquareMatrix SquareMatrix::inverse() const
{
    SquareMatrix copy(*this);
    SquareMatrix value = SquareMatrix::createUnit(this->size());
    for (int i = 0; i < this->size(); i++)
    {
        auto diagonal = copy.get(i, i);
        for (int j = 0; j < this->size(); j++)
        {
            copy.set(i, j, copy.get(i, j) / diagonal);
            value.set(i, j, value.get(i, j) / diagonal);
        }
        for (int j = 0; j < this->size(); j++)
        {
            if (i == j)
                continue;
            auto nondiagonal = copy.get(j, i);
            for (int k = 0; k < this->size(); k++)
            {
                copy.set(j, k, copy.get(j, k) - copy.get(i, k) * nondiagonal);
                value.set(j, k, value.get(j, k) - value.get(i, k) * nondiagonal);
            }
        }
    }
    return value;
}
SquareMatrix &SquareMatrix::operator=(const SquareMatrix &right)
{
    this->Matrix::operator=(right);
    return *this;
}
SquareMatrix SquareMatrix::createUnit(int size)
{
    SquareMatrix value(size);
    for (int i = 0; i < size; i++)
        value.set(i, i, 1);
    return value;
}