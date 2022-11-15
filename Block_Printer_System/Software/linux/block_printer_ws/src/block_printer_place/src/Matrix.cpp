#include <block_printer_place/Matrix.h>

using namespace block_printer_place;

Matrix::Matrix(int line, int column)
{
    this->line = line;
    this->column = column;
    for (int i = 0; i < line; i++)
    {
        this->elements.push_back(std::vector<double>());
        for (int j = 0; j < column; j++)
        {
            this->elements[i].push_back(0);
        }
    }
}
Matrix::Matrix(const Matrix &matrix) : elements(matrix.elements), line(matrix.line), column(matrix.column)
{
}
Matrix::Matrix(const std::initializer_list<ColumnVector> &columnVectors)
{
    this->line = columnVectors.begin()->line();
    this->column = columnVectors.size();
    for (int line = 0; line < this->line; line++)
    {
        this->elements.push_back(std::vector<double>());
        for (int column = 0; column < this->column; column++)
        {
            this->elements[line].push_back(0);
        }
    }
    for (int line = 0; line < this->line; line++)
    {
        for (int column = 0; column < this->column; column++)
        {
            this->elements[line][column] = columnVectors.begin()[column].get(line);
        }
    }
}
int Matrix::getLine() const
{
    return this->line;
}
int Matrix::getColumn() const
{
    return this->column;
}
double Matrix::get(int line, int column) const
{
    return this->elements.at(line).at(column);
}
void Matrix::set(int line, int column, double value)
{
    this->elements[line][column] = value;
}
ColumnVector Matrix::getColumnVector(int column) const
{
    ColumnVector value(this->line);
    for (int i = 0; i < this->line; i++)
    {
        value.set(i, this->get(i, column));
    }
    return value;
}
Matrix Matrix::operator+(const Matrix &right) const
{
    Matrix value(this->line, this->column);
    for (int i = 0; i < this->line; i++)
    {
        for (int j = 0; j < this->column; j++)
        {
            value.set(i, j, this->get(i, j) + right.get(i, j));
        }
    }
    return value;
}
Matrix Matrix::operator-(const Matrix &right) const
{
    if (this->line != right.line || this->column != right.column)
        throw std::exception();
    Matrix value(this->line, this->column);
    for (int i = 0; i < this->line; i++)
    {
        for (int j = 0; j < this->column; j++)
        {
            value.set(i, j, this->get(i, j) - right.get(i, j));
        }
    }
    return value;
}
Matrix Matrix::operator*(const Matrix &right) const
{
    Matrix value(this->line, right.column);
    for (int line = 0; line < value.line; line++)
    {
        for (int column = 0; column < value.column; column++)
        {
            double sum = 0;
            for (int i = 0; i < this->column; i++)
            {
                sum += this->get(line, i) * right.get(i, column);
            }
            value.set(line, column, sum);
        }
    }
    return value;
}
ColumnVector Matrix::operator*(const ColumnVector &right) const
{
    ColumnVector value(this->line);
    for (int line = 0; line < this->line; line++)
    {
        double sum = 0;
        for (int column = 0; column < this->column; column++)
        {
            sum += this->get(line, column) * right.get(column);
        }
        value.set(line, sum);
    }
    return value;
}
Matrix &Matrix::operator=(const Matrix &right)
{
    this->elements = right.elements;
    this->line = right.line;
    this->column = right.column;
    return *this;
}