#include <block_printer_place/ColumnVector.h>

using namespace block_printer_place;

ColumnVector::ColumnVector(int line)
{
    for (int i = 0; i < line; i++)
    {
        this->elements.push_back(0);
    }
}

ColumnVector::ColumnVector(const ColumnVector &columnVector)    : elements(columnVector.elements)
{
}
ColumnVector::ColumnVector(const std::initializer_list<double> &elements)
{
    for (int i = 0; i < (int)elements.size(); i++)
    {
        this->elements.push_back(elements.begin()[i]);
    }
}
int ColumnVector::line() const
{
    return (int)this->elements.size();
}
double ColumnVector::get(int line) const
{
    return this->elements[line];
}
void ColumnVector::set(int line, double value)
{
    this->elements[line] = value;
}