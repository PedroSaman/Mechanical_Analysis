#include <block_printer_place/LeastSquarePlane.h>
#include <block_printer_place/SquareMatrix.h>
#include <block_printer_place/ColumnVector.h>
#include <cmath>

using namespace block_printer_place;

LeastSquarePlane::LeastSquarePlane() : xCoefficient(0), yCoefficient(0), constant(0), rsme(0){}

LeastSquarePlane::LeastSquarePlane(const LeastSquarePlane &leastSquarePlane)
    : xCoefficient(leastSquarePlane.xCoefficient),
      yCoefficient(leastSquarePlane.yCoefficient),
      constant(leastSquarePlane.constant),
      rsme(leastSquarePlane.rsme){}

LeastSquarePlane::LeastSquarePlane(const std::vector<double> &xPositions, const std::vector<double> &yPositions, const std::vector<double> &values)
{
    int num = (int)values.size();
    double xsum = 0, xxsum = 0, xysum = 0, xzsum = 0,
           ysum = 0, yysum = 0, yzsum = 0,
           zsum = 0;
    for (int i = 0; i < num; i++)
    {
        xsum += xPositions[i];
        xxsum += xPositions[i] * xPositions[i];
        xysum += xPositions[i] * yPositions[i];
        xzsum += xPositions[i] * values[i];
        ysum += yPositions[i];
        yysum += yPositions[i] * yPositions[i];
        yzsum += yPositions[i] * values[i];
        zsum += values[i];
    }
    ColumnVector column0 = {(double)num, xsum, ysum};
    ColumnVector column1 = {xsum, xxsum, xysum};
    ColumnVector column2 = {ysum, xysum, yysum};
    SquareMatrix squareMatrix = {column0, column1, column2};
    ColumnVector columnVector = {zsum, xzsum, yzsum};
    auto soltion = squareMatrix.inverse() * columnVector;
    this->xCoefficient = soltion.get(1);
    this->yCoefficient = soltion.get(2);
    this->constant = soltion.get(0);
    double errorSquare = 0;
    for (int i = 0; i < num; i++)
    {
        auto error = this->valueAt(xPositions[i], yPositions[i]);
        errorSquare += (error - values[i]) * (error - values[i]);
    }
    this->rsme = std::sqrt(errorSquare / num);
}

double LeastSquarePlane::valueAt(double x, double y) const
{
    return x * this->xCoefficient + y * this->yCoefficient + this->constant;
}

double LeastSquarePlane::valueAt(const Point2<double> &position) const
{
    return this->valueAt(position.x, position.y);
}

double LeastSquarePlane::getRsme() const
{
    return this->rsme;
}