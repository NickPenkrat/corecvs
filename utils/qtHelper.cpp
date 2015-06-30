#include <QtCore/QtCore>
/**
 * \file qtHelper.cpp
 *
 * \date Apr 27, 2013
 **/

#include "qtHelper.h"

using corecvs::Vector2dd;
using corecvs::Vector3dd;

QDebug & operator<< (QDebug & stream, const Vector2dd & vector)
{
    stream << "[" << vector.x() << "," << vector.y() << "]";
    return stream;
}

QDebug & operator<< (QDebug & stream, const Vector3dd & vector)
{
    stream << "[" << vector.x() << "," << vector.y() << "," << vector.z() << "]";

    return stream;
}

void setValueBlocking(QDoubleSpinBox *box, double value)
{
    bool wasBlocked = box->blockSignals(true);
    box->setValue(value);
    box->blockSignals(wasBlocked);
}


QDebug &operator<<(QDebug &stream, const corecvs::Vector2d<int> &vector)
{
    stream << "[" << vector.x() << "," << vector.y() << "]";
    return stream;
}
