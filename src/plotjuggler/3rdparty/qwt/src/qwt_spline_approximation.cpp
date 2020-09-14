/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_spline_approximation.h"
#include "qwt_spline_parametrization.h"

class QwtSplineApproximation::PrivateData
{
public:
    PrivateData():
        boundaryType( QwtSplineApproximation::ConditionalBoundaries )
    {
        parametrization = new QwtSplineParametrization( 
            QwtSplineParametrization::ParameterChordal );
    }

    ~PrivateData()
    {
        delete parametrization;
    }

    QwtSplineParametrization *parametrization;
    QwtSplineApproximation::BoundaryType boundaryType;
};

/*!
  \brief Constructor

  The default setting is a non closing spline with chordal parametrization

  \sa setParametrization(), setClosing()
 */
QwtSplineApproximation::QwtSplineApproximation()
{
    d_data = new PrivateData;
}

//! Destructor
QwtSplineApproximation::~QwtSplineApproximation()
{
    delete d_data;
}

/*!
  The locality of an spline interpolation identifies how many adjacent
  polynoms are affected, when changing the position of one point.

  A locality of 'n' means, that changing the coordinates of a point
  has an effect on 'n' leading and 'n' following polynoms.
  Those polynoms can be calculated from a local subpolygon.

  A value of 0 means, that the interpolation is not local and any modification
  of the polygon requires to recalculate all polynoms ( f.e cubic splines ). 

  \return Order of locality
 */
uint QwtSplineApproximation::locality() const
{
    return 0;
}

void QwtSplineApproximation::setParametrization( int type )
{
    if ( d_data->parametrization->type() != type )
    {
        delete d_data->parametrization;
        d_data->parametrization = new QwtSplineParametrization( type );
    }
}

void QwtSplineApproximation::setParametrization( QwtSplineParametrization *parametrization )
{
    if ( ( parametrization != NULL ) && ( d_data->parametrization != parametrization ) )
    {
        delete d_data->parametrization;
        d_data->parametrization = parametrization;
    }
}   

const QwtSplineParametrization *QwtSplineApproximation::parametrization() const
{
    return d_data->parametrization;
}

void QwtSplineApproximation::setBoundaryType( BoundaryType boundaryType )
{
    d_data->boundaryType = boundaryType;;
}

QwtSplineApproximation::BoundaryType QwtSplineApproximation::boundaryType() const
{
    return d_data->boundaryType;
}
