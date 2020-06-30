/*
    losolver.h (part of SimFerryController)
    Copyright (C) 2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef LOSOLVER_H
#define LOSOLVER_H

#include "Eigen/Geometry"

class LOSolver
{
public:
    enum ErrorCode
    {
        ERROR_NONE = 0,

        ERROR_INVALID_REFERENCE_POINTS = 100,

        ERROR_INVALID_POINTS = 200,
        ERROR_INVALID_AXES_CONVENTION,

        ERROR_NOT_KNOWN = 0xFF
    };

    enum AxesConvention
    {
        AC_NED,
        AC_EUS,
    };

    LOSolver();
    ErrorCode getLastError(void) { return errorCode; }

    bool getReferencePointsValidity(void) { return refPointsValid; }
    bool setReferencePoints(const Eigen::Vector3d& refPointA, const Eigen::Vector3d& refPointB, const Eigen::Vector3d& refPointC);
    bool setPoints(const Eigen::Vector3d& pointA, const Eigen::Vector3d& pointB, const Eigen::Vector3d& pointC);
    bool getTransformMatrix(Eigen::Transform<double, 3, Eigen::Affine>& transform,
                            Eigen::Transform<double, 3, Eigen::Affine>* orientationTransform_Debug = nullptr);

    bool getYawPitchRollAngles(const Eigen::Transform<double, 3, Eigen::Affine>& transform, double& yaw, double& pitch, double& roll, const AxesConvention convention = AC_EUS);
    static bool getYawPitchRollAngles(Eigen::Transform<double, 3, Eigen::Affine> transform, double& yaw, double& pitch, double& roll, ErrorCode& errorCode, const AxesConvention convention = AC_EUS);

    // TODO: This whole axes convention conversion scheme sucks and should be replaced with a better one.
    static Eigen::Transform<double, 3, Eigen::Affine> changeAxesConvention(const Eigen::Transform<double, 3, Eigen::Affine>& transform, const AxesConvention from, const AxesConvention to);
    static Eigen::Vector3d changeAxesConvention(const Eigen::Vector3d source, const AxesConvention from, const AxesConvention to);

private:
    ErrorCode errorCode = ERROR_NONE;

    Eigen::Matrix3d refBasis;

    bool refPointsValid = false;
    Eigen::Vector3d refPoints[3];
    Eigen::Vector3d points[3];

    // For speed-up when checking the validity of points:
    double refDistAB;
    double refDistAC;
    double refDistBC;

    bool calculateReferenceBasis(void);

    // TODO: This whole axes convention conversion scheme sucks and should be replaced with a better one.
    struct AxesConventionConversion
    {
        int fromIndex[3];
        double multiplier[3];
    };

    static Eigen::Transform<double, 3, Eigen::Affine> changeAxesConvention(const Eigen::Transform<double, 3, Eigen::Affine>& source, const AxesConventionConversion& conv);
    static AxesConventionConversion invertAxesConversionDirection(const AxesConventionConversion& source);
    static AxesConventionConversion getXXXToNEDConversionData(const AxesConvention from);
    static AxesConventionConversion getNEDToXXXConversionData(const AxesConvention to);
    static Eigen::Vector3d changeAxesConvention(const Eigen::Vector3d& source, const AxesConventionConversion& conv);

};

#endif // LOSOLVER_H
