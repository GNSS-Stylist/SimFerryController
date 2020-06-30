/*
    autopilot.cpp (part of SimFerryController)
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

#include "autopilot.h"
#include "losolver.h"

Autopilot::Autopilot()
{

}

Autopilot::Autopilot(const Settings& settings)
{
    init(settings);
}

void Autopilot::init(const Settings &settings)
{
    this->settings = settings;

    pid_Position_N.setPID(settings.pidSettings_Position.p, settings.pidSettings_Position.i, settings.pidSettings_Position.d, settings.pidSettings_Position.f);
    pid_Position_N.setOutputLimits(settings.pidSettings_Position.maxOut);

    pid_Position_E.setPID(settings.pidSettings_Position.p, settings.pidSettings_Position.i, settings.pidSettings_Position.d, settings.pidSettings_Position.f);
    pid_Position_E.setOutputLimits(settings.pidSettings_Position.maxOut);

    pid_Heading.setPID(settings.pidSettings_Heading.p, settings.pidSettings_Heading.i, settings.pidSettings_Heading.d, settings.pidSettings_Heading.f);
    pid_Heading.setOutputLimits(settings.pidSettings_Heading.maxOut);

    pid_Position_N.setMaxIOutput(settings.pidSettings_Position.maxI);
    pid_Position_E.setMaxIOutput(settings.pidSettings_Position.maxI);
    pid_Heading.setMaxIOutput(settings.pidSettings_Heading.maxI);

    state = STATE_UNKNOWN;
}

void Autopilot::setDestination(const Destination destination)
{
    this->destination = destination;
}

void Autopilot::update(const Eigen::Transform<double, 3, Eigen::Affine> &transform, Outputs &outputs, double cycleTime, DebugOutputs* debugOutputs)
{
    Eigen::Vector3d origin3D = transform.translation();

    double heading, pitch, roll;
    LOSolver::ErrorCode errorCode;

    LOSolver::getYawPitchRollAngles(transform, heading, pitch, roll, errorCode, LOSolver::AC_NED);

    Eigen::Vector2d originCoords_2D_NE(origin3D(0), origin3D(1));
    Eigen::Vector2d targetCoords_2D_NE(destination.coord_N, destination.coord_E);
    Eigen::Vector2d relativeTargetCoords_2D_NE = targetCoords_2D_NE - originCoords_2D_NE;

    double absBearing = atan2(relativeTargetCoords_2D_NE(1), relativeTargetCoords_2D_NE(0));
    double distanceToTarget = relativeTargetCoords_2D_NE.norm();
    double relativeBearing = atan2(sin(absBearing - heading), cos(absBearing - heading));
    double headingError = -atan2(sin(destination.heading - heading), cos(destination.heading - heading));

    Eigen::Vector2d velocity = (originCoords_2D_NE - lastOrigin_2D_NE) / cycleTime;
    double speed = velocity.norm();
    double directionOfTravel = atan2(velocity(1), velocity(0));

    if (distanceToTarget > settings.nearLimit)
    {
        if (relativeBearing < (-M_PI / 2.))
        {
            outputs.direction_Front = -M_PI / 2.;
        }
        else if (relativeBearing > (M_PI / 2.))
        {
            outputs.direction_Front = M_PI / 2.;
        }
        else
        {
            outputs.direction_Front = fmod((relativeBearing + relativeBearing *
                                            settings.cruiseDirectionProp), M_PI);

            if (outputs.direction_Front < (-M_PI / 2.))
            {
                outputs.direction_Front = (-M_PI / 2.);
            }
            if (outputs.direction_Front > (M_PI / 2.))
            {
                outputs.direction_Front = (M_PI / 2.);
            }

        }

        outputs.direction_Back = -outputs.direction_Front;

        outputs.propulsion_Front = settings.cruisePropulsion;
        outputs.propulsion_Back = settings.cruisePropulsion;

        state = STATE_CRUISING;
    }
    else
    {
        if (state != STATE_NEAR)
        {
            // init PID-controllers when coming from cruising-state

            if (!settings.pidSettings_Position.rememberI)
            {
                pid_Position_N.reset();
                pid_Position_E.reset();
            }

            if (!settings.pidSettings_Heading.rememberI)
            {
                pid_Heading.reset();
            }
        }

        pid_Position_N.setSetpoint(targetCoords_2D_NE(0));
        double propulsion_N = pid_Position_N.getOutput(originCoords_2D_NE(0));

        pid_Position_E.setSetpoint(targetCoords_2D_NE(1));
        double proulsion_E = pid_Position_E.getOutput(originCoords_2D_NE(1));

        Eigen::Vector2d positionPropulsionVec_2D_NE(-propulsion_N, proulsion_E);

        // As the position propulsion vector is in woorld coordinates
        // we need to rotate it according to ferry's heading
        Eigen::Rotation2D<double> rotation(heading);
        Eigen::Vector2d positionPropulsionVec_Ferry = rotation.toRotationMatrix() * positionPropulsionVec_2D_NE;

        // Use headingError as a source value for heading-PID-controller
        pid_Heading.setSetpoint(0);
// Use this to test only position:        double headingPropulsion_Ferry = 0;
        double headingPropulsion_Ferry = pid_Heading.getOutput(headingError);

        // Heading correction only adds to left/right (or port/starboard) propulsion
        Eigen::Vector2d headingPropulsionVec_Ferry(0, headingPropulsion_Ferry);

        // Now we can just add position- and heading correction values together
        Eigen::Vector2d propulsionVec_Ferry_Front = positionPropulsionVec_Ferry + headingPropulsionVec_Ferry;
        Eigen::Vector2d propulsionVec_Ferry_Back = positionPropulsionVec_Ferry - headingPropulsionVec_Ferry;

        outputs.propulsion_Front = propulsionVec_Ferry_Front.norm();
        outputs.direction_Front = atan2(propulsionVec_Ferry_Front(1), - propulsionVec_Ferry_Front(0));

        outputs.propulsion_Back = propulsionVec_Ferry_Back.norm();
        outputs.direction_Back = atan2(propulsionVec_Ferry_Back(1), - propulsionVec_Ferry_Back(0));

        state = STATE_NEAR;
    }

    lastOrigin_2D_NE = originCoords_2D_NE;

    if (debugOutputs)
    {
        debugOutputs->absBearing = absBearing;
        debugOutputs->relativeBearing = relativeBearing;
        debugOutputs->distanceToTarget = distanceToTarget;
        debugOutputs->speed = speed;
        debugOutputs->directionOfTravel = directionOfTravel;
        debugOutputs->headingError = headingError;
        debugOutputs->state = state;
    }
}

