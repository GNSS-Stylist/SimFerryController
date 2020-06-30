/*
    autopilot.h (part of SimFerryController)
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

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "Eigen/Geometry"
#include "MiniPID/MiniPID.h"

class Autopilot
{
public:
    struct PIDSettings
    {
        double p;
        double i;
        double d;
        double f;       // See MiniPID's documentation
        double maxI;    // Limiting value for I (+-)
        double maxOut;
        bool rememberI; // Remember I term during cruising-state (expecting steady wind force / direction)
    };

    struct Settings
    {
        // Note: metres (m) are correct here only if units of your coordinate system is 1 m.
#if 0
        double maxSpeed;                    // m / s
        double maxAngularSpeed;             // Radians / s
        double maxPropulsion;               // arbitrary units
        double distanceFromCenter_Front;    // m, typically positive value
        double distanceFromCenter_Back;     // m, typically negative value
        double estimatedAcceleration;       // m / (s * s) / unit of propulsion
        double estimatedAngularAcceleration;// Radians / (s * s) / unit of propulsion
#endif
        double nearLimit;                   // Distance from target (m) where to change from "cruise" to "near"-mode

        double cruisePropulsion;
        double cruiseDirectionProp;
        PIDSettings pidSettings_Position;
        PIDSettings pidSettings_Heading;
    };

    struct Destination
    {
        double coord_N;     // North
        double coord_E;     // East
        double heading;     // Radians
    };

    struct Outputs
    {
        bool valid;
        double direction_Front;     // Radians as relative heading
        double propulsion_Front;    // Arbitrary units
        double direction_Back;
        double propulsion_Back;
    };

    enum State
    {
        STATE_UNKNOWN = 0,
        STATE_CRUISING,
        STATE_NEAR,
    };

    struct DebugOutputs
    {
        double absBearing;
        double relativeBearing;
        double distanceToTarget;
        Eigen::Vector2d velocityVec;
        double speed;
        double directionOfTravel;
        double headingError;
        State state;
    };

    Autopilot();
    Autopilot(const Settings& settings);
    void init(const Settings& settings);
    void setDestination(const Destination destination);
    void update(const Eigen::Transform<double, 3, Eigen::Affine>& transform, Outputs& outputs, double cycleTime, DebugOutputs* debugOutputs = nullptr);

private:

    Settings settings;
    Destination destination;
    Eigen::Vector2d lastOrigin_2D_NE;
    State state;

    MiniPID pid_Position_N;
    MiniPID pid_Position_E;
    MiniPID pid_Heading;
};

#endif // AUTOPILOT_H
