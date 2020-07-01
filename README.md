# SimFerryController

This is a Qt/C++-project that is meant to be used with https://github.com/GNSS-Stylist/FerrySim_Godot. It does not make any sense to use this as a "standalone" app. This acts as a location/orientation solver that works on the data sent by the FerrySim_Godot. This also includes a really simple autopilot.

More info in youtube-video https://youtu.be/k7VmWJ6lLlM

Uses:
- Qt (https://www.qt.io)
- Eigen C++ template library for linear algebra (http://eigen.tuxfamily.org)
- MiniPID (https://github.com/tekdemo/MiniPID)
