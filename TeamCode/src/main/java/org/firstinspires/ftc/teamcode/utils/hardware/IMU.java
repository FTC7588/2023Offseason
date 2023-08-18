package org.firstinspires.ftc.teamcode.utils.hardware;

import org.firstinspires.ftc.teamcode.utils.geometry.EulerAngles;

interface IMU {

    void update();

    double getRoll();

    double getPitch();

    double getYaw();

    EulerAngles getAngles();

}
