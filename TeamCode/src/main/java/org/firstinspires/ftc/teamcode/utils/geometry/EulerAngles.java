package org.firstinspires.ftc.teamcode.utils.geometry;

public class EulerAngles {

    public double roll;
    public double pitch;
    public double yaw;

    public EulerAngles(double roll, double pitch, double yaw) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }
}
