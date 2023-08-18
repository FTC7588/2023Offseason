package org.firstinspires.ftc.teamcode.utils.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.geometry.EulerAngles;

public class PoofyIMU implements IMU {

    private com.qualcomm.robotcore.hardware.IMU imu;
    private EulerAngles angles;

    public PoofyIMU(com.qualcomm.robotcore.hardware.IMU imu) {
        this.imu = imu;
    }

    @Override
    public void update() {
        angles = new EulerAngles(
                imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS),
                imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }

    @Override
    public double getRoll() {
        return angles.getRoll();
    }

    @Override
    public double getPitch() {
        return angles.getPitch();
    }

    @Override
    public double getYaw() {
        return angles.getYaw();
    }

    public double getRollDegrees() {
        return Math.toDegrees(getRoll());
    }

    public double getPitchDegrees() {
        return Math.toDegrees(getPitch());
    }

    public double getYawDegrees() {
        return Math.toDegrees(getYaw());
    }

    @Override
    public EulerAngles getAngles() {
        return angles;
    }
}
