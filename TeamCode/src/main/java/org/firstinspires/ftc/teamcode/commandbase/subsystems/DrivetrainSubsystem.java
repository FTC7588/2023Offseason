package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.enums.DriveMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double heading;

    private final MecanumDrive drive;
    private DriveMode mode;

    public DrivetrainSubsystem(RobotHardware robot) {
        this.robot = robot;

        drive = new MecanumDrive(
                robot.fL,
                robot.fR,
                robot.rL,
                robot.rR,
                DRIVE_X_COEFFS,
                DRIVE_Y_COEFFS,
                DRIVE_THETA_COEFFS,
                DRIVE_MAX_TURN_SPEED_PID
        );


    }


    public void read() {
        heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void loop() {

    }

    public void write() {
        drive.write();
    }


    public void robotCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            drive.driveRobotCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed
            );
        } else {
            drive.driveRobotCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
        mode = DriveMode.ROBOT_CENTRIC;
    }

    public DriveMode getMode() {
        return mode;
    }

    public double getHeading() {
        return Math.toDegrees(heading);
    }

    public double getThetaTarget() {
        return drive.getThetaTarget();
    }

    public void setThetaTarget(double thetaTarget) {
        drive.setThetaTarget(thetaTarget);
    }
}
