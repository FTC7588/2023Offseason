package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.utils.pid.PoofyPIDCoefficients;
import org.firstinspires.ftc.teamcode.utils.pid.PoofyPIDController;

public class MecanumDrive {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private double frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed;
    private double maxOutput = 1;

    private final PoofyPIDController xController;
    private final PoofyPIDController yController;
    private final PoofyPIDController thetaController;
    private double thetaTarget;
    private double maxTurnSpeedPID;

    private double turnSpeed;
    private double theta;

    private Pose2d currentPose;

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        xController = new PoofyPIDController.Builder().build();
        yController = new PoofyPIDController.Builder().build();
        thetaController = new PoofyPIDController.Builder().build();
        currentPose = new Pose2d(0, 0, 0);
    }

    public MecanumDrive(DcMotorEx frontLeft,
                        DcMotorEx frontRight,
                        DcMotorEx backLeft,
                        DcMotorEx backRight,
                        PoofyPIDCoefficients xCoeffs,
                        PoofyPIDCoefficients yCoeffs,
                        PoofyPIDCoefficients thetaCoeffs,
                        double maxTurnSpeedPID
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        xController = new PoofyPIDController(xCoeffs);
        yController = new PoofyPIDController(yCoeffs);

        thetaController = new PoofyPIDController(thetaCoeffs);
        thetaController.setInputBounds(-Math.PI, Math.PI);
        thetaController.setOutputBounds(-1, 1);

        this.maxTurnSpeedPID = maxTurnSpeedPID;

        currentPose = new Pose2d(0, 0, 0);
    }

    public void setMaxSpeed(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;

        frontLeftSpeed = frontLeftPower;
        backLeftSpeed = backLeftPower;
        frontRightSpeed = frontRightPower;
        backRightSpeed = backRightPower;
    }

    public void driveRobotCentricPID(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        thetaTarget -= turnSpeed * maxTurnSpeedPID;
        turnSpeed = -thetaController.calculate(gyroAngle, Math.toRadians(thetaTarget));

        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(turnSpeed), 1);
        double frontLeftPower = (forwardSpeed + strafeSpeed + turnSpeed) / denominator;
        double backLeftPower = (forwardSpeed - strafeSpeed + turnSpeed) / denominator;
        double frontRightPower = (forwardSpeed - strafeSpeed - turnSpeed) / denominator;
        double backRightPower = (forwardSpeed + strafeSpeed - turnSpeed) / denominator;

        frontLeftSpeed = frontLeftPower;
        backLeftSpeed = backLeftPower;
        frontRightSpeed = frontRightPower;
        backRightSpeed = backRightPower;
    }

    public void driveFieldCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double turnSpeed,
                                  double gyroAngle
    ) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        driveRobotCentric(
                input.getX(),
                input.getY(),
                turnSpeed
        );
    }

    public void driveFieldCentricPID(double strafeSpeed,
                                  double forwardSpeed,
                                  double turnSpeed,
                                  double gyroAngle
    ) {
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        driveRobotCentricPID(
                input.getX(),
                input.getY(),
                turnSpeed,
                gyroAngle
        );
    }

    public void drivePointCentric(double strafeSpeed,
                                  double forwardSpeed,
                                  double gyroAngle,
                                  Vector2d target,
                                  Pose2d currentPose,
                                  double angleOffset
    ) {

        this.currentPose = currentPose;

        Vector2d difference = target.subtract(currentPose.getVector());

        theta = difference.getAngle() + Math.toRadians(angleOffset);

        turnSpeed = thetaController.calculate(theta, currentPose.getTheta());

        double turnError = Math.abs(theta = currentPose.getTheta());
//
//        strafeSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));
//        forwardSpeed *= Math.cos(Range.clip(turnError, -Math.PI/2, Math.PI/2));

        driveFieldCentric(
                strafeSpeed,
                forwardSpeed,
                -turnSpeed,
                gyroAngle
        );
    }

    public void driveFollowTag(Pose2d tagPose,
                               Pose2d targetFollowingPose
    ) {
        xController.setTargetPosition(targetFollowingPose.getX());
        double strafeSpeed = xController.calculate(tagPose.getX());

        yController.setTargetPosition(targetFollowingPose.getY());
        double forwardSpeed = yController.calculate(tagPose.getY());

        double turnSpeed = thetaController.calculate(Math.toRadians(targetFollowingPose.getTheta()), Math.toRadians(tagPose.getTheta()));

        driveRobotCentric(
                strafeSpeed,
                -forwardSpeed,
                turnSpeed
        );
    }



    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    public double getThetaTarget() {
        return thetaTarget;
    }

    public void setThetaTarget(double thetaTarget) {
        this.thetaTarget = thetaTarget;

    }

    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        this.frontLeft
                .setPower(frontLeftSpeed * maxOutput);
        this.frontRight
                .setPower(frontRightSpeed * maxOutput);
        this.backLeft
                .setPower(backLeftSpeed * maxOutput);
        this.backRight
                .setPower(backRightSpeed * maxOutput);
    }

    public void write() {
        driveWithMotorPowers(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

}
