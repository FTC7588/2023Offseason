package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Test extends CommandOpMode {

    DcMotorEx motor;
    Servo grab;
    public static double open = 0.5;
    public static double closed = 0.25;

    @Override
    public void initialize() {
        motor = hardwareMap.get(DcMotorEx.class, "paraEncoder");
        grab = hardwareMap.get(Servo.class, "grab");
    }

    @Override
    public void run() {
        if (gamepad1.a) {
            motor.setPower(1);
        } else if (gamepad1.y) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }

        if (gamepad2.a) {
            grab.setPosition(closed);
        } else if (gamepad2.b) {
            grab.setPosition(open);
        } else if (gamepad2.y) {
            grab.setPosition(1-closed);
        }
    }
}
