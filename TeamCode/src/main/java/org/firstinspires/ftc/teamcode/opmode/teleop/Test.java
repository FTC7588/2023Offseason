package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Test extends CommandOpMode {

    DcMotorEx motor;

    @Override
    public void initialize() {
        motor = hardwareMap.get(DcMotorEx.class, "paraEncoder");
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
    }
}
