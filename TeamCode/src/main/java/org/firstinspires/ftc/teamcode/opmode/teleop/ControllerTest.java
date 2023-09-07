package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ControllerTest extends LinearOpMode {

    public Gamepad g;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        g = gamepad1;

        boolean once = false;

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            if (g.cross) {
                tal("cross");
            }
            if (g.circle) {
                tal("circle");
            }
            if (g.square) {
                tal("square");
            }
            if (g.triangle) {
                tal("triangle");
            }
            if (g.a) {
                tal("a");
            }
            if (g.b) {
                tal("b");
            }
            if (g.x) {
                tal("x");
            }
            if (g.y) {
                tal("y");
            }
            if (g.dpad_down) {
                tal("dpad down");
            }
            if (g.dpad_left) {
                tal("dpad left");
            }
            if (g.dpad_right) {
                tal("dpad right");
            }
            if (g.dpad_up) {
                tal("dpad up");
            }
            if (g.left_bumper) {
                tal("left bumper");
            }
            if (g.right_bumper) {
                tal("right bumper");
            }
            if (g.left_stick_button) {
                tal("left stick button");
            }
            if (g.right_stick_button) {
                tal("right stick button");
            }
            if (g.options) {
                tal("options");
            }
            if (g.share) {
                tal("share");
            }
            if (g.touchpad_finger_1) {
                tal("touchpad 1");
            }
            if (g.touchpad_finger_2) {
                tal("touchpad 2");
            }
            if (g.touchpad) {
                tal("touchpad");
            }
            if (g.guide) {
                tal("guide");
            }
            if (g.ps) {
                tal("ps");
            }

            if (g.cross) {
                g.rumble(100, 100, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }

            if (g.square && !once) {
                g.rumbleBlips(2);
                once = true;
            }

            if (g.circle && !once) {
                g.rumbleBlips(5);
                once = true;
            }

            if (g.atRest()) {
                tal("nothing pressed");
            }

            if (g.triangle) {
                g.stopRumble();
            }

            if (g.dpad_up) {
                once = false;
            }

            telemetry.addLine("running");
            telemetry.addLine(String.format("touchpad 1 XY %6.1f %6.1f", g.touchpad_finger_1_x, g.touchpad_finger_1_y));
            telemetry.addLine(String.format("touchpad 2 XY %6.1f %6.1f", g.touchpad_finger_2_x, g.touchpad_finger_2_y));
            telemetry.addData("rumble finish ETA", g.nextRumbleApproxFinishTime);


            telemetry.update();

        }

    }

    public void tal(String string) {
        telemetry.addLine(string);
    }

}
