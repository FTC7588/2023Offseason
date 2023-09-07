package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.vision.VisionPortal;

public abstract class CommandOpModeEx extends CommandOpMode {

    private boolean runOnce = false;

    public void initLoop() {}

    public void runInit() {}


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeInInit()) {
            initLoop();
        }

        while (!isStopRequested() && opModeIsActive()) {
            if (!runOnce) {
                runInit();
                runOnce = true;
            }
            run();
        }

        reset();
    }

}
