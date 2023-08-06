package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {

    public void initLoop() {}

    public void runInit() {}

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeInInit()) {
            initLoop();
        }

        runInit();

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }

}
