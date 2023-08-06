package org.firstinspires.ftc.teamcode.utils.pid;

public abstract class PoofyFeedForwardController {

    public abstract double calculate(double measuredPosition);

    public abstract void setTargetPosition(double targetPosition);

}

