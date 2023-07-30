package org.firstinspires.ftc.teamcode.util.pid;

public abstract class PoofyFeedForwardController {

    public abstract double calculate(double measuredPosition);

    public abstract void setTargetPosition(double targetPosition);

}

