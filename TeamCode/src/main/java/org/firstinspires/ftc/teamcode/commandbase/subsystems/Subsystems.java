package org.firstinspires.ftc.teamcode.commandbase.subsystems;

public class Subsystems {

    //private final DrivetrainSubsystem drive;
    private final ElevatorSubsystem ele;

    public Subsystems(
            ElevatorSubsystem ele
    ) {
        this.ele = ele;
    }

    public ElevatorSubsystem getEle() {
        return ele;
    }

}
