package org.firstinspires.ftc.teamcode.commandbase.subsystems;

public class Subsystems {

    private final DrivetrainSubsystem drive;
    private final ElevatorSubsystem ele;

    public Subsystems(
            DrivetrainSubsystem drive,
            ElevatorSubsystem ele
    ) {
        this.drive = drive;
        this.ele = ele;
    }

    public DrivetrainSubsystem getDrive() {
        return drive;
    }

    public ElevatorSubsystem getEle() {
        return ele;
    }

}
