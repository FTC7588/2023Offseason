package org.firstinspires.ftc.teamcode.commandbase.subsystems;

public class Subsystems {

    private final DrivetrainSubsystem drive;
    private final ElevatorSubsystem ele;
    private final IntakeSubsystem intake;

    public Subsystems(
            DrivetrainSubsystem drive,
            ElevatorSubsystem ele,
            IntakeSubsystem intake
    ) {
        this.drive = drive;
        this.ele = ele;
        this.intake = intake;
    }

    public DrivetrainSubsystem getDrive() {
        return drive;
    }

    public ElevatorSubsystem getEle() {
        return ele;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }
}
