package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ElevatorSubsystem;

public class MoveElevatorToPosition extends CommandBase {

    private final double target;
    private final ElevatorSubsystem elevatorSubsystem;

    public MoveElevatorToPosition(ElevatorSubsystem elevatorSubsystem, double target) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(this.elevatorSubsystem);
        this.target = target;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtGoal();
    }

}
