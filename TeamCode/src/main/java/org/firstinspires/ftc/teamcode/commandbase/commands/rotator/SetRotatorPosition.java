package org.firstinspires.ftc.teamcode.commandbase.commands.rotator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.RotatorSubsystem;

public class SetRotatorPosition extends CommandBase {

    private final RotatorSubsystem m_rotatorSubsystem;
    private final double position;

    public SetRotatorPosition(RotatorSubsystem rotatorSubsystem, double position) {
        this.m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(m_rotatorSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {
        m_rotatorSubsystem.setTargetPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
