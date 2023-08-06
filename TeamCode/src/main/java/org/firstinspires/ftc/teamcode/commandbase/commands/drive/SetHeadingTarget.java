package org.firstinspires.ftc.teamcode.commandbase.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DrivetrainSubsystem;

public class SetHeadingTarget extends CommandBase {

    private double target;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public SetHeadingTarget(DrivetrainSubsystem drivetrainSubsystem, double target) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.target = target;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setThetaTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
