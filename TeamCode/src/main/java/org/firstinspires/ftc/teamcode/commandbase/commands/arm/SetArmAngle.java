package org.firstinspires.ftc.teamcode.commandbase.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ArmSubsystem;

public class SetArmAngle extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double angleTarget;

    public SetArmAngle(ArmSubsystem armSubsystem, double angleTarget) {
        this.armSubsystem = armSubsystem;
        addRequirements(this.armSubsystem);
        this.angleTarget = angleTarget;
    }

    @Override
    public void initialize() {
        armSubsystem.setArmAngle(angleTarget);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isFinished();
    }

}
