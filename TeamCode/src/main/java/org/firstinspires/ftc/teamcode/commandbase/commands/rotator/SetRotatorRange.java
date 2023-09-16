package org.firstinspires.ftc.teamcode.commandbase.commands.rotator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.RotatorSubsystem;

public class SetRotatorRange extends CommandBase {

    private final PwmControl.PwmRange range;
    private final RotatorSubsystem m_rotatorSubsystem;

    public SetRotatorRange(RotatorSubsystem rotatorSubsystem, PwmControl.PwmRange range) {
        m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(m_rotatorSubsystem);
        this.range = range;
    }

    @Override
    public void initialize() {
        m_rotatorSubsystem.setPWMRange(range);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
