package org.firstinspires.ftc.teamcode.commandbase.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;

import java.util.function.DoubleSupplier;

public class FollowTag extends CommandBase {

    private final Pose2d followPose;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public FollowTag(DrivetrainSubsystem drivetrainSubsystem, Pose2d followPose) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.followPose = followPose;
    }

    @Override
    public void execute() {
        //m_drivetrainSubsystem.followTagMode(followPose);
    }

}
