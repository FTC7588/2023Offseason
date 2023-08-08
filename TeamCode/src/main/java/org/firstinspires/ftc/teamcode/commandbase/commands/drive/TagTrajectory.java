package org.firstinspires.ftc.teamcode.commandbase.commands.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.RRDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.SampleMecanumDrive;

public class TagTrajectory extends CommandBase {

    private Pose2d target;
    private SampleMecanumDrive m_drivetrainSubsystem;

    private Trajectory traj;

    public TagTrajectory(SampleMecanumDrive drivetrainSubsystem, Pose2d target) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        //addRequirements(m_drivetrainSubsystem);
        this.target = target;
    }

    @Override
    public void initialize() {
        traj = m_drivetrainSubsystem.trajectoryBuilder(m_drivetrainSubsystem.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(0, -40))
                .build();
        m_drivetrainSubsystem.followTrajectoryAsync(traj);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !m_drivetrainSubsystem.isBusy();
    }

}
