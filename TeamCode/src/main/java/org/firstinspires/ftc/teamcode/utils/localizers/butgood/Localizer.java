package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;

public interface Localizer {

    Pose2d poseEstimate = null;

    Pose2d poseVelocity = null;

    void update();
}
