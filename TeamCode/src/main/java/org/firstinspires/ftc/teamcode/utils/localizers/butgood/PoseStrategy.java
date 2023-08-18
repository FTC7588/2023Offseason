package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

public enum PoseStrategy {
    /** Choose the Pose with the lowest ambiguity. */
    LOWEST_AMBIGUITY,

    /** Choose the Pose which is closest to the camera height. */
    CLOSEST_TO_CAMERA_HEIGHT,

    /** Choose the Pose which is closest to the last pose calculated */
    CLOSEST_TO_LAST_POSE,

    /** Return the average of the best target poses using ambiguity as weight. */
    AVERAGE_BEST_TARGETS,

    /** Use all visible tags to compute a single pose estimate.. */
    MULTI_TAG_PNP
}
