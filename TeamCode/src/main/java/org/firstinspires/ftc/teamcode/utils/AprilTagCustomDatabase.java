package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagCustomDatabase {

    public static AprilTagLibrary getLargeLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(0,
                        "MEOW",
                        6.625,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .addTag(1,
                        "WOOF",
                        6.625,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .addTag(2,
                        "OINK",
                        6.625,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .addTag(3,
                        "RAWR",
                        6.625,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .build();
    }

    public static AprilTagLibrary getSmallLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(0,
                        "MEOW",
                        3.125,
                        new VectorF(30, 9.5F, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, Math.toRadians(180))
                )
                .addTag(1,
                        "WOOF",
                        3.125,
                        new VectorF(30, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, Math.toRadians(180))
                )
                .addTag(2,
                        "OINK",
                        3.125,
                        new VectorF(-30, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, Math.toRadians(0))
                )
                .addTag(3,
                        "RAWR",
                        3.125,
                        new VectorF(0, -30, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, Math.toRadians(90))
                )
                .build();
    }

    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(60.25f,41.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(60.25f,35.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(60.25f,29.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(60.25f,-29.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(60.25f,-35.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(60.25f,-41.41f,4f), DistanceUnit.INCH,
                        new Quaternion(0.683f,-0.183f,0.183f,0.683f,0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f,-40.625f,5.5f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f,-35.125f,4f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f,35.125f,4f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f,40.625f,5.5f), DistanceUnit.INCH,
                        new Quaternion(0.7071f,0,0,-7.071f,0))
                .build();
    }

}
