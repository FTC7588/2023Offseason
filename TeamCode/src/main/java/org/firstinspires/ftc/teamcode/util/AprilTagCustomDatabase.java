package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
                        3.1,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .addTag(1,
                        "WOOF",
                        3.1,
                        new VectorF(9.5F, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .addTag(2,
                        "OINK",
                        3.1,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .addTag(3,
                        "RAWR",
                        3.1,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        MathUtil.eulerToQuaternion(0, 0, 0)
                )
                .build();
    }

}
