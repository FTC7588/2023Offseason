package org.firstinspires.ftc.teamcode.utils.geometry;

import android.annotation.SuppressLint;

public class Transform2d {

    private final Vector2d vector;
    private final double rotation;

    public Transform2d(Pose2d initial, Pose2d last) {
        vector = last.getVector().subtract(initial.getVector())
                .rotateBy(-initial.getTheta());

        rotation = last.getTheta() - initial.getTheta();
    }

    public Transform2d(Vector2d vector, double rotation) {
        this.vector = vector;
        this.rotation = rotation;
    }

    public Transform2d(double x, double y, double rotation) {
        this.vector = new Vector2d(x, y);
        this.rotation = rotation;
    }

    public Transform2d() {
        this.vector = new Vector2d();
        this.rotation = 0;
    }

    public Transform2d times(double scalar) {
        return new Transform2d(vector.scale(scalar), rotation * scalar);
    }

    public Vector2d getVector() {
        return vector;
    }

    public double getRotation() {
        return rotation;
    }

    public Transform2d inverse() {
        return new Transform2d(
               getVector().unaryMinus().rotateBy(-getRotation()),
                -getRotation()
        );
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f°)", vector.getX(), vector.getY(), Math.toDegrees(rotation));
    }
}
