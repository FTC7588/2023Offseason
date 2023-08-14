package org.firstinspires.ftc.teamcode.utils.geometry;

import android.annotation.SuppressLint;

public class Pose2d {
    public double x;
    public double y;
    public double theta;

    public Pose2d (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose2d (Vector2d xy, double theta) {
        this.x = xy.getX();
        this.y = xy.getY();
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public Vector2d getVector() {
        return new Vector2d(x, y);
    }

    public Vector2d headingVec() {
        return new Vector2d(Math.cos(theta), Math.sin(theta));
    }

    public Pose2d plus(Transform2d other) {
        return transformBy(other);
    }

    public Pose2d minus(Pose2d other) {
        return new Pose2d(this.x - other.x, this.y - other.y, this.theta - other.theta);
    }

    public Pose2d times(double scalar) {
        return new Pose2d(scalar * x, scalar * y, scalar * theta);
    }

    public Pose2d div(double scalar) {
        return new Pose2d(x / scalar, y / scalar, theta / scalar);
    }

    public Pose2d transformBy(Transform2d other) {
        return new Pose2d(
                new Vector2d(x, y).add(other.getVector().rotateBy(theta)),
                theta + other.getRotation()
        );
    }

    public Pose2d relativeTo(Pose2d other) {
        Transform2d transform2d = new Transform2d(other, this);
        return new Pose2d(transform2d.getVector(), transform2d.getRotation());
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f°)", x, y, Math.toDegrees(theta));
    }
}