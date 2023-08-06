package org.firstinspires.ftc.teamcode.util;

public class CameraIntrinsics {

    double fx, fy, cx, cy;

    public CameraIntrinsics(double fx, double fy, double cx, double cy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    public double getFx() {
        return fx;
    }

    public double getFy() {
        return fy;
    }

    public double getCx() {
        return cx;
    }

    public double getCy() {
        return cy;
    }

    public double[] getIntrinsics() {
        return new double[] {fx, fy, cx, cy};
    }
}
