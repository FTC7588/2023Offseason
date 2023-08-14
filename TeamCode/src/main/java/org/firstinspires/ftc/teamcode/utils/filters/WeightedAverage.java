package org.firstinspires.ftc.teamcode.utils.filters;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;

import java.util.Arrays;

public class WeightedAverage {

//    public static void main(String[] args) {
//        Pose3d test = new Pose3d(
//                new Vector3d(10, 10, 10),
//                new Rotation3d(0, 0, 0)
//        );
//
//        Pose3d big = new Pose3d(
//                new Vector3d(30, 30, 30),
//                new Rotation3d(0, 0, 0)
//        );
//
//        Pose3d[] poses = new Pose3d[]{test, big};
//        Pose3d weighted = getWeightedAverage(poses, 2);
//        System.out.println(weighted.toString());
//    }


    public static Pose3d getWeightedAverage3d(Pose3d[] poses, double strength) {
        if (poses.length == 1) {
            return poses[0];
        } else {
            //init new pose variables
            double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;

            //initialize weights
            double[] weights = new double[poses.length];
            double maxWeight = 0;

            for (Pose3d pose : poses) {
                double w = pythagoreanTheorem(pose.getX(), pose.getY(), pose.getZ());
                if (w > maxWeight) {
                    maxWeight = w;
                }
            }

            double maxStrength = maxWeight * strength;

            for (int i = 0; i < poses.length; i++) {
                weights[i] = maxStrength - Math.abs(pythagoreanTheorem(poses[i].getX(), poses[i].getY(), poses[i].getZ()));

                x += poses[i].getVector().getX() * weights[i];
                y += poses[i].getVector().getY() * weights[i];
                z += poses[i].getVector().getZ() * weights[i];
                roll += poses[i].getRotation().getX() * weights[i];
                pitch += poses[i].getRotation().getY() * weights[i];
                yaw += poses[i].getRotation().getZ() * weights[i];
            }

            double weightsSum = Arrays.stream(weights).sum();

            x /= weightsSum;
            y /= weightsSum;
            z /= weightsSum;
            roll /= weightsSum;
            pitch /= weightsSum;
            yaw /= weightsSum;

            return new Pose3d(
                    new Vector3d(x, y, z),
                    new Rotation3d(roll, pitch, yaw)
            );
        }

    }

    public static Pose2d getWeightedAverage2d(Pose2d[] poses, double strength) {
        if (poses.length == 1) {
            return poses[0];
        } else {
            //init new pose variables
            double x = 0, y = 0, yaw = 0;

            //initialize weights
            double[] weights = new double[poses.length];
            double maxWeight = 0;

            for (Pose2d pose : poses) {
                double w = pythagoreanTheorem(pose.getX(), pose.getY());
                if (w > maxWeight) {
                    maxWeight = w;
                }
            }

            double maxStrength = maxWeight * strength;

            for (int i = 0; i < poses.length; i++) {
                weights[i] = maxStrength - Math.abs(pythagoreanTheorem(poses[i].getX(), poses[i].getY()));

                x += poses[i].getVector().getX() * weights[i];
                y += poses[i].getVector().getY() * weights[i];
                yaw += poses[i].getTheta() * weights[i];
            }

            double weightsSum = Arrays.stream(weights).sum();

            x /= weightsSum;
            y /= weightsSum;
            yaw /= weightsSum;

            return new Pose2d(x, y, yaw);
        }
    }

    private static double pythagoreanTheorem(double x, double y) {
        double v = Math.abs(Math.pow(x, 2)) + Math.abs(Math.pow(y, 2));
        //System.out.println(Math.sqrt(v));
        return Math.sqrt(v);
    }

    private static double pythagoreanTheorem(double x, double y, double z) {
        double v = Math.abs(Math.pow(x, 2)) + Math.abs(Math.pow(y, 2)) + Math.abs(Math.pow(z, 2));
        //System.out.println(Math.sqrt(v));
        return Math.sqrt(v);
    }
}
