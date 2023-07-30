package org.firstinspires.ftc.teamcode.util.pid;

public class PoofyPIDCoefficients {

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kV = 0;
    private double kA = 0;
    private double kS = 0;
    private double kF = 0;

    public PoofyPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PoofyPIDCoefficients(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD);
        this.kF = kF;
    }

    public PoofyPIDCoefficients(double kP, double kI, double kD, double kV, double kA) {
        this(kP, kI, kD);
        this.kV = kV;
        this.kA = kA;
    }

    public PoofyPIDCoefficients(double kP, double kI, double kD, double kV, double kA, double kS, double kF) {
        this(kP, kI, kD, kV, kA);
        this.kS = kS;
        this.kF = kF;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

}
