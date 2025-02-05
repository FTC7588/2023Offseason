package org.firstinspires.ftc.teamcode.utils.pid;

public class PoofyPIDController extends PoofyFeedForwardController {

    private double kP;
    private double kI;
    private double kD;
    private double kV;
    private double kA;
    private double kS;
    private double kF;

    private double angle;

    private double targetPosition;
    private double targetVelocity;
    private double targetAcceleration;

    private double previousError;
    private double positionError;
    private double totalError;
    private double velocityError;

    private double lastTimeStamp;
    private double period;

    private double positionTolerance;
    private double velocityTolerance;

    private boolean angleFeedforward = false;

    private boolean inputBounded = false;
    private double minInput;
    private double maxInput;

    private boolean outputBounded = false;
    private double minOutput;
    private double maxOutput;

    private boolean integralBounded = false;
    private double minIntegral;
    private double maxIntegral;

    public PoofyPIDController(PoofyPIDCoefficients coeffs) {
        this.kP = coeffs.getkP();
        this.kI = coeffs.getkI();
        this.kD = coeffs.getkD();
        this.kV = coeffs.getkV();
        this.kA = coeffs.getkA();
        this.kS = coeffs.getkS();
        this.kF = coeffs.getkF();
    }

    public PoofyPIDController(double kP, double kI, double kD, double kV, double kA, double kS, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kF = kF;
    }

    public double calculate(double measuredPosition, double targetPosition) {
        setTargetPosition(targetPosition);
        return calculate(measuredPosition);
    }

    @Override
    public double calculate(double measuredPosition) {

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        positionError = calculatePositionError(targetPosition, measuredPosition);
        integrate(positionError, period);
        calculateVelocityError(positionError, period);
        previousError = positionError;
        double output = positionError * kP
                + totalError * kI
                + velocityError * kD
                + targetVelocity * kV
                + targetAcceleration * kA;

        if (angleFeedforward) {
            output += kF * getSIN(angle);
        } else {
            output += kF;
        }

        if (output == 0.0) {
            output = 0;
        } else {
            output += getSign(output) * kS;
        }

        if (outputBounded) {
            output = Math.max(minOutput, Math.min(output, maxOutput));
        }

        return output;
    }

    private double getSIN(double angle) {
        if (angle >= 90) {
            angle = 180 - angle;
        } else if (angle <= -90) {
            angle = -180 - angle;
        }
        return Math.sin(Math.toRadians(angle));
    }

    public boolean atSetPoint() {
        return Math.abs(positionError) < positionTolerance
                && Math.abs(velocityError) < velocityTolerance;
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setTargetAcceleration(double targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }

    public void enableAngleFeedforward() {
        angleFeedforward = true;
    }

    public void disableAngleFeedforward() {
        angleFeedforward = false;
    }

    public void setAngle(double angleDeg) {
        angle = angleDeg;
    }

    //continuous
    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    public void setIntegrationBounds(double min, double max) {
        if (min < max) {
            integralBounded = true;
            minIntegral = min;
            maxIntegral = max;
        }
    }

    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    private double calculatePositionError(double reference, double state) {
        double error = reference - state;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (Math.abs(error) > inputRange / 2.0) {
                error -= getSign(error) * inputRange;
            }
        }
        return error;
    }

    private void integrate(double error, double dt) {
        totalError += ((error + previousError) / 2) * dt;
        if (integralBounded) {
            totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);
        }
    }

    private void calculateVelocityError(double error, double dt) {
        velocityError = (error - previousError) / dt;
    }

    private double getSign(double number) {
        if (number < 0) {
            return -1;
        } else if (number == 0) {
            return 0;
        } else {
            return 1;
        }
    }

    public double getPositionError() {
        return positionError;
    }

    public double getVelocityError() {
        return velocityError;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getPeriod() {
        return period;
    }

    public boolean isInputBounded() {
        return inputBounded;
    }

    public void reset() {
        totalError = 0;
        lastTimeStamp = 0;
        previousError = 0;
    }



    public static class Builder {
        private double kP = 0;
        private double kI = 0;
        private double kD = 0;
        private double kV = 0;
        private double kA = 0;
        private double kS = 0;
        private double kF = 0;

        public PoofyPIDController.Builder addP(double kP) {
            this.kP = kP;
            return this;
        }

        public PoofyPIDController.Builder addI(double kI) {
            this.kI = kI;
            return this;
        }

        public PoofyPIDController.Builder addD(double kD) {
            this.kD = kD;
            return this;
        }

        public PoofyPIDController.Builder addV(double kV) {
            this.kV = kV;
            return this;
        }

        public PoofyPIDController.Builder addA(double kA) {
            this.kA = kA;
            return this;
        }

        public PoofyPIDController.Builder addS(double kS) {
            this.kS = kS;
            return this;
        }

        public PoofyPIDController.Builder addF(double kF) {
            this.kF = kF;
            return this;
        }

        public PoofyPIDController.Builder addPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        public PoofyPIDController.Builder addVA(double kV, double kA) {
            this.kV = kV;
            this.kA = kA;
            return this;
        }

        public PoofyPIDController build() {
            return new PoofyPIDController(kP, kI, kD, kV, kA, kS, kF);
        }
    }
}
