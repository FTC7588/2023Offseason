package org.firstinspires.ftc.teamcode.util.filters;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {

    private final int windowSize;
    private final Queue<Double> buffer;
    private double sum;
    private double avg;

    public MovingAverage(int windowSize) {
        this.windowSize = windowSize;
        this.buffer = new LinkedList<>();
        this.sum = 0;
    }

    public void addNumber(double num) {
        buffer.add(num);
        sum += num;

        if (buffer.size() > windowSize) {
            sum -= buffer.poll();
        }

        avg = sum / buffer.size();
    }

    public void reset() {
        buffer.clear();
        sum = 0;
    }

    public double getSum() {
        return sum;
    }

    public double getAverage() {
        return avg;
    }
}
