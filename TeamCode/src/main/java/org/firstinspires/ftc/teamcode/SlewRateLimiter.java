package org.firstinspires.ftc.teamcode;

public class SlewRateLimiter {

    private final double maxRatePerSecond; // Maximum allowed change per second
    private double previousValue;          // Last output value
    private long previousTimeNanos;        // Last calculation time in nanoseconds
    private boolean isFirstCall;           // Track if this is the first calculation

    public SlewRateLimiter(double maxRatePerSecond) {
        this.maxRatePerSecond = Math.abs(maxRatePerSecond); // Ensure positive
        this.previousValue = 0.0;
        this.previousTimeNanos = System.nanoTime();
        this.isFirstCall = true;
    }


    public double calculate(double input) {
        // Get current time
        long currentTimeNanos = System.nanoTime();

        // On first call, just return the input and initialize
        if (isFirstCall) {
            previousValue = input;
            previousTimeNanos = currentTimeNanos;
            isFirstCall = false;
            return input;
        }

        // Calculate time elapsed in seconds
        double deltaTimeSeconds = (currentTimeNanos - previousTimeNanos) / 1_000_000_000.0;

        // Handle edge cases
        if (deltaTimeSeconds <= 0) {
            // No time has passed, return previous value
            return previousValue;
        }

        // Calculate maximum allowed change for this time step
        double maxChange = maxRatePerSecond * deltaTimeSeconds;

        // Calculate desired change
        double desiredChange = input - previousValue;

        // Clamp the change to the maximum allowed
        double limitedChange = Math.max(-maxChange, Math.min(maxChange, desiredChange));

        // Calculate new output value
        double output = previousValue + limitedChange;

        // Update state for next call
        previousValue = output;
        previousTimeNanos = currentTimeNanos;

        return output;
    }


    public void reset() {
        previousValue = 0.0;
        previousTimeNanos = System.nanoTime();
        isFirstCall = true;
    }

    public void reset(double value) {
        previousValue = value;
        previousTimeNanos = System.nanoTime();
        isFirstCall = false;
    }


    public double getLastValue() {
        return previousValue;
    }
}

