package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterPID {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private ElapsedTime timerS;

    private double lastShooterError = 0;
    private double shoot = 0;
    private double shooterTarget = Constants.SHOOTER_TARGET;
    private boolean targetChanged = false;


    public ShooterPID(DcMotorEx motor1, DcMotorEx motor2) {
        this.shooterMotor1 = motor1;
        this.shooterMotor2 = motor2;
        this.timerS = new ElapsedTime();
        timerS.reset();
    }


    public void setTarget(double target) {
        if (Math.abs(this.shooterTarget - target) > 0.1) {
            targetChanged = true;
        }
        this.shooterTarget = target;
    }

    private void calculatePID() {
        // Always calculate PID
        double currentVelocity = (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
        double error = shooterTarget - currentVelocity;

        // Calculate dt using milliseconds (more stable than microseconds)
        double dt = Math.max(timerS.milliseconds() / 1000.0, 1e-3);

        // Calculate derivative, but prevent derivative kick on target changes
        double derivative = targetChanged ? 0 : (error - lastShooterError) / dt;
        targetChanged = false;

        // Feedforward (scaled properly to motor max RPM)
        double maxRPM = 6000;
        shoot = Constants.Kp_Shoot * error + Constants.Kd_shoot * derivative + Constants.Kf_Shoot * (shooterTarget / maxRPM);

        // Clamp power to ±1 to prevent SDK overhead and jitter
        shoot = Math.max(-1.0, Math.min(shoot, 1.0));

        lastShooterError = error;

        // Reset timer at the end for next cycle
        timerS.reset();

        // Apply power to both motors
        shooterMotor1.setPower(shoot);
        shooterMotor2.setPower(shoot);
    }

    public double update() {
        // PID always runs
        calculatePID();
        return shoot;
    }

    public double updateAuto() {
        calculatePID();
        return shoot;
    }


    public double getOutput() {
        return shoot;
    }


    public double getError() {
        double avgVelocity = (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
        return shooterTarget - avgVelocity;
    }


    public double[] getMotorVelocities() {
        return new double[]{shooterMotor1.getVelocity(), shooterMotor2.getVelocity()};
    }

    public boolean atTarget() {
        return Math.abs(getError()) <= Constants.Shooter_Tolerance;
    }


    public void reset() {
        lastShooterError = 0;
        shoot = 0;
        timerS.reset();
    }

    public void stop() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        reset();
    }
}
