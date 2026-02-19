package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot TeleOp", group="TeleOp")
public class Motor extends OpMode {

    private DcMotor climb;

    // Toggle state for main

    @Override
    public void init() {

        // Initialize Climb Motor
        climb = hardwareMap.get(DcMotor.class, "climb");
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Localization (AprilTag vision)


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive", "Ready");
        telemetry.addData("Shooter", "Ready");
        telemetry.addData("Shooter Intake", "Ready");
        telemetry.addData("Main Intake", "Ready");
        telemetry.addData("Climb", "Ready");
        telemetry.addData("Localization", "Ready");
        telemetry.update();
    }



    @Override
    public void loop() {
        // Right Bumper to toggle between AprilTag 20 and 24


        // Climb Control - Y button (up), A button (down)
        if (gamepad1.y) {
            climb.setPower(Constants.CLIMB_POWER);
        } else if (gamepad1.a) {
            climb.setPower(-Constants.CLIMB_POWER);
        } else {
            climb.setPower(0);
        }

        // Display Drive Telemetry




        // Display Shooter Telemetry

    }
}

