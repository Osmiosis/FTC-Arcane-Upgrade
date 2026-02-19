package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Mecanum Drive Autonomous")
//@Disabled
public class MecanumDriveAutonomousTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Them motors pls name them in runOpMode this :|
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;

    //How fast do u want to go gng?
    private final double DRIVE_POWER = 0.5;    //Power for forward/backward movement
    private final double ROTATE_POWER = 0.4;   //Power for rotation
    private final double STRAFE_POWER = 1;   //Power for strafing left/right

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //THE MOTOR NAMES OH MY DAYS
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        //Setting them directions for them motors
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        //So obviously the motors need to brake when u aint using them goated explanation by Aarav
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //We dont need no encoders for this simple auton + we gonna use odometry computer later
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Some cringe ass telemetry
        telemetry.addData("Status", "Initialized. Ready to run!");
        telemetry.addData("Info", "Press PLAY to start autonomous");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
          
            stopMotors();
            sleep(500);

            //Strafe left for 2 seconds
            telemetry.addData("Status", "Step 5: Strafing Left");
            telemetry.update();
            strafeLeft(STRAFE_POWER, 2.0);

            //Stop for a minute and SMILE
            stopMotors();
            sleep(500);

            //Strafe right for 2 seconds
            telemetry.addData("Status", "Step 6: Strafing Right");
            telemetry.update();
            strafeRight(STRAFE_POWER, 2.0);

            //Stop for a minute and SMILE
            stopMotors();
            sleep(500);

            //Drive forward for 3 seconds
            telemetry.addData("Status", "Step 7: Driving Forward");
            telemetry.update();
            driveForward(DRIVE_POWER, 3.0);

            //Stop drop and ROLL gg
            stopMotors();

            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.update();
        }
    }

//Boring functions I aint putting comments for each just fucking read the fucking name
    private void driveForward(double power, double timeSeconds) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);

        sleep((long)(timeSeconds * 1000));
    }


    private void driveBackward(double power, double timeSeconds) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(-power);

        sleep((long)(timeSeconds * 1000));
    }

    private void rotateRight(double power, double timeSeconds) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);

        sleep((long)(timeSeconds * 1000));
    }

    private void rotateLeft(double power, double timeSeconds) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);

        sleep((long)(timeSeconds * 1000));
    }

    private void strafeLeft(double power, double timeSeconds) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);

        sleep((long)(timeSeconds * 10000));
    }

    private void strafeRight(double power, double timeSeconds) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);

        sleep((long)(timeSeconds * 10000));
    }


    private void stopMotors() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
}
