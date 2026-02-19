package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ShooterTest", group="TeleOp")
public class ShooterTest extends OpMode {

    private DcMotor shootleft;
    private DcMotor shootright;


    @Override
    public void init() {


        shootleft = hardwareMap.get(DcMotor.class, "shootleft");
        shootright = hardwareMap.get(DcMotor.class, "shootright");
        shootleft.setDirection(DcMotorSimple.Direction.REVERSE);



    }



    @Override
    public void loop() {
        shootleft.setPower(-1);
        shootright.setPower(-1);
    }
}

