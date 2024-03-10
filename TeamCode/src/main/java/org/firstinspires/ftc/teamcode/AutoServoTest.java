package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoServoTest", group = "TEST")
@Disabled
public class AutoServoTest extends LinearOpMode {
    Servo myServo = null;
    CRServo chainServo = null;
    @Override
    public void runOpMode() throws InterruptedException {
        myServo = hardwareMap.get(Servo.class, "My Servo");
        chainServo = hardwareMap.get(CRServo.class, "ChainServo");

        myServo.setDirection(Servo.Direction.REVERSE);
        myServo.setPosition(-1);
        chainServo.setDirection(CRServo.Direction.FORWARD);
        chainServo.setPower(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            chainServo.setPower(1);
            sleep(5000);
        }
    }
}