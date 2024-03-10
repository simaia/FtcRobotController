package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleopArmTest", group = "TEST")
@Disabled
public class TeleopArmTest extends LinearOpMode {
    CRServo myCRServo = null;
    @Override
    public void runOpMode() throws InterruptedException {
        myCRServo = hardwareMap.get(CRServo.class, "My CR Servo");

        myCRServo.setDirection(CRServo.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a) {
                myCRServo.setPower(1);
            } else if(currentGamepad1.b) {
                myCRServo.setPower(-1);
            } else {
                myCRServo.setPower(0);
            }
        }
    }
}