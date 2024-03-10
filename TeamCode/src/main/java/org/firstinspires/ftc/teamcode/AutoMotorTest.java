package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoMotorTest", group = "TEST")
@Disabled
public class AutoMotorTest extends LinearOpMode {
    DcMotor myMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        myMotor = hardwareMap.get(DcMotor.class, "My Motor");
        myMotor.setDirection(DcMotor.Direction.FORWARD);
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            myMotor.setPower(1);
            sleep(2000);
            myMotor.setPower(0);
            sleep(2000);
        }
    }
}