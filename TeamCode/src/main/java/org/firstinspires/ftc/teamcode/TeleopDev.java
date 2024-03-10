package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleopDev", group = "DEV")
@Disabled
public class TeleopDev extends LinearOpMode {
    DcMotor motorRightFront = null;
    DcMotor motorRightBack = null;
    DcMotor motorLeftFront = null;
    DcMotor motorLeftBack = null;
    CRServo servoArmBaseRight = null;
    CRServo servoArmBaseLeft = null;
    CRServo servoArmTop = null;
    CRServo servoIntake = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.get(DcMotor.class, "Right Front Motor");
        motorRightBack = hardwareMap.get(DcMotor.class, "Right Back Motor");
        motorLeftFront = hardwareMap.get(DcMotor.class, "Left Front Motor");
        motorLeftBack = hardwareMap.get(DcMotor.class, "Left Back Motor");
        servoArmBaseRight = hardwareMap.get(CRServo.class, "Arm Base Servo Right");
        servoArmBaseLeft = hardwareMap.get(CRServo.class, "Arm Base Servo Left");
        servoArmTop = hardwareMap.get(CRServo.class, "Arm Top Servo");
        servoIntake = hardwareMap.get(CRServo.class, "Intake Servo");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        servoArmBaseRight.setDirection(CRServo.Direction.REVERSE);
        servoArmBaseLeft.setDirection(CRServo.Direction.FORWARD);
        servoArmTop.setDirection(CRServo.Direction.REVERSE);
        servoIntake.setDirection(CRServo.Direction.REVERSE);

        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            // NAVIGATION Controller

            /*

            double max;

            double axial   = -currentGamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  currentGamepad1.left_stick_x;
            double yaw     =  currentGamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // Normalization of values
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            motorLeftFront.setPower(leftFrontPower);
            motorRightFront.setPower(rightFrontPower);
            motorLeftBack.setPower(leftBackPower);
            motorRightBack.setPower(rightBackPower);

             */

            // SERVO Controller

            /*

            double servoArmBaseAxial = -currentGamepad1.left_stick_y;
            double servoArmTopAxial  = -currentGamepad1.right_stick_y;

            if(currentGamepad1.a) {
                servoIntake.setPower(1);
            } else if (currentGamepad1.b) {
                servoIntake.setPower(-1);
            } else {
                servoIntake.setPower(0);
            }

            servoArmBase.setPower(servoArmBaseAxial);
            servoArmTop.setPower(servoArmTopAxial);

             */

            // UNIFIED Controller

            double max;

            double axial   = -currentGamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  currentGamepad1.left_stick_x;
            double yaw     =  currentGamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // Normalization of values
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            motorLeftFront.setPower(leftFrontPower);
            motorRightFront.setPower(rightFrontPower);
            motorLeftBack.setPower(leftBackPower);
            motorRightBack.setPower(rightBackPower);

            if(currentGamepad1.a) {
                servoIntake.setPower(1);
            } else if (currentGamepad1.b) {
                servoIntake.setPower(-1);
            } else {
                servoIntake.setPower(0);
            }

            if(currentGamepad1.left_bumper) {
                servoArmTop.setPower(0.7);
            } else if (currentGamepad1.right_bumper) {
                servoArmTop.setPower(-0.45);
            } else {
                servoArmTop.setPower(0.07);
            }

            servoArmBaseRight.setPower(currentGamepad1.left_trigger - currentGamepad1.right_trigger);
            servoArmBaseLeft.setPower(currentGamepad1.left_trigger - currentGamepad1.right_trigger);
        }
    }
}