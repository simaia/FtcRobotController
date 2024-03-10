package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp(name = "Newteleop", group = "PROD")
public class NewTeleop extends LinearOpMode {
    DcMotor motorRightFront = null;
    DcMotor motorRightBack = null;
    DcMotor motorLeftFront = null;
    DcMotor motorLeftBack = null;
    DcMotor motorBratBaza = null;
    DcMotor motorBratSus = null;
    CRServo servoIntake = null;
    CRServo servoDrona = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.get(DcMotor.class, "Right Front Motor");
        motorRightBack = hardwareMap.get(DcMotor.class, "Right Back Motor");
        motorLeftFront = hardwareMap.get(DcMotor.class, "Left Front Motor");
        motorLeftBack = hardwareMap.get(DcMotor.class, "Left Back Motor");
        motorBratBaza = hardwareMap.get(DcMotor.class,  "Motor Brat Baza");
        motorBratSus = hardwareMap.get(DcMotor.class,  "Motor Brat Sus");
        servoIntake = hardwareMap.get(CRServo.class, "Servo Intake");
        servoDrona = hardwareMap.get(CRServo.class, "Servo Drona");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorBratBaza.setDirection(DcMotor.Direction.FORWARD);
        motorBratSus.setDirection(DcMotor.Direction.FORWARD);
        servoIntake.setDirection(CRServo.Direction.FORWARD);

        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBratBaza.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBratSus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBratBaza.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBratSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            if (gamepad2.left_bumper) { //merge in sus
                motorBratBaza.setPower(0.5);
            } else if (gamepad2.right_bumper) { //merge in jos
                motorBratBaza.setPower(-0.5);
            } else if (gamepad2.dpad_down) { //merge puternic in jos (atarnare)
                motorBratBaza.setPower(-1);
            } else {
                motorBratBaza.setPower(0);
            }

            if (gamepad2.left_trigger > 0.5) {
                motorBratSus.setPower(0.5); //motor brat sus merge upwards
            } else if (gamepad2.left_trigger > 0 & gamepad2.left_trigger <= 0.5) {
                motorBratSus.setPower(gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0.5){
                motorBratSus.setPower(-0.5); //motor brat sus merge downwards
            } else if (gamepad2.right_trigger > 0 & gamepad2.right_trigger <= 0.5) {
                motorBratSus.setPower(-gamepad2.right_trigger);
            } else {
                motorBratSus.setPower(0);
            }

            if (gamepad2.x) { //se inchide
                servoIntake.setPower(0.5);
            } else if (gamepad2.b) { //se deschide
                servoIntake.setPower(-0.5);
            } else {
                servoIntake.setPower(0);
            }

            if (gamepad1.dpad_down) {
                servoDrona.setPower(-0.5);
            } else if (gamepad1.dpad_up) {
                servoDrona.setPower(0.5);
            } else {
                servoDrona.setPower(0);
            }
        }
    }
}