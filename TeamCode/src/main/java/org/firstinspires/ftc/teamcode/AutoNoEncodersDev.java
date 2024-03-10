package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoNoEncodersDev", group = "DEV")
@Disabled
public class AutoNoEncodersDev extends LinearOpMode {

    // CONSTANTS
    final double MM_PER_MS = 2;
    final double INCH_TO_MM = 25.4;
    final double DEGREES_PER_MS = 0.16;

    // DERIVED CONSTANTS

    final double MS_PER_MM = 1.0 / MM_PER_MS;
    final double MS_PER_DEGREE = 1.0 / DEGREES_PER_MS;

    // IMPORTANT POINTS
    final Position backstageBlue = new Position(22.75 * 0.5 * INCH_TO_MM, 24 * 0.5 * INCH_TO_MM);
    final Position backstageRed = new Position(22.75 * 5.5 * INCH_TO_MM, 24 * 0.5 * INCH_TO_MM);
    final Position startBlueBackstage = new Position(9 * INCH_TO_MM, 24 * 2.5 * INCH_TO_MM);
    final Position startBluePublic = new Position(9 * INCH_TO_MM, 24 * 4.5 * INCH_TO_MM);
    final Position startRedBackstage = new Position((22.75 * 6 - 9) * INCH_TO_MM, 24 * 2.5 * INCH_TO_MM);
    final Position startRedPublic = new Position((22.75 * 6 - 9) * INCH_TO_MM, 24 * 4.5 * INCH_TO_MM);
    DcMotor motorRightFront = null;
    DcMotor motorRightBack = null;
    DcMotor motorLeftFront = null;
    DcMotor motorLeftBack = null;
    Rev2mDistanceSensor sensorLeft = null;
    Rev2mDistanceSensor sensorRight = null;
    RevColorSensorV3 sensorColour = null;
    Position currentPosition = null;
    Direction currentDirection = null;
    Team currentTeam = null;
    StartingLocation currentStartingLocation = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.get(DcMotor.class, "Right Front Motor");
        motorRightBack = hardwareMap.get(DcMotor.class, "Right Back Motor");
        motorLeftFront = hardwareMap.get(DcMotor.class, "Left Front Motor");
        motorLeftBack = hardwareMap.get(DcMotor.class, "Left Back Motor");
        // servoArmBase = hardwareMap.get(CRServo.class, "Arm Base Servo");
        // servoArmTop = hardwareMap.get(Servo.class, "Arm Top Servo");
        // sensorLeft = hardwareMap.get(Rev2mDistanceSensor.class, "Left Sensor");
        // sensorRight = hardwareMap.get(Rev2mDistanceSensor.class, "Right Sensor");
        // sensorColour = hardwareMap.get(RevColorSensorV3.class, "Colour Sensor");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initialize(Team.RED, StartingLocation.PUBLIC, Randomization.A);
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Front Motor Position", motorLeftFront.getCurrentPosition());
            telemetry.update();

            moveToBackstage();
            sleep(30000);
        }
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public enum Rotation {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    public enum Team {
        BLUE,
        RED
    }

    public enum StartingLocation {
        BACKSTAGE,
        PUBLIC
    }

    public enum Randomization {
        A,
        B,
        C
    }

    public static class Position {
        double x;
        double y;
        public Position(double x0, double y0) {
            x = x0;
            y = y0;
        }
    }

    public void initialize(Team team, StartingLocation location, Randomization randomization) {
        if(team == Team.RED && location == StartingLocation.PUBLIC) {
            currentPosition = startRedPublic;
            currentDirection = Direction.LEFT;
            currentTeam = Team.RED;
            currentStartingLocation = StartingLocation.PUBLIC;
        }
        if(team == Team.RED && location == StartingLocation.BACKSTAGE) {
            currentPosition = startRedBackstage;
            currentDirection = Direction.LEFT;
            currentTeam = Team.RED;
            currentStartingLocation = StartingLocation.BACKSTAGE;
        }
        if(team == Team.BLUE && location == StartingLocation.PUBLIC) {
            currentPosition = startBluePublic;
            currentDirection = Direction.RIGHT;
            currentTeam = Team.BLUE;
            currentStartingLocation = StartingLocation.PUBLIC;
        }
        if(team == Team.BLUE && location == StartingLocation.BACKSTAGE) {
            currentPosition = startBlueBackstage;
            currentDirection = Direction.RIGHT;
            currentTeam = Team.BLUE;
            currentStartingLocation = StartingLocation.BACKSTAGE;
        }
    }

    public void moveToBackstage() {
        if (currentTeam == Team.RED) {
            moveToPosition(backstageRed);
        }
        if (currentTeam == Team.BLUE) {
            moveToPosition(backstageBlue);
        }
    }

    public void moveToPosition(Position target) {
        double delta_x = target.x - currentPosition.x;
        double delta_y = target.y - currentPosition.y;
        if (delta_x < 0) {
            changeDirection(Direction.LEFT);
            move(Direction.FORWARD, -delta_x);
        }
        if (delta_x > 0) {
            changeDirection(Direction.RIGHT);
            move(Direction.FORWARD, delta_x);
        }
        if (delta_y < 0) {
            changeDirection(Direction.FORWARD);
            move(Direction.FORWARD, -delta_y);
        }
        if (delta_x > 0) {
            changeDirection(Direction.BACKWARD);
            move(Direction.FORWARD, delta_y);
        }
    }

    public void changeDirection(Direction target) {
        switch(currentDirection) {
            case LEFT: {
                switch(target) {
                    case FORWARD: {
                        rotate(Rotation.CLOCKWISE, 90);
                        break;
                    }
                    case RIGHT: {
                        rotate(Rotation.CLOCKWISE, 180);
                        break;
                    }
                    case BACKWARD: {
                        rotate(Rotation.COUNTERCLOCKWISE, 90);
                    }
                }
                break;
            }
            case FORWARD: {
                switch(target) {
                    case RIGHT: {
                        rotate(Rotation.CLOCKWISE, 90);
                        break;
                    }
                    case BACKWARD: {
                        rotate(Rotation.CLOCKWISE, 180);
                        break;
                    }
                    case LEFT: {
                        rotate(Rotation.COUNTERCLOCKWISE, 90);
                    }
                }
                break;
            }
            case RIGHT: {
                switch(target) {
                    case BACKWARD: {
                        rotate(Rotation.CLOCKWISE, 90);
                        break;
                    }
                    case LEFT: {
                        rotate(Rotation.CLOCKWISE, 180);
                        break;
                    }
                    case FORWARD: {
                        rotate(Rotation.COUNTERCLOCKWISE, 90);
                    }
                }
                break;
            }
            case BACKWARD: {
                switch(target) {
                    case LEFT: {
                        rotate(Rotation.CLOCKWISE, 90);
                        break;
                    }
                    case FORWARD: {
                        rotate(Rotation.CLOCKWISE, 180);
                        break;
                    }
                    case RIGHT: {
                        rotate(Rotation.COUNTERCLOCKWISE, 90);
                    }
                }
                break;
            }
        }
    }

    public void move(Direction direction, double millimeters) {
        switch (direction) {
            case FORWARD: {
                telemetry.addData("Moving", "Forward");
                telemetry.update();
                motorLeftBack.setPower(1);
                motorLeftFront.setPower(1);
                motorRightFront.setPower(1);
                motorRightBack.setPower(1);
                break;
            }
            case BACKWARD: {
                telemetry.addData("Moving", "Backward");
                telemetry.update();
                motorLeftBack.setPower(-1);
                motorLeftFront.setPower(-1);
                motorRightFront.setPower(-1);
                motorRightBack.setPower(-1);
                break;
            }
            case LEFT: {
                telemetry.addData("Moving", "Left");
                telemetry.update();
                motorLeftBack.setPower(1);
                motorLeftFront.setPower(-1);
                motorRightFront.setPower(1);
                motorRightBack.setPower(-1);
                break;
            }
            case RIGHT: {
                telemetry.addData("Moving", "Right");
                telemetry.update();
                motorLeftBack.setPower(-1);
                motorLeftFront.setPower(1);
                motorRightFront.setPower(-1);
                motorRightBack.setPower(1);
                break;
            }
        }
        if(millimeters >= 0) {
            sleep((long) (millimeters * MS_PER_MM));
        }
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }

    public void rotate(Rotation rotation, double degrees) {
        switch (rotation) {
            case CLOCKWISE: {
                telemetry.addData("Rotating", "Clockwise");
                telemetry.update();
                motorLeftBack.setPower(1);
                motorLeftFront.setPower(1);
                motorRightFront.setPower(-1);
                motorRightBack.setPower(-1);
                break;
            }
            case COUNTERCLOCKWISE: {
                telemetry.addData("Rotating", "Counterclockwise");
                telemetry.update();
                motorLeftBack.setPower(-1);
                motorLeftFront.setPower(-1);
                motorRightFront.setPower(1);
                motorRightBack.setPower(1);
                break;
            }
        }
        if(degrees >= 0) {
            sleep((long) (degrees * MS_PER_DEGREE));
        }
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }
}