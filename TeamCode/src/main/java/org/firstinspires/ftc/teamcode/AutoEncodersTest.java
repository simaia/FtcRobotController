package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AutoEncodersTest", group = "DEV")
@Disabled
public class AutoEncodersTest extends LinearOpMode {

    // CONSTANTS
    final double PI = 3.14159265359;
    final double WHEEL_DIAMETER_IN_MM = 100;
    final double TICKS_PER_SEC = 1080; // Number of wheel rotations per second * 360
    final double INCH_TO_MM = 25.4;
    final double DEGREES_PER_MS = 0.108;
    final double ACCELERATION_TIME_MS = 0;
    final double DISTANCE_DURING_ACCELERATION_MM = 50;
    final double DEGREES_DURING_ACCELERATION = 0;

    // DERIVED CONSTANTS
    final double MM_PER_MS = TICKS_PER_SEC / 360_000.0 * WHEEL_DIAMETER_IN_MM * PI;
    final double MS_PER_MM = 1.0 / MM_PER_MS;
    final double MS_PER_DEGREE = 1.0 / DEGREES_PER_MS;

    // IMPORTANT POINTS
    final Position backstageBlue = new Position(22.75 * 0.5 * INCH_TO_MM, 24 * 0.5 * INCH_TO_MM);
    final Position backstageRed = new Position(22.75 * 5.5 * INCH_TO_MM, 24 * 0.5 * INCH_TO_MM);
    final Position startBlueBackstage = new Position(9 * INCH_TO_MM, 24 * 2.5 * INCH_TO_MM);
    final Position startBluePublic = new Position(9 * INCH_TO_MM, 24 * 4.5 * INCH_TO_MM);
    final Position startRedBackstage = new Position((22.75 * 6 - 9) * INCH_TO_MM, 24 * 2.5 * INCH_TO_MM);
    final Position startRedPublic = new Position((22.75 * 6 - 9) * INCH_TO_MM, 24 * 4.5 * INCH_TO_MM);
    DcMotorEx motorRightFront = null;
    DcMotorEx motorRightBack = null;
    DcMotorEx motorLeftFront = null;
    DcMotorEx motorLeftBack = null;
    Rev2mDistanceSensor sensorLeft = null;
    Rev2mDistanceSensor sensorRight = null;
    RevColorSensorV3 sensorColour = null;
    Position currentPosition = null;
    Direction currentDirection = null;
    Team currentTeam = null;
    StartingLocation currentStartingLocation = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.get(DcMotorEx.class, "Right Front Motor");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "Right Back Motor");
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "Left Front Motor");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "Left Back Motor");
        // servoArmBase = hardwareMap.get(CRServo.class, "Arm Base Servo");
        // servoArmTop = hardwareMap.get(Servo.class, "Arm Top Servo");
        // sensorLeft = hardwareMap.get(Rev2mDistanceSensor.class, "Left Sensor");
        // sensorRight = hardwareMap.get(Rev2mDistanceSensor.class, "Right Sensor");
        // sensorColour = hardwareMap.get(RevColorSensorV3.class, "Colour Sensor");

        motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLeftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

            //move(Direction.FORWARD, 1000);
            rotate(Rotation.CLOCKWISE, 360);
            rotate(Rotation.COUNTERCLOCKWISE, 720);
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
                motorLeftBack.setVelocity(TICKS_PER_SEC);
                motorLeftFront.setVelocity(TICKS_PER_SEC);
                motorRightFront.setVelocity(TICKS_PER_SEC);
                motorRightBack.setVelocity(TICKS_PER_SEC);
                break;
            }
            case BACKWARD: {
                telemetry.addData("Moving", "Backward");
                telemetry.update();
                motorLeftBack.setVelocity(-TICKS_PER_SEC);
                motorLeftFront.setVelocity(-TICKS_PER_SEC);
                motorRightFront.setVelocity(-TICKS_PER_SEC);
                motorRightBack.setVelocity(-TICKS_PER_SEC);
                break;
            }
            case LEFT: {
                telemetry.addData("Moving", "Left");
                telemetry.update();
                motorLeftBack.setVelocity(TICKS_PER_SEC);
                motorLeftFront.setVelocity(-TICKS_PER_SEC);
                motorRightFront.setVelocity(TICKS_PER_SEC);
                motorRightBack.setVelocity(-TICKS_PER_SEC);
                break;
            }
            case RIGHT: {
                telemetry.addData("Moving", "Right");
                telemetry.update();
                motorLeftBack.setVelocity(-TICKS_PER_SEC);
                motorLeftFront.setVelocity(TICKS_PER_SEC);
                motorRightFront.setVelocity(-TICKS_PER_SEC);
                motorRightBack.setVelocity(TICKS_PER_SEC);
                break;
            }
        }
        if(millimeters >= DISTANCE_DURING_ACCELERATION_MM) {
            sleep((long) ((millimeters - DISTANCE_DURING_ACCELERATION_MM) * MS_PER_MM + ACCELERATION_TIME_MS));
        }
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(200);
    }

    public void rotate(Rotation rotation, double degrees) {
        switch (rotation) {
            case CLOCKWISE: {
                telemetry.addData("Rotating", "Clockwise");
                telemetry.update();
                motorLeftBack.setVelocity(TICKS_PER_SEC);
                motorLeftFront.setVelocity(TICKS_PER_SEC);
                motorRightFront.setVelocity(-TICKS_PER_SEC);
                motorRightBack.setVelocity(-TICKS_PER_SEC);
                break;
            }
            case COUNTERCLOCKWISE: {
                telemetry.addData("Rotating", "Counterclockwise");
                telemetry.update();
                motorLeftBack.setVelocity(-TICKS_PER_SEC);
                motorLeftFront.setVelocity(-TICKS_PER_SEC);
                motorRightFront.setVelocity(TICKS_PER_SEC);
                motorRightBack.setVelocity(TICKS_PER_SEC);
                break;
            }
        }
        if(degrees >= DISTANCE_DURING_ACCELERATION_MM) {
            sleep((long) ((degrees - DEGREES_DURING_ACCELERATION) * MS_PER_DEGREE - ACCELERATION_TIME_MS));
        }
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(200);
    }
}