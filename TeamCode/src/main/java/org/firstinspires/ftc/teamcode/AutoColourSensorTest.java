package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoColourSensorTest", group = "TEST")
@Disabled
public class AutoColourSensorTest extends LinearOpMode {
    RevColorSensorV3 sensorColour = null;
    @Override
    public void runOpMode() throws InterruptedException {
        sensorColour = hardwareMap.get(RevColorSensorV3.class, "Colour Sensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Green", sensorColour.green());
            telemetry.addData("Red", sensorColour.red());
            telemetry.addData("Blue", sensorColour.blue());
            telemetry.update();
        }
    }
}
