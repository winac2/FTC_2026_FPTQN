package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name ="helloworld")

public class helloworld extends OpMode
{

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Message", "Hello World!");
        telemetry.update();
    }
    @Override
    public void loop() {
        // You can also update the message in the loop if needed
        // telemetry.addData("Status", "Running");
        // telemetry.update();
    }
}
