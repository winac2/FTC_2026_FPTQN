package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumDrivetrain")

public class MecanumDrivetrain extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    ColorSensor CS = new ColorSensor();

    GameFunctions GF = new GameFunctions();

    PIDController pid = new PIDController(0.01, 0.0, 0.001);

    public IMU imu;
    double forward, strafe, rotate;

    boolean pressPadUp, statePadUp;


    @Override
    public void init(){

        drive.init(hardwareMap);
        CS.init(hardwareMap);
        GF.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        pressPadUp = false;
        statePadUp = false;

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();

    }

    //-------------------Hàm điều khiển-------------
    public void drive(){
        forward = gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;

        drive.driveFieldRelative(forward,strafe,rotate);
    }

    //====================Hàm bắn================
    public void fireArtifacts(){
        if (gamepad2.a) {
            GF.loadMotif("GREEN", "PURPLE", "PURPLE");
            GF.fire_on();
            while (!GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            if (GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            GF.fire_off();
        }
        else if (gamepad2.b) {
            GF.loadMotif("PURPLE", "GREEN", "PURPLE");
            GF.fire_on();
            while (!GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            if (GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            GF.fire_off();
        }
        else if (gamepad2.x) {
            GF.loadMotif("PURPLE", "PURPLE", "GREEN");
            GF.fire_on();
            while (!GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            if (GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            GF.fire_off();
        }
        else if (gamepad2.y) {
            GF.loadMotif("UNKNOWN", "UNKNOWN", "UNKNOWN");
            GF.fire_on();
            while (!GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            if (GameFunctions.state.equals(GameFunctions.ClassifyState.OFF)) {
                GF.update();
            }
            GF.fire_off();
        }
    }

    //=====================Hàm INTAKE==================
    public void intake(){
        if (gamepad2.dpad_up && !pressPadUp) {
            statePadUp = !statePadUp;
            if (statePadUp) {
                GF.intake(0.8);
                GF.fill();
            }
            else {
                GF.intake(0);
            }
        }
        pressPadUp = gamepad2.dpad_up;
    }

    public void testFire(){
        if (gamepad1.dpad_up) {
            GF.fire_on();
        }
        else {
            GF.fire_off();
        }
    }

    public void loop(){

        //----------------------Thông số IMU------------------------------
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("red = ", CS.normRed);
        telemetry.addData("green = ", CS.normGreen);
        telemetry.addData("blue = ", CS.normBlue);
        telemetry.addData("Color Detected: ", CS.getDetectedColor());
        telemetry.update();

        //---------------------Các tác vụ----------------------------
        drive(); // Lái
        
        intake(); //intake
        
        fireArtifacts(); // bắn bóng

        testFire();
    }

}
