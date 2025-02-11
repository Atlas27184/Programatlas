package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Programação braba 2")
public class Mecanum_TeleOp_Melhorado extends LinearOpMode {

    DcMotor superior_esquerdo;
    DcMotor superior_direito;
    DcMotor inferior_esquerdo;
    DcMotor inferior_direito;
    DcMotor gira_slide;
    IMU imu;

    private double targetAngle;

    @Override
    public void runOpMode() {

        double sy, position;

        superior_esquerdo = hardwareMap.get(DcMotor.class, "superior esquerdo");
        superior_direito = hardwareMap.get(DcMotor.class, "superior direito");
        inferior_esquerdo = hardwareMap.get(DcMotor.class, "inferior esquerdo");
        inferior_direito = hardwareMap.get(DcMotor.class, "inferior direito");
        gira_slide = hardwareMap.get(DcMotor.class, "gira_slide");
        imu = hardwareMap.get(IMU.class, "imu");



        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        superior_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        inferior_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        superior_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        superior_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inferior_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inferior_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gira_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();

        if (opModeIsActive()) {


            while (opModeIsActive()) {
                drive();
                telemetry.update();
                gira_slide.setPower(gamepad2.left_stick_y);
            }
        }
    }
    public void drive() {
        double y, x, rx;
        double kP = 0.03;
        y = gamepad1.left_stick_y * 0.8;
        x = gamepad1.left_stick_x * 0.8;
        rx = gamepad1.right_stick_x * 0.8;

        double currentAngle = getHeading();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        if (rx != 0) {
            targetAngle = currentAngle;
        }
        if (gamepad1.y) {
            targetAngle = 0;
        }
        if (gamepad1.x) {
            targetAngle = 90;
        }
        if (gamepad1.b) {
            targetAngle = -90;
        }
        if (gamepad1.a) {
            targetAngle = 180;
        }
        if (gamepad1.left_bumper) {
            targetAngle = -45;
        }

        if (gamepad1.right_bumper) {
            targetAngle = 45;
        }

        double error = normalizeAngle(targetAngle - currentAngle);
        double correction = (rx == 0) ? error * kP : 0;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx) + Math.abs(correction), 1);

        superior_esquerdo.setPower((rotY - rotX + correction - rx) / denominator);
        inferior_esquerdo.setPower((rotY + rotX + correction - rx) / denominator);
        superior_direito.setPower((rotY + rotX - correction + rx) / denominator);
        inferior_direito.setPower((rotY - rotX - correction + rx) / denominator);

        telemetry.addData("CurrentAngle:", currentAngle);
        telemetry.addData("TargetAngle:", targetAngle);
    }

    public double getHeading() {
        // Aqui, recuperamos o ângulo de yaw diretamente
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}