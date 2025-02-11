package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Programação braba")
public class Mecanum_TeleOp extends LinearOpMode {

    DcMotor superior_esquerdo;
    DcMotor superior_direito;
    DcMotor inferior_esquerdo;
    DcMotor inferior_direito;
    IMU imu;

    private double targetAngle;

    @Override
    public void runOpMode() {

        double sy, position;

        superior_esquerdo = hardwareMap.get(DcMotor.class, "superior esquerdo");
        superior_direito = hardwareMap.get(DcMotor.class, "superior direito");
        inferior_esquerdo = hardwareMap.get(DcMotor.class, "inferior esquerdo");
        inferior_direito = hardwareMap.get(DcMotor.class, "inferior direito");
        imu = hardwareMap.get(IMU.class, "imu");



        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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



        waitForStart();

        if (opModeIsActive()) {


            while (opModeIsActive()) {
                drive();
                telemetry.update();
            }
        }
    }
    public void drive() {
        double y, x, rx;
        double kP = 0.03;
        y = gamepad1.left_stick_y * 0.8;
        x = gamepad1.right_stick_x * 0.8;
        rx = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.8;

        double currentAngle = getHeading();

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

        superior_esquerdo.setPower(y - x + correction - rx);
        inferior_esquerdo.setPower(y + x + correction - rx);
        superior_direito.setPower(y + x - correction + rx);
        inferior_direito.setPower(y - x - correction + rx);

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