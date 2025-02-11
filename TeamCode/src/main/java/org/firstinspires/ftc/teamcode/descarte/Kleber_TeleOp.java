package org.firstinspires.ftc.teamcode.descarte;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Kleber_TeleOp")
public class Kleber_TeleOp extends LinearOpMode {

    DcMotor motor_esquerdo;
    DcMotor motor_direito;
    DcMotor anexo_1;
    DcMotor anexo_2;
    CRServo vassourinha;
    Servo garrinha_1;
    Servo garrinha_2;
    IMU imu;


    @Override
    public void runOpMode() {

        double sy, sry, position;

        motor_esquerdo = hardwareMap.get(DcMotor.class, "motor esquerdo");
        motor_direito = hardwareMap.get(DcMotor.class, "motor direito");
        anexo_1 = hardwareMap.get(DcMotor.class, "anexo 1");
        anexo_2 = hardwareMap.get(DcMotor.class, "anexo 2");
        vassourinha = hardwareMap.get(CRServo.class, "vassourinha");
        garrinha_1 = hardwareMap.get(Servo.class, "gar");
        garrinha_2 = hardwareMap.get(Servo.class, "rinha");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        anexo_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        anexo_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        anexo_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        anexo_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        anexo_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                position = garrinha_1.getPosition();
                telemetry.addData("Posição:", position);
                telemetry.update();

                sy = gamepad2.left_stick_y;
                sry = gamepad2.right_stick_y * 0.5;

                anexo_1.setDirection(DcMotorSimple.Direction.REVERSE);
                anexo_1.setPower(sy);


                anexo_2.setPower(sry);

                drive();

                if (gamepad2.a) {
                    garrinha_1.setPosition(1);
                    garrinha_2.setPosition(0.8);
                }
                if (gamepad2.b) {
                    garrinha_1.setPosition(0.8);
                    garrinha_2.setPosition(1);
                }

                if (gamepad2.right_bumper) {
                    vassourinha.setPower(0.7);
                } else if (gamepad2.left_bumper) {
                    vassourinha.setPower(-0.7);
                } else {
                    vassourinha.setPower(0);
                }
                telemetry.update();
            }
        }
    }
    public void drive() {
        double y, rx;
        double kP = 0.2;
        y = gamepad1.left_stick_y * 0.8;
        rx = gamepad1.right_stick_x * 4;

        double currentAngle = getHeading();
        double targetAngle = currentAngle + rx;
        double error = normalizeAngle(targetAngle - currentAngle);
        double correction = error * kP;

        motor_esquerdo.setPower(y - correction);
        motor_direito.setPower(y + correction);
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