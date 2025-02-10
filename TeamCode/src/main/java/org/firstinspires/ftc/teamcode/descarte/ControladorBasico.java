package org.firstinspires.ftc.teamcode.descarte;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Controlador Básico")
public class ControladorBasico extends LinearOpMode {

    DcMotor motor_esquerdo;
    DcMotor motor_direito;
    DcMotor anexo_1;
    DcMotor anexo_2;
    CRServo vassourinha;
    Servo garrinha;


    @Override
    public void runOpMode() {

        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        double y, rx, sy, sry, position;

        motor_esquerdo = hardwareMap.get(DcMotor.class, "motor esquerdo");
        motor_direito = hardwareMap.get(DcMotor.class, "motor direito");
        anexo_1 = hardwareMap.get(DcMotor.class, "anexo 1");
        anexo_2 = hardwareMap.get(DcMotor.class, "anexo 2");
        vassourinha = hardwareMap.get(CRServo.class, "vassourinha");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        waitForStart();

        if (opModeIsActive()) {

            motor_esquerdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor_direito.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            anexo_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            anexo_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
            anexo_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            anexo_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {
                    position = garrinha.getPosition();
                    telemetry.addData("Posição:", position);
                    telemetry.update();

                    y = gamepad1.left_stick_y * 0.8;
                    rx = gamepad1.right_stick_x * 0.8;
                    sy = gamepad2.left_stick_y;
                    sry = gamepad2.right_stick_y * 0.5;

                    anexo_1.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor_esquerdo.setPower(y - rx);
                    motor_direito.setPower(y + rx);
                    anexo_1.setPower(sy);


                    anexo_2.setPower(sry);

                if (gamepad2.a) {
                    garrinha.setPosition(0.4);
                }
                if (gamepad2.b) {
                    garrinha.setPosition(0);
                }

                if (gamepad2.right_bumper) {
                    vassourinha.setPower(0.3);
                } else if (gamepad2.left_bumper) {
                    vassourinha.setPower(-0.3);
                } else {
                    vassourinha.setPower(0);
                }
                telemetry.update();
            }
        }
    }
}