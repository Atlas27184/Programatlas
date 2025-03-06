package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Programação braba 2")
public class Mecanum_TeleOp_Melhorado extends LinearOpMode {

    DcMotor superior_esquerdo;
    DcMotor superior_direito;
    DcMotor inferior_esquerdo;
    DcMotor inferior_direito;
    DcMotor gira_slide;
    DcMotor slide_esquerdo;
    DcMotor slide_direito;
    DcMotor t_hex;
    Servo docinho;
    Servo florzinha;
    IMU imu;

    private double targetAngle;
    private int targetPosition = 0;
    private int slide_targetPosition = 0;
    private int garra_targetPosition = 0;
    double previous_error = 0; // Guarda o erro do loop anterior
    long last_time = System.nanoTime(); // Guarda o tempo da última atualização

    @Override
    public void runOpMode() {

        double sy, position;

        superior_esquerdo = hardwareMap.get(DcMotor.class, "superior esquerdo");
        superior_direito = hardwareMap.get(DcMotor.class, "superior direito");
        inferior_esquerdo = hardwareMap.get(DcMotor.class, "inferior esquerdo");
        inferior_direito = hardwareMap.get(DcMotor.class, "inferior direito");
        gira_slide = hardwareMap.get(DcMotor.class, "gira_slide");
        slide_esquerdo = hardwareMap.get(DcMotor.class, "slide_esquerdo");
        slide_direito = hardwareMap.get(DcMotor.class, "slide_direito");
        t_hex = hardwareMap.get(DcMotor.class, "T Hex");
        docinho = hardwareMap.get(Servo.class, "Docinho");
        florzinha = hardwareMap.get(Servo.class, "Florzinha");
        imu = hardwareMap.get(IMU.class, "imu");



        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        slide_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gira_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        t_hex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gira_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        t_hex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        superior_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        inferior_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        gira_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        superior_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        superior_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inferior_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inferior_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gira_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        t_hex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart();

        if (opModeIsActive()) {
            gira_slide.setTargetPosition(10);
            t_hex.setTargetPosition(0);
            while (opModeIsActive()) {
                drive();
                garra();
                slide();
                girarSlide();
                telemetry.update();
            }
        }
    }

    public void drive() {
        double y, x, rx;
        double kP = 0.0075;
        double kD = 0.0004;
        double kI = 0.0001; // Comece com um valor pequeno
        double integralSum = 0; // Acumulador
        double maxIntegral = 100; // Limite para evitar excesso
        double previousError = 0;
        long lastTime = System.currentTimeMillis();
        double deadZone = 1.5; // Zona Morta de 1.5 graus
        double minPower = 0.1; // Potência mínima para correção

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

        if (gamepad1.y) targetAngle = 0;
        if (gamepad1.x) targetAngle = 90;
        if (gamepad1.b) targetAngle = -90;
        if (gamepad1.a) targetAngle = 180;
        if (gamepad1.left_bumper) targetAngle = -45;
        if (gamepad1.right_bumper) targetAngle = 45;

        double error = normalizeAngle(targetAngle - currentAngle);

        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;
        double derivative = (error - previousError) / deltaTime;

        if (Math.abs(error) > deadZone) {
            integralSum += error * deltaTime; // Soma os erros ao longo do tempo
            integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum)); // Limita o integral
        } else {
            integralSum = 0; // Zera quando o robô chega perto do alvo
        }

        double correction = (error * kP) + (derivative * kD) + (integralSum * kI);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx) + Math.abs(correction), 1);

        double powerSE = (rotY - rotX + correction - rx) / denominator; // Superior Esquerdo
        double powerSD = (rotY + rotX + correction - rx) / denominator; // Superior Direito
        double powerIE = (rotY + rotX - correction + rx) / denominator; // Inferior Esquerdo
        double powerID = (rotY - rotX - correction + rx) / denominator; // Inferior Direito

        // Normalização
        double maxPower = Math.max(Math.max(Math.abs(powerSE), Math.abs(powerSD)),
                Math.max(Math.abs(powerIE), Math.abs(powerID)));

        if (maxPower > 1.0) {
            powerSE /= maxPower;
            powerSD /= maxPower;
            powerIE /= maxPower;
            powerID /= maxPower;
        }

        superior_esquerdo.setPower(powerSE);
        superior_direito.setPower(powerSD);
        inferior_esquerdo.setPower(powerIE);
        inferior_direito.setPower(powerID);

        previousError = error;
        lastTime = currentTime;

        telemetry.addData("CurrentAngle:", currentAngle);
        telemetry.addData("TargetAngle:", targetAngle);
        telemetry.addData("Error:", error);
        telemetry.addData("Correction:", correction);
    }

    public void garra() {
        double docinho_position = docinho.getPosition();
        double florzinha_position = florzinha.getPosition();
        double florzinha_target = gamepad2.right_stick_x / 50;

        double kP_garra = 0.07;
        double kD_garra = 0.0003; // Ajuste fino para o freio
        int garra_position = t_hex.getCurrentPosition();

        double error = garra_targetPosition - garra_position;

        // ΔTempo
        long current_time = System.nanoTime();
        double delta_time = (current_time - last_time) / 1e9; // Converte para segundos
        last_time = current_time;

        // ΔErro
        double derivative = (error - previous_error) / delta_time;
        previous_error = error;

        double correction = (error * kP_garra) + (derivative * kD_garra);

        // Limitando a potência mínima
        correction = Math.max(Math.abs(correction), 0.15);

        if (gamepad2.right_stick_y != 0) {
            garra_targetPosition = garra_position; // Travar posição atual
            t_hex.setPower(gamepad2.right_stick_y * 0.5);
            t_hex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            t_hex.setTargetPosition(garra_targetPosition);
            t_hex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            t_hex.setPower(correction);
        }

        if (gamepad2.b) {
            docinho.setPosition(0.5);
        }
        if (gamepad2.a) {
            docinho.setPosition(0.15);
        }

        if (gamepad2.left_bumper) {
            florzinha.setPosition(0.27);
        }
        if (gamepad2.right_bumper) {
            florzinha.setPosition(0.98);
        }
        if (gamepad2.x) {
            florzinha.setPosition(0.65);
        }
        if (gamepad2.right_stick_x != 0) {
            florzinha.setPosition(florzinha_position + florzinha_target);
        }

        telemetry.addData("Docinho:", docinho_position);
        telemetry.addData("Florzinha:", florzinha_position);
        telemetry.addData("T Hex:", garra_position);
        telemetry.addData("Target:", garra_targetPosition);
        telemetry.addData("Correction:", correction);
        telemetry.addData("Error:", error);
        telemetry.addData("Derivative:", derivative);
    }


    public void slide() {
        double slide_power;
        double kP_slide = 0.0003;
        int slide_currentPosition = slide_esquerdo.getCurrentPosition();
        if((gamepad2.right_trigger - gamepad2.left_trigger) != 0) {
            slide_targetPosition = slide_currentPosition;
        }

        double error = slide_targetPosition - slide_currentPosition;
        double correction = error * kP_slide;

        if((slide_esquerdo.getCurrentPosition() >= 2400) && (gamepad2.right_trigger != 0)) {
            slide_power = 0;
        } else if((slide_esquerdo.getCurrentPosition() <= 200) && (gamepad2.left_trigger != 0))  {
            slide_power = 0;
        } else {
            slide_power = gamepad2.right_trigger - gamepad2.left_trigger + correction;
        }
        slide_esquerdo.setPower(slide_power);
        slide_direito.setPower(slide_power);
        telemetry.addData("Slide:", slide_esquerdo.getCurrentPosition());
        telemetry.addData("Gira Slide:", gira_slide.getCurrentPosition());
    }

    public void girarSlide() {

        final double KpGiraSlide = 0.00355;
        final double KiGiraSlide = 0; // Adicionando constante integral
        double previousError = 0;
        double integralSum = 0; // Inicializando o somatório integral
        long lastTime = System.currentTimeMillis();
        final double KdGiraSlide = 0.000035; // Ajuste conforme necessário
        double maxIntegral = 1000; // Limite para evitar acumulação excessiva

        if(gamepad2.left_stick_y != 0) {
            gira_slide.setPower(gamepad2.left_stick_y * 0.6);
            targetPosition = gira_slide.getCurrentPosition();
            integralSum = 0; // Zera o somatório quando houver movimento manual
        }
        else {
            if (gamepad2.dpad_right) {
                targetPosition = -190;
                florzinha.setPosition(0.98);
                garra_targetPosition = -80;
                docinho.setPosition(0.15);
            }
            if(gamepad2.dpad_down) {
                targetPosition = -190;
                florzinha.setPosition(0.98);
                garra_targetPosition = -150;
                docinho.setPosition(0.05);
            }
            if(gamepad2.dpad_up) {
                targetPosition = -620;
                florzinha.setPosition(0.25);
                garra_targetPosition = -80;
            }
            if(gamepad2.dpad_left) {
                targetPosition = -620;
                florzinha.setPosition(0.98);
                garra_targetPosition = -15;
            }

            gira_slide.setTargetPosition(targetPosition);
            double currentPosition = gira_slide.getCurrentPosition();
            double error = targetPosition - currentPosition;
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;

            // Somatório Integral
            integralSum += error * deltaTime;
            integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum)); // Limita o somatório integral

            // Termo Derivativo
            double derivative = (error - previousError) / deltaTime;
            if (Math.abs(error) < 10) {
                derivative = 0;
            }
            // Aplicando PID
            double correction = (error * KpGiraSlide) + (integralSum * KiGiraSlide) + (derivative * KdGiraSlide);
            correction = Math.max(-0.3, Math.min(correction, 0.3));

            gira_slide.setPower(correction);

            // Atualizando os valores para o próximo ciclo
            previousError = error;
            lastTime = currentTime;

            telemetry.addData("Alvo do Gira Slide:", targetPosition);
            telemetry.addData("Erro:", error);
            telemetry.addData("Integral:", integralSum);
            telemetry.addData("Derivativo:", derivative);
            telemetry.addData("Correção:", correction);
        }
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