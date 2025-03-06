package org.firstinspires.ftc.teamcode.auto;

// Importações
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "Autonomo com Mecanum")
public class AutoMecanum extends LinearOpMode {

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

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_CIRCUMFERENCE_MM = 101.6 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    private static final double Kp = 0.05;
    private static final double Ki = 0.0001;
    private static final double Kd = 0.002;
    private static final double KpTurn = 0.059;
    private int targetPosition = 0;
    private int garra_targetPosition = -105;
    double previous_error = 0; // Guarda o erro do loop anterior
    long last_time = System.nanoTime(); // Guarda o tempo da última atualização
    int stage = 0;

    @Override
    public void runOpMode() {
        // Inicializando os motores e sensores
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
        sleep(1000);
        imu.resetYaw();

        slide_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gira_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        t_hex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gira_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        t_hex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        superior_direito.setDirection(DcMotorSimple.Direction.REVERSE);
        inferior_direito.setDirection(DcMotorSimple.Direction.REVERSE);
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

            garra("fechada", "cima");
            new Thread(() -> girarSlide("basket", -620)).start();
            drive(400, -400, 1.0, 0);
            rotateToAngle(-42);
            sleep(500);
            new Thread(() -> slide(2420)).start();
            drive(-160, 0, 1.0, 0);
            rotateToAngle(-45);
            garra("aberta", "entregar");
            drive(0, 20, 1.0, 0);
            new Thread(() -> slide(2420)).start();
            garra("aberta", "voltar");
            slide(550);
            rotateToAngle(0);
            drive(160, 160, 1.0, 0);
            rotateToAngle(0);
            drive(130, 0, 0.1, 0);
            girarSlide("specimen", -190);
            garra("aberta", "-140");
            docinho.setPosition(0.15);
            sleep(400);
            drive(-300, 0, 1.0, 0);
            new Thread(() -> girarSlide("basket", -620)).start();
            rotateToAngle(-45);
            drive(-50, 0, 1.0, 0);
            sleep(500);
            slide(2420);
            rotateToAngle(-45);
            garra("aberta", "entregar");
            drive(0, 20, 1.0, 0);
            new Thread(() -> slide(2420)).start();
            garra("aberta", "voltar");
            slide(550);
        }

    }

    public void drive(double forwardMM, double strafeMM, double power, double targetAngle) {
        int forwardTicks = (int) (forwardMM * DRIVE_COUNTS_PER_MM);
        int strafeTicks = (int) (strafeMM * DRIVE_COUNTS_PER_MM);
        double previousError = 0;
        double integralSum = 0;
        long lastTime = System.currentTimeMillis();

        superior_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        superior_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inferior_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inferior_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        superior_esquerdo.setTargetPosition(forwardTicks + strafeTicks);
        superior_direito.setTargetPosition(forwardTicks - strafeTicks);
        inferior_esquerdo.setTargetPosition(forwardTicks - strafeTicks);
        inferior_direito.setTargetPosition(forwardTicks + strafeTicks);

        superior_esquerdo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        superior_direito.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inferior_esquerdo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inferior_direito.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                (superior_esquerdo.isBusy() || superior_direito.isBusy() || inferior_esquerdo.isBusy() || inferior_direito.isBusy())) {

            double currentAngle = getHeading();
            double error = normalizeAngle(targetAngle - currentAngle);
            integralSum += error;
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;
            double derivative = (error - previousError) / deltaTime;

            double correction = (error * Kp) + (integralSum * Ki) + (derivative * Kd);
            previousError = error;
            lastTime = currentTime;

            double powerSE = power - correction; // Superior Esquerdo
            double powerSD = power + correction; // Superior Direito
            double powerIE = power + correction; // Inferior Esquerdo
            double powerID = power - correction; // Inferior Direito

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

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Correction", correction);
            telemetry.addData("Power SE", powerSE);
            telemetry.addData("Power SD", powerSD);
            telemetry.addData("Power IE", powerIE);
            telemetry.addData("Power ID", powerID);
            telemetry.update();
        }

        superior_esquerdo.setPower(0);
        superior_direito.setPower(0);
        inferior_esquerdo.setPower(0);
        inferior_direito.setPower(0);

        sleep(50);
    }





    public void rotateToAngle(double targetAngle) {

        while (opModeIsActive()) {

            superior_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            superior_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            inferior_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            inferior_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double currentAngle = getHeading();
            double error = (targetAngle - currentAngle);

            if (Math.abs(error) < 5.0) {
                break;
            }

            double correction = error * KpTurn;
            correction = Math.max(-0.7, Math.min(correction, 0.7));

            superior_esquerdo.setPower(-correction);
            inferior_esquerdo.setPower(-correction);
            superior_direito.setPower(correction);
            inferior_direito.setPower(correction);

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Correction", correction);
            telemetry.addData("Slide Position:", slide_esquerdo.getCurrentPosition());
            telemetry.update();
        }

        superior_esquerdo.setPower(0);
        superior_direito.setPower(0);
        inferior_esquerdo.setPower(0);
        inferior_direito.setPower(0);
        sleep(20);
    }

    public void slide(int targetPosition) {
        double slide_power;
        double kP_slide = 0.0019;

        while(opModeIsActive()) {
            int slide_currentPosition = slide_esquerdo.getCurrentPosition();
            int error = targetPosition - slide_currentPosition;
            double correction = error * kP_slide;

            if (Math.abs(error) < 20.0) {
                break;
            } else {

                if ((slide_esquerdo.getCurrentPosition() >= 2500) && (targetPosition > slide_currentPosition)) {
                    slide_power = 0;
                } else if ((slide_esquerdo.getCurrentPosition() <= 600) && (targetPosition < slide_currentPosition)) {
                    slide_power = 0;
                } else {
                    slide_power = correction + Math.signum(correction) * 0.3;

                    ;
                }
            }

                slide_esquerdo.setPower(slide_power);
                slide_direito.setPower(slide_power);
                telemetry.addData("Slide:", slide_esquerdo.getCurrentPosition());
                telemetry.addData("Gira Slide:", gira_slide.getCurrentPosition());
                telemetry.update();
            }
        slide_esquerdo.setPower(0);
        slide_direito.setPower(0);
    }

    public void girarSlide(String Posicao, int position) {

        final double KpGiraSlide = 0.00355;
        final double KiGiraSlide = 0; // Adicionando constante integral
        double previousError = 0;
        double integralSum = 0; // Inicializando o somatório integral
        long lastTime = System.currentTimeMillis();
        final double KdGiraSlide = 0.000047; // Ajuste conforme necessário
        double maxIntegral = 1000; // Limite para evitar acumulação excessiva
            if (Posicao != "Null") {

            if (Posicao == "specimen") {
                targetPosition = -190;
                florzinha.setPosition(0.98);
                garra("aberta", "frente");
            }
            if(Posicao == "sample") {
                targetPosition = -190;
                florzinha.setPosition(0.98);
                garra("aberta", "baixo");
            }
            if(Posicao == "basket") {
                targetPosition = -620;
                florzinha.setPosition(0.98);
                garra("fechada", "frente");
            }
            if(Posicao == "chamber") {
                targetPosition = -620;
                florzinha.setPosition(0.28);
                garra("fechada", "cima");
            } } else {
                targetPosition = position;
            }

            gira_slide.setTargetPosition(targetPosition);
            gira_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive()) {

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

                if (Math.abs(error) < 1.0) {
                    break;
                }

                telemetry.addData("Alvo do Gira Slide:", targetPosition);
                telemetry.addData("Erro:", error);
                telemetry.addData("Integral:", integralSum);
                telemetry.addData("Derivativo:", derivative);
                telemetry.addData("Correção:", correction);
            }
    }

    public void garra(String aberta_fechada, String giro_garra) {
        double kP_garra = 0.07;
        double kD_garra = 0.0003;// Ajuste fino para o freio

        if (giro_garra == "-140") {
            garra_targetPosition = -130;
        }
        if (giro_garra == "cima") {
            garra_targetPosition = -30;
        }
        if (giro_garra == "entregar") {
            garra_targetPosition = -65;
        }
        if(giro_garra == "frente") {
            garra_targetPosition = -105;
        }
        if(giro_garra == "baixo") {
            garra_targetPosition = -150;
        }
        if(giro_garra == "voltar") {
            garra_targetPosition = -120;
        }

        while (opModeIsActive()) {
            int garra_position = t_hex.getCurrentPosition();

            double error = garra_targetPosition - garra_position;

            if (Math.abs(error) < 1.0) {
                break;
            }

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

            t_hex.setTargetPosition(garra_targetPosition);
            t_hex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            t_hex.setPower(correction);
        }
        t_hex.setPower(0);

        if(aberta_fechada == "aberta") {
            docinho.setPosition(0.5);
        }
        if(aberta_fechada == "fechada") {
            docinho.setPosition(0.15);
        }
    }



    public double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
