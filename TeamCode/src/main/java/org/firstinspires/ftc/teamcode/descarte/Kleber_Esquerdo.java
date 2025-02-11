package org.firstinspires.ftc.teamcode.descarte;

//importações
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Autônomo Esquerdo") //Definição do nome que vai aparecer no Driver Hub
public class Kleber_Esquerdo extends LinearOpMode {

    DcMotor motor_esquerdo;
    DcMotor motor_direito;
    DcMotor anexo_1;
    DcMotor anexo_2;
    CRServo vassourinha;
    Servo garrinha;
    IMU imu;

    //Definição das constantes utilizadas na programação
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    private static final double Kp = 0.06;
    private static final double KpTurn = 0.0099;
    private static final double KpAnexo1 = 0.002;
    private static final double KpAnexo2 = 0.003;

    @Override
    public void runOpMode() {
        // Inicializando os motores e sensores
        motor_esquerdo = hardwareMap.get(DcMotor.class, "motor esquerdo");
        motor_direito = hardwareMap.get(DcMotor.class, "motor direito");
        anexo_1 = hardwareMap.get(DcMotor.class, "anexo 1");
        anexo_2 = hardwareMap.get(DcMotor.class, "anexo 2");
        vassourinha = hardwareMap.get(CRServo.class, "vassourinha");
        garrinha = hardwareMap.get(Servo.class, "garrinha");
        imu = hardwareMap.get(IMU.class, "imu");

        // Configuração de orientação e inicialização do IMU
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );


        imu.initialize(parameters); // Inicialização do IMU

        sleep(1000);

        // Configuração dos motores
        motor_direito.setDirection(DcMotorSimple.Direction.REVERSE);
        anexo_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        anexo_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        anexo_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        anexo_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        anexo_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        // o IMU reseta o seu ângulo, e depois mostra no driver, enquanto não iniciar.
        imu.resetYaw();

        telemetry.addData("IMU Yaw Inicial", getHeading());
        telemetry.update();

        // Espera pelo início da partida
        waitForStart();

        // Exemplos de chamadas para movimento com ângulo
        drive(0.7, 2200);
        drive(-0.7, 1700);
        rotateToAngle(180);
        moverAnexo1(1600);
        moverAnexo2(240);
        moverAnexo2(11);
        vassoura(3000, true);
        vassoura(3000, false);
        moverAnexo1(10);


        while (opModeIsActive()) {
            telemetry.addData("Posição do anexo 1:", anexo_1.getCurrentPosition());
            telemetry.addData("Posição do anexo 2:", anexo_2.getCurrentPosition());
            telemetry.update();
        }
    }

    public void drive(double power, double distanceMM) {
        // Calcula a posição alvo em ticks
        int targetPosition = (int) (distanceMM * DRIVE_COUNTS_PER_MM);
        double targetAngle = getHeading(); // Define o ângulo alvo como o ângulo atual, mantendo o robô sempre reto.

        motor_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Laço de movimento com correção de direção
        while (opModeIsActive() && (Math.abs(motor_esquerdo.getCurrentPosition()) < targetPosition || Math.abs(motor_direito.getCurrentPosition()) < targetPosition)) {
            double currentAngle = getHeading();
            double error = normalizeAngle(targetAngle - currentAngle);

            // Aplica a correção proporcional
            double correction = error * Kp;

            //definição da força dos motores, com base no valor dado anteriormente, e a correção
            motor_esquerdo.setPower(power - correction);
            motor_direito.setPower(power + correction);

            // Exibição dos dados para depuração
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position Left", motor_esquerdo.getCurrentPosition());
            telemetry.addData("Current Position Right", motor_direito.getCurrentPosition());
            telemetry.update();
        }

        // Parar os motores após atingir o alvo
        motor_esquerdo.setPower(0);
        motor_direito.setPower(0);
        sleep(200);
    }

    public void rotateToAngle(double targetAngle) {
        double FinalAngle = targetAngle + getHeading();
        while (opModeIsActive()) {
            double currentAngle = getHeading();
            double error = normalizeAngle(FinalAngle - currentAngle);

            // Se o erro for pequeno o suficiente, a rotação está próxima do ângulo desejado
            if (Math.abs(error) < 1.0) {
                break;
            }

            // Aplica uma correção proporcional ao erro para rotacionar, e define um limite a ela
            double correction = error * KpTurn;
            correction = Math.max(-0.7, Math.min(correction, 0.7));

            motor_esquerdo.setPower(-correction);
            motor_direito.setPower(correction);

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Parar os motores após atingir o alvo
        motor_esquerdo.setPower(0);
        motor_direito.setPower(0);
        sleep(200);
    }

        // Método para obter o ângulo de rotação (heading) da IMU
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

    public void moverAnexo1(int targetPosition) {

        anexo_1.setTargetPosition(targetPosition);

        while (opModeIsActive()) {
            double currentPosition = anexo_1.getCurrentPosition();
            double error = targetPosition - currentPosition;

            if (Math.abs(error) < 1.0) {
                break;
            }

            double correction = error * KpAnexo1;
            correction = Math.max(-1, Math.min(correction, 1));

            anexo_1.setPower(correction);
        }
    }

    public void moverAnexo2(int targetPosition) {

        anexo_2.setTargetPosition(targetPosition);

        while (opModeIsActive()) {
            double currentPosition = anexo_2.getCurrentPosition();
            double error = targetPosition - currentPosition;

            if (Math.abs(error) < 1.0) {
                break;
            }

            double correction = error * KpAnexo2;
            correction = Math.max(-1, Math.min(correction, 1));

            anexo_2.setPower(correction);
        }
    }

    public void vassoura(long tempo, boolean frente) {
        ElapsedTime elapsedTime = new ElapsedTime();  // Cria o cronômetro para medir o tempo de execução

        if (frente) {
            vassourinha.setPower(0.5);  // Inicia a vassourinha
        } else {
            vassourinha.setPower(-0.5);  // Inicia a vassourinha em sentido contrário
        }

        // Enquanto o tempo não tiver passado, continue a ação
        while (elapsedTime.milliseconds() < tempo) {
            // O robô continua executando o código enquanto o tempo não acabou
            // Não é necessário fazer mais nada aqui
            // O robô não vai parar de rodar a vassourinha até o tempo passar
        }

        // Depois que o tempo acabou, a vassourinha é desligada
        vassourinha.setPower(0);
    }

    public void garra(double position) {
        garrinha.setPosition(position);
    }
}