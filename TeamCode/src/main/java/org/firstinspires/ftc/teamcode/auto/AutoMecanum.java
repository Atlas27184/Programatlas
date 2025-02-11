package org.firstinspires.ftc.teamcode.auto;

// Importações
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name = "Autonomo com Mecanum")
public class AutoMecanum extends LinearOpMode {

    DcMotor superior_esquerdo;
    DcMotor superior_direito;
    DcMotor inferior_esquerdo;
    DcMotor inferior_direito;
    IMU imu;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_CIRCUMFERENCE_MM = 101.6 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    private static final double Kp = 0.06;
    private static final double KpTurn = 0.0099;

    @Override
    public void runOpMode() {
        // Inicializando os motores e sensores
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

        superior_direito.setDirection(DcMotorSimple.Direction.REVERSE);
        inferior_direito.setDirection(DcMotorSimple.Direction.REVERSE);


        superior_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        superior_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inferior_direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inferior_esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            moveWithEncoders(300, 0, 0.5, 0); // Move para frente 500 mm com ângulo-alvo de 0 graus
            moveWithEncoders(0, 300, 0.5, 0);
            rotateToAngle(-90);               // Gira para 90 graus
        }
    }

    public void moveWithEncoders(double forwardMM, double strafeMM, double power, double targetAngle) {
        int forwardTicks = (int) (forwardMM * DRIVE_COUNTS_PER_MM);
        int strafeTicks = (int) (strafeMM * DRIVE_COUNTS_PER_MM);

        superior_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inferior_esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        superior_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inferior_direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        superior_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        superior_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inferior_esquerdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inferior_direito.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentAngle;
        double error;
        double correction;
        double BRpower, BLpower, FLpower, FRpower;

        // Calcula o número de ticks a percorrer
        int startFL = superior_esquerdo.getCurrentPosition();
        int startFR = superior_direito.getCurrentPosition();
        int startBL = inferior_esquerdo.getCurrentPosition();
        int startBR = inferior_direito.getCurrentPosition();

        int targetFL = startFL + forwardTicks + strafeTicks;
        int targetFR = startFR + forwardTicks - strafeTicks;
        int targetBL = startBL + forwardTicks - strafeTicks;
        int targetBR = startBR + forwardTicks + strafeTicks;

        while (opModeIsActive() &&
                (Math.abs(superior_esquerdo.getCurrentPosition() - targetFL) > 10) && (Math.abs(superior_direito.getCurrentPosition() - targetFR) > 10)) {

            currentAngle = getHeading();
            error = normalizeAngle(targetAngle - currentAngle);
            correction = error * Kp;

            if (Math.abs(targetFL - startFL) < 10) {
                break;
            }

            if (targetFL > 0) {
                FLpower = power;
            } else { FLpower = -power;}

            if (targetFR > 0) {
                FRpower = power;
            } else { FRpower = -power;}

            if (targetBR > 0) {
                BRpower = power;
            } else { BRpower = -power;}

            if (targetBL > 0) {
                BLpower = power;
            } else { BLpower = -power;}

            double frontLeftPower = FLpower - correction;
            double frontRightPower = FRpower + correction;
            double backLeftPower = BLpower - correction;
            double backRightPower = BRpower + correction;

            superior_esquerdo.setPower(frontLeftPower);
            superior_direito.setPower(frontRightPower);
            inferior_esquerdo.setPower(backLeftPower);
            inferior_direito.setPower(backRightPower);

            telemetry.addData("Target FL", targetFL);
            telemetry.addData("Current FL", superior_esquerdo.getCurrentPosition());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        superior_esquerdo.setPower(0);
        superior_direito.setPower(0);
        inferior_esquerdo.setPower(0);
        inferior_direito.setPower(0);
    }

    public void rotateToAngle(double targetAngle) {
        double finalAngle = targetAngle + getHeading();
        while (opModeIsActive()) {
            double currentAngle = getHeading();
            double error = normalizeAngle(finalAngle - currentAngle);

            if (Math.abs(error) < 1.0) {
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
            telemetry.update();
        }

        superior_esquerdo.setPower(0);
        superior_direito.setPower(0);
        inferior_esquerdo.setPower(0);
        inferior_direito.setPower(0);
        sleep(200);
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
