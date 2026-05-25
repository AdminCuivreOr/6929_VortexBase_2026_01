package frc.robot.Turret;

import frc.robot.LimelightHelpers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    // =========================
    // MOTEUR
    // =========================

    private final SparkMax m_motor =
        new SparkMax(TurretConstants.MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder m_encoder =
        m_motor.getEncoder();

    // =========================
    // LIMIT SWITCHES
    // =========================



    // =========================
    // PID POSITION
    // =========================

    private final PIDController pid =
        new PIDController(
            TurretConstants.kP,
            TurretConstants.kI,
            TurretConstants.kD
        );

    // =========================
    // CONSTRUCTOR
    // =========================

    public TurretSubsystem() {

        SparkMaxConfig config = new SparkMaxConfig();

        /*
         * Exemple :
         * moteur 100 tours = tourelle 1 tour
         *
         * donc :
         * 1 tour moteur = 3.6 degrés tourelle
         */

        double degreesPerMotorRotation = 3.6;

        config.encoder.positionConversionFactor(
            degreesPerMotorRotation
        );

        m_motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        pid.setTolerance(
            TurretConstants.ANGLE_TOLERANCE_DEG
        );
    }

    // =========================
    // ANGLE ACTUEL
    // =========================

    public double getAngle() {

        return m_encoder.getPosition()
            - TurretConstants.ZERO_OFFSET_DEG;
    }

    // =========================
    // PID POSITION
    // =========================

    public void moveToAngle(double targetDeg) {

        // Limites physiques
        targetDeg = MathUtil.clamp(
            targetDeg,
            TurretConstants.MIN_ANGLE_DEG,
            TurretConstants.MAX_ANGLE_DEG
        );

        double currentAngle = getAngle();

        double output =
            pid.calculate(currentAngle, targetDeg);

        // Limite vitesse moteur
        output = MathUtil.clamp(output, -0.4, 0.4);

        double error = targetDeg - currentAngle;

        // Stop si proche
        if (Math.abs(error)
            < TurretConstants.ANGLE_TOLERANCE_DEG) {

            output = 0;
        }

        // Protection limit switches
       
      
        m_motor.set(output);

        SmartDashboard.putNumber(
            "Turret/TargetAngle",
            targetDeg
        );

        SmartDashboard.putNumber(
            "Turret/CurrentAngle",
            currentAngle
        );

        SmartDashboard.putNumber(
            "Turret/Error",
            error
        );

        SmartDashboard.putNumber(
            "Turret/PIDOutput",
            output
        );
    }

    // =========================
    // MANUAL MOTOR CONTROL
    // =========================

    public void setMotor(double speed) {

        speed = MathUtil.clamp(speed, -0.4, 0.4);

       
       

        m_motor.set(speed);
    }

    // =========================
    // STOP
    // =========================

    public void stop() {

        m_motor.stopMotor();
    }

    // =========================
    // PERIODIC
    // =========================

    @Override
    public void periodic() {

        SmartDashboard.putNumber(
            "Turret/AngleDeg",
            getAngle()
        );

       
       
    }
}