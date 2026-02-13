package frc.robot.subsystems;

 import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
 
import frc.robot.Constants;
 
public class TurretSubsystem extends SubsystemBase {
 
  private final SparkFlex m_motor =
      new SparkFlex(Constants.TurretConstants.MOTOR_ID, MotorType.kBrushless);
 
  // Through-Bore connected to SparkFlex data port (duty-cycle absolute)
  private final SparkAbsoluteEncoder m_absEncoder =
      m_motor.getAbsoluteEncoder();
 
 
  public TurretSubsystem() {
    var config = new SparkMaxConfig();
    double degPerEncoderRotation = 360.0 / 10.0;

    config.encoder.positionConversionFactor(degPerEncoderRotation);
 
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // Encoder outputs "rotations". We convert to radians at the turret.
    // If encoder is on turret axis: ENCODER_GEAR_RATIO = 1.0
    // If encoder is before gearing: set gear ratio accordingly.
 
    // Important: continuous input for wrap-around aiming
    
  }
 
  /** Turret angle in radians, wrapped to [-pi, pi], with your zero offset applied. */
  public double getAngle() {
    // Absolute encoder gives [0..2π) in our conversion, but can be any continuous value depending on REVLib behavior.
    double rawDeg = m_absEncoder.getPosition();
 
    // Apply zero offset (calibrated)
    double adjusted = rawDeg - Constants.TurretConstants.ZERO_OFFSET_DEG;

    return adjusted;
  }
 
  /** Command turret to an angle (radians), turret-relative. */
  public void moveToAngle(double targetDeg) {
    // Optional: prevent cable wrap (strongly recommended)
    targetDeg = MathUtil.clamp(
        targetDeg,
        Constants.TurretConstants.MIN_ANGLE_DEG,
        Constants.TurretConstants.MAX_ANGLE_DEG
    );
 
    double output = 0.1;
    double position = getAngle();
    double erreur = targetDeg - position;

    if (erreur > Constants.TurretConstants.ANGLE_TOLERANCE_DEG) {
     m_motor.set(output);
    }
    else if (erreur < -Constants.TurretConstants.ANGLE_TOLERANCE_DEG)  {
     m_motor.set(-output);
    }
    else {
    // si erreur plus grsnde que 5 moteur à 0,1, si plus petit que -5 moteur à -0,1 sinon moteur à 0
    m_motor.set(0);
    }
    SmartDashboard.putNumber("Turret/SetpointDeg", targetDeg);
  }
 
  public void stop() {
    m_motor.stopMotor();
  }
 
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/AngleDeg", getAngle());
  }
}
