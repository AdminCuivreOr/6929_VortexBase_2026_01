package frc.robot.subsystems;

 import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.SparkBaseConfig.IdleMode;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
 
import frc.robot.Constants;
 
public class TurretSubsystem extends SubsystemBase {
 
  private final CANSparkFlex m_motor =
      new CANSparkFlex(Constants.TurretConstants.MOTOR_ID, MotorType.kBrushless);
 
  // Through-Bore connected to SparkFlex data port (duty-cycle absolute)
  private final SparkAbsoluteEncoder m_absEncoder =
      m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
 
  private final ProfiledPIDController m_pid =
      new ProfiledPIDController(
          Constants.TurretConstants.kP,
          Constants.TurretConstants.kI,
          Constants.TurretConstants.kD,
          new TrapezoidProfile.Constraints(
              Constants.TurretConstants.MAX_VEL_RAD_PER_SEC,
              Constants.TurretConstants.MAX_ACCEL_RAD_PER_SEC2));
 
  public TurretSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(Constants.TurretConstants.CURRENT_LIMIT_A);
    m_motor.setInverted(Constants.TurretConstants.MOTOR_INVERTED);
 
    // Encoder outputs "rotations". We convert to radians at the turret.
    // If encoder is on turret axis: ENCODER_GEAR_RATIO = 1.0
    // If encoder is before gearing: set gear ratio accordingly.
    double radPerEncoderRotation =
        (2.0 * Math.PI) / Constants.TurretConstants.ENCODER_GEAR_RATIO;
 
    m_absEncoder.setPositionConversionFactor(radPerEncoderRotation); // rotations -> radians
    m_absEncoder.setVelocityConversionFactor(radPerEncoderRotation); // RPM-ish -> rad/s-ish (fine for debug)
 
    // Important: continuous input for wrap-around aiming
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
    m_pid.setTolerance(Constants.TurretConstants.ANGLE_TOLERANCE_RAD);
  }
 
  /** Turret angle in radians, wrapped to [-pi, pi], with your zero offset applied. */
  public double getAngleRad() {
    // Absolute encoder gives [0..2π) in our conversion, but can be any continuous value depending on REVLib behavior.
    double rawRad = m_absEncoder.getPosition();
 
    // Apply zero offset (calibrated)
    double adjusted = rawRad - Constants.TurretConstants.ZERO_OFFSET_RAD;
 
    return MathUtil.angleModulus(adjusted);
  }
 
  /** Command turret to an angle (radians), turret-relative. */
  public void setAngleSetpointRad(double targetRad) {
    // Optional: prevent cable wrap (strongly recommended)
    targetRad = MathUtil.clamp(
        targetRad,
        Constants.TurretConstants.MIN_ANGLE_RAD,
        Constants.TurretConstants.MAX_ANGLE_RAD
    );
 
    double output = m_pid.calculate(getAngleRad(), targetRad);
 
    // Optional static friction feedforward
    if (Math.abs(output) > 1e-4) {
      output += Math.copySign(Constants.TurretConstants.kS, output);
    }
 
    output = MathUtil.clamp(output, -Constants.TurretConstants.MAX_OUTPUT, Constants.TurretConstants.MAX_OUTPUT);
    m_motor.set(output);
 
    SmartDashboard.putNumber("Turret/SetpointDeg", Math.toDegrees(targetRad));
  }
 
  public void stop() {
    m_motor.stopMotor();
  }
 
  public boolean atSetpoint() {
    return m_pid.atGoal();
  }
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/AngleDeg", Math.toDegrees(getAngleRad()));
    SmartDashboard.putNumber("Turret/AbsRawRot", m_absEncoder.getPosition() / (2.0 * Math.PI)); // approx rotations
    SmartDashboard.putNumber("Turret/ErrorDeg", Math.toDegrees(m_pid.getPositionError()));
  }
}
