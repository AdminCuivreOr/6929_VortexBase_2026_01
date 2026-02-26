package frc.robot.subsystems;

 import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
 
import frc.robot.Constants;

import frc.robot.LimelightHelpers;
 
public class TurretSubsystem extends SubsystemBase {
 
  private final SparkMax m_motor =
      new SparkMax(Constants.TurretConstants.MOTOR_ID, MotorType.kBrushless);
  // Creates a PIDController with gains kP, kI, and kD
  PIDController pid = new PIDController(Constants.TurretConstants.kP, 0,0);

  DigitalInput m_limitSwitchGauche = new DigitalInput(7);
  DigitalInput m_limitSwitchDroite = new DigitalInput(8);


  // Through-Bore connected to SparkFlex data port (duty-cycle absolute)
  private final RelativeEncoder m_relEncoder =
      m_motor.getEncoder();
 
 
  public TurretSubsystem() {
    var config = new SparkMaxConfig();
    double degPerEncoderRotation = 0.78; //valeur encodeur 1 tour / 360, prendre la valeur dans le rev hardware client, sinon marche pas

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
    double rawDeg = m_relEncoder.getPosition(); // pas vraiment des degrés, valeur encodeur
 
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
 
    double output = pid.calculate(m_relEncoder.getPosition(), targetDeg);
    double position = getAngle();
    double erreur = targetDeg - position;

    if (erreur > Constants.TurretConstants.ANGLE_TOLERANCE_DEG) {
     m_motor.set(output);
    }
    else if (erreur < -Constants.TurretConstants.ANGLE_TOLERANCE_DEG)  {
     m_motor.set(output);
    }
    else {
    // si erreur plus grsnde que 5 moteur à 0.1, si plus petit que -5 moteur à -0.1 sinon moteur à 0
    m_motor.set(0);
    }
    SmartDashboard.putNumber("Turret/SetpointDeg", targetDeg);
    SmartDashboard.putNumber("Turret/erreur", erreur);
    SmartDashboard.putNumber("Turret/Output", output);


  }
 
  public void stop() {
    m_motor.stopMotor();
  }
 
 
  @Override
  public void periodic() {
      SmartDashboard.putNumber("Turret/AngleDeg", m_relEncoder.getPosition());
     if (m_limitSwitchDroite.get() == true){ // set l'angle à 90
   m_relEncoder.setPosition(Constants.TurretConstants.MAX_ANGLE_DEG);
    }

    if (m_limitSwitchGauche.get() == false){ // set l'angle à -90
 m_relEncoder.setPosition(Constants.TurretConstants.MIN_ANGLE_DEG);
    }
   
    
    
    SmartDashboard.putBoolean("limitSwitchDroite", m_limitSwitchDroite.get());
    SmartDashboard.putBoolean("limitSwitchGauche", m_limitSwitchGauche.get());


    SmartDashboard.putNumber("Turret/AngleDeg", getAngle()); 
    SmartDashboard.putNumber("Turret/CurrentAngleDeg",m_relEncoder.getPosition());
  }
}
