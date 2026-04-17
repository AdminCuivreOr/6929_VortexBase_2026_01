package frc.robot.Intake.BrasIntake;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Intake.IntakeConstants;
 
public class BrasIntakeSubsystem extends SubsystemBase {

      private static double kS = 1.1;
    private static double kG = 1.2;
     private static double kV = 1.3;

    private final SparkMax m_BrasintakeMotor = new SparkMax(IntakeConstants.motorID, MotorType.kBrushless);
    private final RelativeEncoder m_BrasintakeEncoder = m_BrasintakeMotor.getEncoder();

    private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);

    private final ProfiledPIDController m_BrasintakePID = new ProfiledPIDController
        (IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd, m_constraints, IntakeConstants.kDt);

    
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

  // private final ProfiledPIDController m_BrasintakePID = new ProfiledPIDController(0.07, IntakeConstants.ki, IntakeConstants.kd, new Constraints(0.5, 0.5));
    
    private double setpoint;
    private double command;
 
    public BrasIntakeSubsystem() {
        m_BrasintakePID.setTolerance(IntakeConstants.tolerance);
        m_BrasintakeMotor.getEncoder().setPosition(0);
    }
 
    public void resetEncoder() {
        // Resets the encoder's position to zero
        m_BrasintakeEncoder.setPosition(0);
    }
 
    public double getPosition() {
        return m_BrasintakeEncoder.getPosition();
    }
 
    public void setPositionTarget(double sp){
        setpoint = sp;
        m_BrasintakePID.setGoal(sp);
    }
 
    public boolean atSetpoint(){
        return m_BrasintakePID.atGoal();
    }
 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("resultBras",(MathUtil.clamp((command + IntakeConstants.kf), -IntakeConstants.maxSpeed, IntakeConstants.maxSpeed)));
        SmartDashboard.putNumber("Voltage_Intake_motor", m_BrasintakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("setpoint_Intake", setpoint);
        SmartDashboard.putNumber("currentposIntake", getPosition());
        SmartDashboard.putNumber("commandIntake", command);
        SmartDashboard.putNumber("Intake_speed", MathUtil.clamp((command + IntakeConstants.kf), -IntakeConstants.maxSpeed, IntakeConstants.maxSpeed));
        SmartDashboard.putNumber("ALternate_Intakespeed", MathUtil.clamp((IntakeConstants.kf), -IntakeConstants.maxSpeed, IntakeConstants.maxSpeed));
        SmartDashboard.putNumber("intake Position (Rotations)", getPosition());

        if(RobotState.isEnabled()){
              m_BrasintakeMotor.setVoltage(
              m_BrasintakePID.calculate(m_BrasintakeEncoder.getPosition())
               + m_feedforward.calculate(m_BrasintakePID.getSetpoint().velocity));
                
            }
        }
        
 
       
    }
