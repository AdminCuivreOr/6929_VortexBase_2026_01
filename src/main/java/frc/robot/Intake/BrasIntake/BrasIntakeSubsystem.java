package frc.robot.Intake.BrasIntake;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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

    private final double kp = 0.2;
    private final double maxVelocity = 3;
    private final double maxAcceleration = 3;


    private final SparkMax m_BrasintakeMotor = new SparkMax(IntakeConstants.motorID, MotorType.kBrushless);
    private final RelativeEncoder m_BrasintakeEncoder = m_BrasintakeMotor.getEncoder();
    private final ProfiledPIDController m_BrasintakePID = 
        new ProfiledPIDController(kp,0,0,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    private ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.02, 0.0);

  // private final ProfiledPIDController m_BrasintakePID = new ProfiledPIDController(0.07, IntakeConstants.ki, IntakeConstants.kd, new Constraints(0.5, 0.5));
    
    private double setpoint;
    private double command;
 
    public BrasIntakeSubsystem() {
        m_BrasintakePID.setTolerance(/*IntakeConstants.tolerance*/1);
        m_BrasintakeMotor.getEncoder().setPosition(0);
    }

    public void resetEncoder() {
        // Resets the encoder's position to zero
        m_BrasintakeEncoder.setPosition(0);
    }
 
    public double getPosition() {
        return m_BrasintakeEncoder.getPosition();
    }

    public double getVitesse() {
        return m_BrasintakeEncoder.getVelocity();
    }

    public void resetPID(){

        
    }

    public void setPositionTarget(double cible){
        setpoint = cible;

        double moteurPID = m_BrasintakePID.calculate(getPosition(), cible);
        double moteurFF = feedforward.calculate(getPosition(), m_BrasintakePID.getSetpoint().velocity);

        m_BrasintakeMotor.set(moteurPID + moteurFF);
        SmartDashboard.putNumber("PROFILED PID FF", moteurFF + moteurPID);
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

           
        
           
           // m_BrasintakeMotor.set(MathUtil.
            //clamp((command + IntakeConstants.kf), -IntakeConstants.maxSpeed, IntakeConstants.maxSpeed));
                
                
            }
        }
        
 
       
    }
