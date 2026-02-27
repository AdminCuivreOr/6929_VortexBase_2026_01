package frc.robot.Intake;
 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
 
public class IntakeSubsystem extends SubsystemBase {
 
    private final SparkFlex intakeRight =
            new SparkFlex(11, MotorType.kBrushless);
    //private final SparkFlex intakeLeft =
       //     new SparkFlex(16, MotorType.kBrushless);
 
   // private final SparkFlex intakeRight =
   //        new SparkFlex(22, MotorType.kBrushless);
 
    public IntakeSubsystem() {
        // Inverser un moteur pour qu'ils tournent dans le même sens
    }
 
    public void drive(double speed) {
        intakeRight.set(speed);
        //intakeLeft.set(-speed);
    }
 
    public void stop() {
        intakeRight.set(0);
       // intakeLeft.set(0);
    }
}