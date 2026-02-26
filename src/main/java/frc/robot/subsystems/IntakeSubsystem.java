package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
 
public class IntakeSubsystem extends SubsystemBase {
 
    private final SparkFlex intakeLeft =
            new SparkFlex(11, MotorType.kBrushless);
 
   // private final SparkFlex intakeRight =
   //        new SparkFlex(22, MotorType.kBrushless);
 
    public IntakeSubsystem() {
        // Inverser un moteur pour qu'ils tournent dans le même sens
    }
 
    public void drive(double speed) {
        intakeLeft.set(speed);
        //intakeRight.set(speed);
    }
 
    public void stop() {
        intakeLeft.set(0);
       // intakeRight.set(0);
    }
}