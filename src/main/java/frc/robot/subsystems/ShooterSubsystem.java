package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax; 
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooterLeft =
            new SparkFlex(23, MotorType.kBrushless);

   // private final SparkFlex shooterRight =
    //        new SparkFlex(22, MotorType.kBrushless);

    public ShooterSubsystem() {
        // Inverser un moteur pour qu'ils tournent dans le même sens
    }

    public void drive(double speed) {
        shooterLeft.set(speed);
        //shooterRight.set(speed);
    }

    public void stop() {
        shooterLeft.set(0);
       // shooterRight.set(0);
    }
}
