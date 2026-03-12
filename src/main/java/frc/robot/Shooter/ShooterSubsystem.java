package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooter =
            new SparkFlex(10, MotorType.kBrushless); // shooter id 10

   // private final SparkFlex shooterRight =
    //        new SparkFlex(22, MotorType.kBrushless);

    public ShooterSubsystem() {
        // Inverser un moteur pour qu'ils tournent dans le même sens
    }

    public void drive(double speed) {
        shooter.set(speed);
        //shooterRight.set(speed);
    }

    public void stop() {
        shooter.set(0);
       // shooterRight.set(0);
    }
}
