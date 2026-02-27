package frc.robot.Tube;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TubeSubsystem extends SubsystemBase {

    private final SparkFlex underTube =
            new SparkFlex(12, MotorType.kBrushless);

    private final SparkFlex outer =
            new SparkFlex(13, MotorType.kBrushless);

   // private final SparkFlex shooterRight =
    //        new SparkFlex(22, MotorType.kBrushless);

    public TubeSubsystem() {
        // Inverser un moteur pour qu'ils tournent dans le même sens#
    }

    public void drive(double speed,double speedouter) {
        underTube.set(speed);
        outer.set(speedouter);
        //shooterRight.set(speed);
    }

    public void stop() {
        underTube.set(0);
        outer.set(0);
       // shooterRight.set(0);
    }
}
