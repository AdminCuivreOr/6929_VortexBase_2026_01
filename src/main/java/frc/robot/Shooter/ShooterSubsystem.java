package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooter =
            new SparkFlex(10, MotorType.kBrushless); // shooter id 10
            PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI,ShooterConstants.kD);
            SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, ShooterConstants.kFF);

  private final RelativeEncoder m_relEncoder =
      shooter.getEncoder();

    public ShooterSubsystem() {
        System.out.println(m_relEncoder.getVelocity());
        double Rpmveut = m_relEncoder.getVelocity();
    }

   public void setRPM(double rpm) {

    double pidOutput = pid.calculate(m_relEncoder.getVelocity(), rpm);
    double ffOutput = ff.calculate(rpm);

    double output = pidOutput + ffOutput;
    output = Math.max(-1, Math.min(1, output));

    shooter.set(output);

    SmartDashboard.putNumber("RPM",(m_relEncoder.getVelocity()));
}

   public void stop() {
    shooter.set(0);
    pid.reset(); 
}
}
