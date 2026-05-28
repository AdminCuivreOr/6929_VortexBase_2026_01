package frc.robot.Shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveAndAuto.SwerveSubsystem;
// avoir un bouton avec un shoot constant
public class ShooterCommandConstant extends Command {

    private final ShooterSubsystem shooter;


    public ShooterCommandConstant(ShooterSubsystem shooter) {
        this.shooter = shooter;
       

        addRequirements(shooter);
    }

    @Override
    public void execute() {

        // appliquer au shooter
        shooter.setRPM(-7000);

        
    }


    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // tourne tant que le bouton est maintenu
    }
}