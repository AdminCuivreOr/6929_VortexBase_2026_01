package frc.robot.Tube;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tube.TubeSubsystem;

public class TubeCommand extends Command {

    private final TubeSubsystem tube;
    private final double speed;
    private final double speedouter;

    public TubeCommand(TubeSubsystem tube, double speed, double speedouter) {
        this.tube = tube;
        this.speed = speed;
        this.speedouter = speedouter;

        addRequirements(tube);
    }

    @Override
    public void execute() {
        tube.drive(speed, speedouter);
    }

    @Override
    public void end(boolean interrupted) {
        tube.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}