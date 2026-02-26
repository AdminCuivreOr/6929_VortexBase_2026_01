/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ActuatorSubsystem;

public class MoveActuatorCommand extends Command {
    private final ActuatorSubsystem m_actuator;
    private final boolean extend;

    public MoveActuatorCommand(ActuatorSubsystem actuator, boolean extend) {
        m_actuator = actuator;
        this.extend = extend;
        addRequirements(m_actuator);
    }

    @Override
    public void execute() {
        if (extend) {
            m_actuator.moveIncrement(0.01); // petit pas vers l’extension
        } else {
            m_actuator.moveIncrement(-0.01); // petit pas vers la rétraction
        }
    }

    @Override
    public boolean isFinished() {
        return false; // continue tant que le bouton est appuyé
    }
} */