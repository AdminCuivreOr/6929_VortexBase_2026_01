// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climb;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
   private final SparkFlex m_Climb =
            new SparkFlex(15, MotorType.kBrushless); //Climb id 15
  /** Creates a new ExampleSubsystem. */
  
  public ClimbSubsystem() {
        // Inverser un moteur pour qu'ils tournent dans le même sens
    }
 
    public void drive(double speed) {
        m_Climb.set(speed);
        //intakeLeft.set(-speed);
    }
 
    public void stop() {
        m_Climb.set(0);
       // intakeLeft.set(0);
    }
}