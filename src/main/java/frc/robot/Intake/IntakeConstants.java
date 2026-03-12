// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


//import swervelib.math.Matter;

public final class IntakeConstants {

  public static final int motorID = 14;
  public static final double maxSpeed = 0.5;
 
  public static final double kp = 0.1;//02
  public static final double ki = 0.0;//015
  public static final double kd = 0.0;
  public static final double kf = 0.0;
  public static final double tolerance = 1.0;
 
  public static final double positionDefaultCount = 0; 
  public static final double Position1Count = -2.9;

  
}
