// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { // make all caps
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int JOYSTICK_ID = 0;
    public static final int CLAW_MOTOR_ID = 3;
    public static final int RIGHT_MOTOR_ID = 4;
    public static final int LEFT_MOTOR_ID = 5;
    public static final int INTAKE_OPEN = 0;
    public static final int INTAKE_CLOSE = 1;
 
    // Control panel ID
    public static final int CONTROL_PANEL_ID = 1;
 
 
  }
  // Speed Constants:
  //public static final double kCLAW_SPEED = 0.4;
  public static final double kCLAW_SPEED_UP = 0.4;
  public static final double kCLAW_SPEED_DOWN = 0.2;
 
 
  // Arm levels
  // To-do : Add actual angle values
  /* 
   * angle = sin^-1(height/length of arm)
   */
  public static enum ArmLevels {
    BOTTOM(30),
    LEVEL1(83),
    LEVEL2(102),
    LEVEL3(122);
 
    private final double armAngle;
 
    private ArmLevels(double armAngle) {
      this.armAngle = armAngle;
    }
 
    public double armAngle() { return this.armAngle; }
  }
 
}