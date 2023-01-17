// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  public Talon left, right;
  public DifferentialDrive drive;
  public Drive() {
    left = new Talon(OperatorConstants.LEFT_MOTOR_ID);
    right = new Talon(OperatorConstants.RIGHT_MOTOR_ID);
    drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
