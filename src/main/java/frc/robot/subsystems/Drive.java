// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  public WPI_TalonSRX left, right;
  //public CANSparkMax testMotor;
  public DifferentialDrive drive;
  public Drive() {
    left = new WPI_TalonSRX(OperatorConstants.LEFT_MOTOR_ID);
    right = new WPI_TalonSRX(OperatorConstants.RIGHT_MOTOR_ID);
    right.setInverted(true);
    drive = new DifferentialDrive(left, right);
    //testMotor = new CANSparkMax(OperatorConstants.SPARK_MOTOR_ID , MotorType.kBrushless);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
