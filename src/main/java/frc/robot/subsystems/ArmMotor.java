// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmLevels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import javax.management.OperationsException;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxAbsoluteEncoder;

public class ArmMotor extends SubsystemBase {
  public static CANSparkMax arm;

  /** Creates a new ArmMotor. */

  public ArmMotor() {
    arm = new CANSparkMax(Constants.OperatorConstants.ARM_MOTOR_ID,MotorType.kBrushless);
    //arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  public void ArmUp() {
    SetArm((Constants.kARM_SPEED_OUT)*-1);
  }

  public void ArmDown() {
    SetArm((Constants.kARM_SPEED_IN));
  }

  public void SetArm(double speed) {
    arm.set(speed);
  }
  
  public void stopArm() {
    arm.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
