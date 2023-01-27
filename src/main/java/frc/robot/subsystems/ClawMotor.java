// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawMotor extends SubsystemBase {
  /** Creates a new ClawMotor. */
    public final WPI_TalonSRX claw;
    public final DigitalInput topLimitSwitch;


  public ClawMotor() {
    claw = new WPI_TalonSRX(OperatorConstants.CLAW_MOTOR_ID);
    topLimitSwitch = new DigitalInput(0);
  }
  public void ClawUp (){
    if(canMove()){
      claw.set(0.5);
    }
  }
  public void ClawDown (){
    if (canMove()){
      claw.set(-0.5);
    }
  }
  public void SetClaw(double speed){
    if (canMove()){
      claw.set(speed);
    }
  }
  public void stopClaw(){
    claw.stopMotor();
  }
  private boolean canMove(){
    return topLimitSwitch.get();
  }

  @Override
  public void periodic() {
    if (!canMove()){
      stopClaw();
    }
    // This method will be called once per scheduler run
  }
}
