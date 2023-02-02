// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawMotor extends SubsystemBase {
  /** Creates a new ClawMotor. */
    public final WPI_TalonSRX claw;
    public final DigitalInput topLimitSwitch;
    public final DigitalInput bottomLimitSwitch;

   private static double kSensorSlope = -86.8;
   private static double kSensorOffset = 694.0;

  public ClawMotor() {
    claw = new WPI_TalonSRX(OperatorConstants.CLAW_MOTOR_ID);
    claw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    claw.setSensorPhase(true);
    claw.config_kF(0, 0);
		claw.config_kP(0, 2);
		claw.config_kI(0, 0);
		claw.config_kD(0,0.25);

    
    
    topLimitSwitch = new DigitalInput(0);
    bottomLimitSwitch = new DigitalInput(1);
    
  }
  public void ClawUp (){
    SetClaw(kCLAW_SPEED);
  }
  public void ClawDown (){
    SetClaw(-kCLAW_SPEED);
  }
  public void SetClaw(double speed){
      claw.set(safetyClamp(speed));
  }
  public void stopClaw(){
    claw.stopMotor();
  }
  private double ticksToAngle(double ticks){
    return 1/kSensorSlope * ticks - kSensorOffset / kSensorSlope;
  }

  private double angleToTicks(double angle){
    return kSensorSlope * angle + kSensorOffset;
  }
  public double getCurrentArmAngle(){
    return ticksToAngle(claw.getSelectedSensorPosition());
  }

  // Given a desired speed, returns a speed that should be sent to the motors based on the state of limit switches
  private double safetyClamp(double speed){
    if (topLimitSwitch.get() && speed > 0){
      return 0;
    }
    if (bottomLimitSwitch.get() && speed < 0){
      return 0;
    }
    return speed;
  }

  @Override
  public void periodic() {
    // if (topLimitSwitch.get() || bottom){
    //   stopClaw();
    // }
    // This method will be called once per scheduler run
  }
}
