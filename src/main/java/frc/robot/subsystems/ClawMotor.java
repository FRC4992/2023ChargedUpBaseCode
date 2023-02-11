// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmLevels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import javax.management.OperationsException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawMotor extends SubsystemBase {
  /** Creates a new ClawMotor. */
    public final WPI_TalonSRX claw;
    public final DigitalInput topLimitSwitch;
    public final DigitalInput bottomLimitSwitch;
    public final PIDController pid;

  //  private static double kSensorSlope = -86.8; // ticks / deg
  //  private static double sensorOffset = 694.0;

   private static double kTickToAngleSlope = -0.0122;
   private static final double kARM_BOTTOM_LIMIT_SWITCH_ANGLE = 22;
   private static final double kARM_TOP_LIMIT_SWITCH_ANGLE = 142;

   private static double lastKnownAngle = 6;
   private static double lastKnownTick = 7972;

   private static ArmLevels targetLevel;

   private static boolean isAtTarget;



   public static double sensorZeroValue = 0;

  public ClawMotor() {
    claw = new WPI_TalonSRX(OperatorConstants.CLAW_MOTOR_ID);
    claw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    claw.setSensorPhase(true);
    claw.config_kF(0, 0);
		claw.config_kP(0, 1.75);
		claw.config_kI(0, 0);
		claw.config_kD(0,0.45);
    resetEncoder(kARM_BOTTOM_LIMIT_SWITCH_ANGLE);
    topLimitSwitch = new DigitalInput(0);
    bottomLimitSwitch = new DigitalInput(1);
    pid = new PIDController(0.04, 0, 0.005);
    pid.setTolerance(3.0);
    
  }
  public void ClawUp (){
    SetClaw(Constants.kCLAW_SPEED);
  }
  public void ClawDown (){
    SetClaw((Constants.kCLAW_SPEED)*-1);
  }
  public void SetClaw(double speed){
      claw.set(safetyClamp(speed));
  }
  public void stopClaw(){
    claw.stopMotor();
  }
  private double ticksToAngle(double ticks){
    // kSensorSlope = ticks / deg
    // 1 / kSensorSlope = deg / tick
    // tick / (deg)/(tick)
    return (kTickToAngleSlope*(ticks-lastKnownTick))+lastKnownAngle;

  }
  public void setArmLevel(ArmLevels level){
    targetLevel = level;
    System.out.println("setting target level to " + level);
  }

  // private double angleToTicks(double angle){
  //   return kSensorSlope * angle + sensorZeroValue;
  // }
  public double getCurrentArmAngle(){
    return ticksToAngle(claw.getSelectedSensorPosition());
  }

  public void resetEncoder(double knownAngle){
    lastKnownAngle = knownAngle;
    lastKnownTick = claw.getSelectedSensorPosition();
  }
  
  public boolean isAtTarget(){
    return isAtTarget;
  }

  // Given a desired speed, returns a speed that should be sent to the motors based on the state of limit switches
  private double safetyClamp(double speed){
    if (topLimitSwitch.get() && speed > 0){
      return 0;
    }
    if (bottomLimitSwitch.get() && speed < 0){
      return 0;
    }
    if (speed > Constants.kCLAW_SPEED){
      return Constants.kCLAW_SPEED;
    }
    if (speed < -Constants.kCLAW_SPEED){
      return -Constants.kCLAW_SPEED;
    }
    return speed;
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.get()){
      System.out.println("Bottomed Out");
      resetEncoder(kARM_BOTTOM_LIMIT_SWITCH_ANGLE);
    }
    if (topLimitSwitch.get()){
      System.out.println("Topped Out");
      resetEncoder(kARM_TOP_LIMIT_SWITCH_ANGLE);
    }
    if (targetLevel == null){
      //System.out.println("target not set");
      stopClaw();
      isAtTarget = true;
    }
    else{
      double setpoint = targetLevel.armAngle();
      double computed_speed = pid.calculate(getCurrentArmAngle(), setpoint);
      double clamped_speed = safetyClamp(computed_speed);
      System.out.println(setpoint);
      System.out.println(computed_speed);
      System.out.println(clamped_speed);
      if (!pid.atSetpoint()){
        claw.set(clamped_speed);
        isAtTarget = false;

      }else{
        claw.set(0);
        isAtTarget = true;
      }
      //System.out.println(clamped_speed);
   }
    // if (topLimitSwitch.get() || bottom){
    //   stopClaw();
    // }
    // This method will be called once per scheduler run
  }
}