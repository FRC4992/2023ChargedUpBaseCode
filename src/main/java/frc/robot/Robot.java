// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.ClawMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ArmLevels;

// april tag imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private double startTime;
 
  public static final String kArmFKey = "ArmF";
  public static final String kArmPKey = "ArmP";
  public static final String kArmIKey = "ArmI";
  public static final String kArmDKey = "ArmD";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    if(!Preferences.containsKey(kArmPKey)){

      Preferences.setDouble(kArmPKey, ClawMotor.armKP);

    }
    if(!Preferences.containsKey(kArmIKey)){

      Preferences.setDouble(kArmIKey, ClawMotor.armKI);

    }
    if(!Preferences.containsKey(kArmDKey)){

      Preferences.setDouble(kArmDKey, ClawMotor.armKD);

    }
    if(!Preferences.containsKey(kArmFKey)){

      Preferences.setDouble(kArmFKey, ClawMotor.armKF);

    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    startTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
   

    if (time -startTime < 1) {
      m_robotContainer.drive.drive.arcadeDrive(0.9, 0.0);
    }

    /* 
    if (pitchAngleDegrees is within a certain range) {
      drive forward
    } 

    if (pitchAngleDegrees is within another range) {
      drive backwards
    }
    */
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if(ClawMotor.armKP != Preferences.getDouble(kArmPKey, ClawMotor.armKP)){

      ClawMotor.armKP = Preferences.getDouble(kArmPKey, ClawMotor.armKP);
      ClawMotor.claw.config_kP(0,ClawMotor.armKP);

    }
    if(ClawMotor.armKI != Preferences.getDouble(kArmIKey, ClawMotor.armKI)){

      ClawMotor.armKI = Preferences.getDouble(kArmIKey, ClawMotor.armKI);
      ClawMotor.claw.config_kI(0,ClawMotor.armKI);

    }
    if(ClawMotor.armKD != Preferences.getDouble(kArmDKey, ClawMotor.armKD)){

      ClawMotor.armKD = Preferences.getDouble(kArmDKey, ClawMotor.armKD);
      ClawMotor.claw.config_kD(0,ClawMotor.armKD);

    }
    if(ClawMotor.armKF != Preferences.getDouble(kArmFKey, ClawMotor.armKF)){

      ClawMotor.armKF = Preferences.getDouble(kArmFKey, ClawMotor.armKF);
      ClawMotor.claw.config_kF(0,ClawMotor.armKF);

    }


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.arcadeDriveCommand.execute();
    // double pitchAngleDegrees = ahrs.getPitch();
    double ticks = m_robotContainer.clawArm.claw.getSelectedSensorPosition();
    double zeroPos = m_robotContainer.clawArm.sensorZeroValue;

    // april tag
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

   

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area); 
     // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 24.0;

    // distance from the target to the floor
    double goalHeightInches = 18.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    SmartDashboard.putNumber("RobotToTagDist", distanceFromLimelightToGoalInches); 
    
    System.out.println(ClawMotor.armKP);
    System.out.println(ClawMotor.armKI);
    System.out.println(ClawMotor.armKD);
    System.out.println(ClawMotor.armKF);

    double trigger_val = -m_robotContainer.m_driverController.getLeftTriggerAxis()*10;
    trigger_val += m_robotContainer.m_driverController.getRightTriggerAxis()*10;

    System.out.println("Trigger_val:"+trigger_val);

     RobotContainer.clawArm.SetClaw(trigger_val);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
