// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmLevels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawDown;
import frc.robot.commands.ClawUp;
import frc.robot.commands.CloseIntake;
import frc.robot.commands.OpenIntake;
import frc.robot.commands.SetClaw;
import frc.robot.subsystems.ClawMotor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drive drive = new Drive();
  public static Intake intake = new Intake();
  public static ArcadeDrive arcadeDriveCommand = new ArcadeDrive();
  public static Joystick stick = new Joystick(OperatorConstants.JOYSTICK_ID);
  public static Joystick controlPanel = new Joystick(OperatorConstants.CONTROL_PANEL_ID);
  public static ClawMotor clawArm = new ClawMotor();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    JoystickButton closeJoystickButton = new JoystickButton(stick, 1); // a 
    closeJoystickButton.onTrue(new CloseIntake()); 

    JoystickButton openJoystickButton = new JoystickButton(stick, 2); // b
    openJoystickButton.onTrue(new OpenIntake());
  
    JoystickButton topHeightButton = new JoystickButton(controlPanel, 3);
    // set height of claw to something random with the top black button
    topHeightButton.whileTrue(new SetClaw(ArmLevels.LEVEL3));

    JoystickButton middleHeightButton = new JoystickButton(controlPanel, 14);
    // set height of claw with second to middle black button
    middleHeightButton.whileTrue(new SetClaw(ArmLevels.LEVEL2));

    JoystickButton bottomHeightButton = new JoystickButton(controlPanel, 15);
    // set height button black button
    bottomHeightButton.whileTrue(new SetClaw(ArmLevels.LEVEL1));

    JoystickButton startingHeightButton = new JoystickButton(controlPanel, 6);
    startingHeightButton.whileTrue(new SetClaw(ArmLevels.BOTTOM));
    


    // set height with green button

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
