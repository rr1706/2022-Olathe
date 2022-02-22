// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.JoystickAnalogButton;
import frc.robot.Utilities.JoystickAnalogButton.Side;
import frc.robot.commands.DriveByController;
import frc.robot.commands.RunElevators;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   // The driver's controllers
   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
   XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_robotDrive = new Drivetrain(); // Create Drivetrain Subsystem
  
  private final Intake m_leftIntake = new Intake(IntakeConstants.kLeftMotorID, IntakeConstants.kLeftAirPorts, "Left");
  private final Intake m_rightIntake = new Intake(IntakeConstants.kRightMotorID, IntakeConstants.kRightAirPorts, "Right");
  
  private final Elevator m_lowElevator = new Elevator(ElevatorConstants.kLowMotorID, ElevatorConstants.kLowSensor, "Low");
  private final Elevator m_highElevator = new Elevator(ElevatorConstants.kHighMotorID, ElevatorConstants.kHighSensor, "High");

  private final Shooter m_shooter = new Shooter(ShooterConstants.kMotorIDs);

  private final RunIntake m_runLeftIntake = new RunIntake(m_leftIntake);
  private final RunIntake m_runRightIntake = new RunIntake(m_rightIntake);

  private final RunElevators m_runElevators = new RunElevators(m_lowElevator, m_highElevator);

  private final RunShooter m_runShooter = new RunShooter(m_shooter);

  private final DriveByController m_drive = new DriveByController(m_robotDrive, m_driverController);

  private final Command autoFiveBall = new WaitCommand(20.0);
  private final Command autoThreeBall = new WaitCommand(20.0);
  private final Command autoOneBall = new WaitCommand(20.0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoChooser();

    m_robotDrive.setDefaultCommand(m_drive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(m_driverController, 0)
        .whenPressed(() -> m_robotDrive.resetOdometry(new Rotation2d(0.0)));
    new POVButton(m_driverController, 180)
        .whenPressed(() -> m_robotDrive.resetOdometry(new Rotation2d(Math.PI)));

    new JoystickAnalogButton(m_driverController, Side.kRight).whenPressed(m_runRightIntake).whenReleased(()->m_runRightIntake.cancel());
    new JoystickAnalogButton(m_driverController, Side.kLeft).whenPressed(m_runLeftIntake).whenReleased(()->m_runLeftIntake.cancel());
    
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_runShooter);
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(()->m_runShooter.cancel());

    new JoystickButton(m_driverController, Button.kX.value).whenPressed(m_runElevators);
    new JoystickButton(m_driverController, Button.kY.value).whenPressed(()->m_runElevators.cancel());

  }

private void configureAutoChooser(){
  m_chooser.addOption("Auto3Ball", autoThreeBall);
  m_chooser.addOption("Auto5Ball", autoFiveBall);
  m_chooser.setDefaultOption("Auto1Ball", autoOneBall);
  SmartDashboard.putData(m_chooser);  
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
