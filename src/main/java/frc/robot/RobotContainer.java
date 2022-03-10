// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.JoystickAnalogButton;
import frc.robot.Utilities.JoystickAnalogButton.Side;
import frc.robot.commands.ClimbFromFloor;
import frc.robot.commands.ClimbNextBar;
import frc.robot.commands.DriveByController;
import frc.robot.commands.FaceTurret;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.InitiateClimbMode;
import frc.robot.commands.RunElevators;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ZeroClimb;
import frc.robot.commands.ZeroHood;
import frc.robot.commands.Autos.FiveBall;
//import frc.robot.commands.Autos.SixBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

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
  
  private final Elevator m_lowElevator = new Elevator(ElevatorConstants.kLowMotorID, ElevatorConstants.kLowSensor, "Low",10000.0);
  private final Elevator m_highElevator = new Elevator(ElevatorConstants.kHighMotorID, ElevatorConstants.kHighSensor, "High",10000.0);

  private final Climber m_climber = new Climber();

  private final Turret m_turret = new Turret();

  private final Shooter m_shooter = new Shooter(ShooterConstants.kMotorIDs);

  private final ShooterHood m_hood = new ShooterHood();

  private final IndexElevator m_indexElevator = new IndexElevator(m_highElevator, m_lowElevator);
  private final FeedShooter m_feedShooter = new FeedShooter(m_turret, m_shooter, m_hood, m_highElevator, m_lowElevator, m_robotDrive);
  private final ZeroClimb m_ZeroClimb = new ZeroClimb(m_climber);
  private final ZeroHood m_ZeroHood = new ZeroHood(m_hood);

  private final RunIntake m_runLeftIntake = new RunIntake(m_leftIntake);
  private final RunIntake m_runRightIntake = new RunIntake(m_rightIntake);

  private final InitiateClimbMode m_climbMode = new InitiateClimbMode(m_shooter, m_hood, m_turret, m_leftIntake, 
    m_rightIntake, m_highElevator, m_lowElevator, m_robotDrive, m_driverController, m_climber);
  private final ClimbFromFloor m_climbFromFloor = new ClimbFromFloor(m_climber);
  private final ClimbNextBar m_climbToHigh= new ClimbNextBar(m_climber);
  private final ClimbNextBar m_climbToTraversal= new ClimbNextBar(m_climber);

  private final RunShooter m_runShooter = new RunShooter(m_shooter, m_turret, m_robotDrive, m_hood, true);
  private final RunElevators m_runElevators = new RunElevators(m_lowElevator, m_highElevator);

  private final DriveByController m_drive = new DriveByController(m_robotDrive, m_driverController);

  private final FaceTurret m_faceTurret = new FaceTurret(m_turret, m_robotDrive); // Create FaceTurret Command

  private final Command autoFiveBall = new FiveBall(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator, m_highElevator, m_turret, m_hood, m_shooter, m_climber);
  //private final Command autoSixBall = new SixBall(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator, m_highElevator, m_turret, m_hood, m_shooter, m_climber);
  private final Command autoThreeBall = new WaitCommand(20.0);
  private final Command autoOneBall = new WaitCommand(20.0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoChooser();

    m_turret.setDefaultCommand(m_faceTurret);
    m_robotDrive.setDefaultCommand(m_drive);
    m_climber.setDefaultCommand(new RunCommand(()->m_climber.run(), m_climber));
    m_hood.setDefaultCommand(new RunCommand(()->m_hood.run(5.0),m_hood));
    m_lowElevator.setDefaultCommand(m_indexElevator);
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

    new JoystickAnalogButton(m_driverController, Side.kRight).whenPressed(m_runRightIntake)
      .whenReleased(new InstantCommand(()->m_runRightIntake.cancel())
        .andThen(new WaitCommand(1.0).andThen(()->m_rightIntake.stop()))
        .withInterrupt(()->m_runRightIntake.isScheduled()));

    new JoystickAnalogButton(m_driverController, Side.kLeft).whenPressed(m_runLeftIntake)
      .whenReleased(new InstantCommand(()->m_runLeftIntake.cancel())
        .andThen(new WaitCommand(1.0).andThen(()->m_leftIntake.stop()))
        .withInterrupt(()->m_runLeftIntake.isScheduled()));
    
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_runShooter);
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(()->m_runShooter.cancel());

    new JoystickButton(m_driverController, Button.kRightBumper.value).whenPressed(m_feedShooter.withInterrupt(()->!m_runShooter.isScheduled())).whenReleased(()->m_feedShooter.stop());

    new JoystickButton(m_operatorController, Button.kRightBumper.value).whenPressed(m_ZeroHood);
    new JoystickButton(m_operatorController, Button.kLeftBumper.value).whenPressed(m_ZeroClimb);
    //new JoystickButton(m_operatorController, Button.kA.value).whenPressed(()->m_climber.extend());
    //new JoystickButton(m_operatorController, Button.kB.value).whenPressed(()->m_climber.retract());

    new JoystickButton(m_operatorController, Button.kBack.value).whenPressed(m_climbMode);
    new JoystickButton(m_operatorController, Button.kStart.value).whenPressed(()->m_climbMode.cancel());

    new JoystickButton(m_operatorController, Button.kA.value).whenPressed(new ConditionalCommand(m_climbFromFloor, new WaitCommand(0.0), ()->m_climbMode.isClimbModeReady()));
    new JoystickButton(m_operatorController, Button.kX.value).whenPressed(new ConditionalCommand(m_climbToHigh, new WaitCommand(0.0), ()->m_climbFromFloor.isFinished()));
    new JoystickButton(m_operatorController, Button.kY.value).whenPressed(new ConditionalCommand(m_climbToTraversal, new WaitCommand(0.0), ()->m_climbToHigh.isFinished()));
    // new JoystickButton(m_operatorController, Button.kB.value).whenPressed(m_climbFromFloor.andThen(m_climbToHigh).andThen(m_climbToTraversal));

  }

private void configureAutoChooser(){
  m_chooser.addOption("Auto3Ball", autoThreeBall);
  m_chooser.addOption("Auto5Ball", autoFiveBall);
  //m_chooser.addOption("Auto6Ball", autoSixBall);
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
