package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class TwoBallOne extends SequentialCommandGroup {

    public TwoBallOne(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb){
        final AutoFromPathPlanner fiveBallUno = new AutoFromPathPlanner(drivetrain, "2022-2BallAuto-One", 3.2);
        final RunIntake runIntake = new RunIntake(leftIntake);
        final FeedShooter m_autoFeed = new FeedShooter(turret, shooter, hood, top, bottom, drivetrain);

        addCommands(
            new InstantCommand(()->drivetrain.resetOdometry(fiveBallUno.getInitialPose())),
            new ParallelCommandGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false),
                new SequentialCommandGroup(
                    fiveBallUno.raceWith(runIntake).alongWith(new IndexElevator(top, bottom)),
                    m_autoFeed.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed.stop())))))
        );
    }
        
}