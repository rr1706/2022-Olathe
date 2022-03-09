package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class FiveBall extends SequentialCommandGroup{
    public final Drivetrain m_driveTrain;
    public final Intake m_leftIntake;
    public final Intake m_rightIntake;
    public final Elevator m_bottom;
    public final Elevator m_top;
    public final Turret m_turret;
    public final ShooterHood m_hood;
    public final Shooter m_shooter;
    public FiveBall(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter){
        m_driveTrain = drivetrain;
        m_leftIntake = leftIntake;
        m_rightIntake = rightIntake;
        m_bottom = bottom;
        m_top = top;
        m_turret = turret;
        m_hood = hood;
        m_shooter = shooter;

        final AutoFromPathPlanner fiveBallUno = new AutoFromPathPlanner(drivetrain, "20225BallAuto-uno", 3.2);
        final AutoFromPathPlanner fiveBallDos = new AutoFromPathPlanner(drivetrain, "20225BallAuto-dos", 3.2);
        final AutoFromPathPlanner fiveBallTres = new AutoFromPathPlanner(drivetrain, "20225BallAuto-tres", 3.2);
        final AutoFromPathPlanner fiveBallQuatro = new AutoFromPathPlanner(drivetrain, "20225BallAuto-quatro", 3.2);
        final FeedShooter feedShooter = new FeedShooter(m_turret, m_shooter, m_hood, m_bottom, m_top);
        final RunShooter runShooter = new RunShooter(m_shooter, m_turret, m_driveTrain, m_hood);
        final RunIntake runLeftIntake = new RunIntake(m_leftIntake);
        final RunIntake runRightIntake = new RunIntake(m_rightIntake);

        addCommands(
            feedShooter.alongWith(runShooter).raceWith(new WaitCommand(0.2)),
            runLeftIntake.raceWith(fiveBallUno),
            runLeftIntake.raceWith(fiveBallDos),
            feedShooter.alongWith(runShooter).raceWith(new WaitCommand(0.2)),
            runLeftIntake.alongWith(fiveBallTres).raceWith(new WaitCommand(0.6)),
            runLeftIntake.raceWith(fiveBallQuatro),
            feedShooter.alongWith(runShooter).raceWith(new WaitCommand(0.2))
        );
    }
    
}
