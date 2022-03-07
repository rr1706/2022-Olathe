package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class FeedShooter extends CommandBase{
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final Timer m_timer = new Timer();

    public FeedShooter(Turret turret, Shooter shooter, ShooterHood shooterHood, Elevator top, Elevator bottom){
        m_turret = turret;
        m_shooter = shooter;
        m_shooterHood = shooterHood;
        m_top = top;
        m_bottom = bottom;
        addRequirements(top,bottom);
    }

    @Override
    public void initialize(){
        m_top.run();
        m_bottom.run();
    }

    @Override
    public void execute(){
        
        boolean canShoot = m_shooter.atSetpoint()&&m_turret.atSetpoint()&&m_shooterHood.atSetpoint();
            if(canShoot){
                m_top.run();
                m_bottom.run();
            }
            else{
                m_top.stop();
                m_bottom.stop();
            }
            
            
            SmartDashboard.putBoolean("Shooting", true);
        
        
    }
    @Override
    public void end(boolean interrupted){
        m_top.stop();
        SmartDashboard.putBoolean("Shooting", false);
    }
}
