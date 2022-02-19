package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Timer m_timer = new Timer();
    private boolean m_reachedSetpoint = false;
    private double m_startUpTime = Double.POSITIVE_INFINITY;
    public RunShooter(Shooter shooter){
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        m_shooter.run();
        if(m_shooter.atSetpoint() && !m_reachedSetpoint){
            m_startUpTime = m_timer.get();
            m_reachedSetpoint = true;
            m_timer.stop();
        }
        SmartDashboard.putNumber("Start Up Time", m_startUpTime);
    }

    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      m_timer.stop();
      m_reachedSetpoint = false;
      m_startUpTime = Double.POSITIVE_INFINITY;
    }
    
}
