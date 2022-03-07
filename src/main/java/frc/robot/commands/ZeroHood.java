package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class ZeroHood extends CommandBase {
    private ShooterHood m_hood;
    private Timer m_timer = new Timer();

    public ZeroHood(ShooterHood hood){
        m_hood = hood;
        addRequirements(m_hood);
    }
    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_hood.setHood(-0.10);
    }

    @Override
    public void execute(){
        double time = m_timer.get();
        double current = m_hood.getTotalCurrent();
        SmartDashboard.putNumber("Hood Current", current);
        if(current>19.5 && time >0.040){
            SmartDashboard.putBoolean("Canceling", true);
            m_hood.setHoodZero();
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
