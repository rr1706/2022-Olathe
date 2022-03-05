package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class ZeroHood extends CommandBase {
    private ShooterHood m_hood;


    public ZeroHood(ShooterHood hood){
        m_hood = hood;
        addRequirements(m_hood);
    }
    @Override
    public void initialize(){
        m_hood.setHood(-0.05);
    }

    @Override
    public void execute(){
        if(m_hood.getTotalCurrent()>15.0){
            m_hood.setHoodZero();
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
      m_hood.setHoodAngle(0);
      m_hood.stop();
    }
}
