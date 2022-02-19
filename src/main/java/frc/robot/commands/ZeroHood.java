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
        m_hood.setHood(-0.040);
    }

    @Override
    public void execute(){
        if(m_hood.getHoodLimit()){
            m_hood.setHoodZero();
            m_hood.setHood(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
      m_hood.setHoodAngle(0);
      m_hood.setHood(0.0);
    }
}
