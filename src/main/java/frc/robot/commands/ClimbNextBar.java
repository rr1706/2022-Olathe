package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbNextBar extends CommandBase{
    private final Climber m_climber;
    private boolean m_hasExtended = false;
    private boolean m_pastHooks = false;
    public ClimbNextBar(Climber climber){
        m_climber = climber;

    }

    @Override
    public void initialize(){
        m_climber.setDesiredPose(85);
        m_climber.run();
        m_pastHooks = false;
        m_hasExtended = false;
    }
    @Override
    public void execute(){
    SmartDashboard.putBoolean("CLimb at Setpoint", m_climber.atSetpoint());

        if(m_climber.getPose()>=20 && !m_hasExtended){
            m_climber.retract();
        }
        if(m_climber.atSetpoint()&& !m_hasExtended){
                m_climber.extend();
                m_climber.setDesiredPose(-3.0);
                m_hasExtended = true;
        }
        else if(m_climber.atSetpoint() && m_hasExtended && !m_pastHooks){
            m_pastHooks = true;
            m_climber.setDesiredPose(6.0);
        }
    }

    @Override 
    public void end(boolean interrupted){
    }
    @Override
    public boolean isFinished(){
        return m_pastHooks;
    }
}
