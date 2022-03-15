package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Extend extends CommandBase{
    private final Climber m_climber;
    private boolean m_hasExtended = false;

    public Extend(Climber climber){
        m_climber = climber;
    }

    @Override
    public void initialize(){
        m_climber.setDesiredPose(82.0);
        m_hasExtended = false;
    }
    @Override
    public void execute(){
    //SmartDashboard.putBoolean("Climb at Setpoint", m_climber.atSetpoint());
    double pose = m_climber.getPose();
    //SmartDashboard.putNumber("Extend Pose", pose);
        if(pose>=20 && !m_hasExtended){
            m_climber.retract();
        }
        if(m_climber.atSetpoint()&& !m_hasExtended){
                m_climber.extend();
                m_hasExtended = true;

        }
    }

    @Override 
    public void end(boolean interrupted){
       // SmartDashboard.putBoolean("Climb at Setpoint", m_climber.atSetpoint());
        //SmartDashboard.putBoolean("IF 1", false);
        //SmartDashboard.putBoolean("IF 2", false);

    }
    @Override
    public boolean isFinished(){
        return m_hasExtended;
    }
}