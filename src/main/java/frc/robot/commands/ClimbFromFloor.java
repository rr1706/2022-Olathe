package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbFromFloor extends CommandBase {
    private final Climber m_climber;
    private boolean m_pastHooks = false;
    
    public ClimbFromFloor(Climber climber){
        m_climber = climber;
        //addRequirements(climber);
    }
    

    @Override
    public void initialize(){
        m_climber.setDesiredPose(0.0);
        m_climber.run();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Climbing from Floor", true);
        if(m_climber.atSetpoint()&&!m_pastHooks)
        {
            m_pastHooks = true;
            m_climber.setDesiredPose(6.0);
        }

        SmartDashboard.putNumber("Climb Encoder", m_climber.getPose());
    }

    @Override
    public void end(boolean interrupted){
        m_pastHooks = false;
    }

    public boolean finishedClimb(){
        return m_pastHooks;
    }

}
