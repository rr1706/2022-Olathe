package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbFromFloor extends CommandBase {
    private final Climber m_climber;
    
    public ClimbFromFloor(Climber climber){
        m_climber = climber;
        
    }
    

    @Override
    public void initialize(){
        m_climber.setDesiredPose(0.0);
        m_climber.run();
    }

    @Override
    public void execute(){
        if(m_climber.atSetpoint())
        {
            m_climber.stop();
        }
    }

}
