package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimb extends CommandBase {
    private Climber m_climber;


    public ZeroClimb(Climber climber){
        m_climber = climber;
        addRequirements(m_climber);
    }
    @Override
    public void initialize(){
        m_climber.setPower(-0.10);
    }

    @Override
    public void execute(){
        if(m_climber.getCurrent()>60.0){
            m_climber.setPoseRef(0.0);
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setDesiredPose(0.0);      
        m_climber.stop();
    }
}
