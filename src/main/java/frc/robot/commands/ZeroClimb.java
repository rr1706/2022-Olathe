package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimb extends CommandBase {
    private final Climber m_climber;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;

    public ZeroClimb(Climber climber){
        m_climber = climber;
        addRequirements(m_climber);
    }
    @Override
    public void initialize(){
        m_climber.setPower(-0.10);
        m_finished = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        double time = m_timer.get();
        if(m_climber.getLimit() && time>0.040){
            m_climber.stop();
            m_climber.setPoseRef(0.0);
            m_climber.setDesiredPose(1.0);
            m_finished = true;
                }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setDesiredPose(5.0);      
        m_climber.stop();
        m_timer.stop();
    }
    @Override
    public boolean isFinished(){
        return m_finished;
    }
}
