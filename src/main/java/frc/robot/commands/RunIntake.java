package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private final Intake m_intake;
    private final Timer m_timer = new Timer();
    private double m_ballTime = 0.00;
    private boolean m_ballDetected = false;
    public RunIntake(Intake intake){
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_intake.extend();
    }

    @Override
    public void execute(){
        m_intake.run();
        final double currentTime = m_timer.get();
        if(m_intake.getCurrent()>15.0 && currentTime>0.250 && !m_ballDetected)
        {
            m_ballTime = currentTime;
            m_ballDetected = true;
        }
        if(m_ballDetected && (currentTime-m_ballTime)>0.25){
            //cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_timer.stop();
        m_ballDetected = false;
        m_ballTime = 0.0;
    }
}
