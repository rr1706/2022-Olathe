package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class RunShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Drivetrain m_drive;
    private final ShooterHood m_hood;
    private final Timer m_timer = new Timer();

    public RunShooter(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood){
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize(){
        m_turret.enable();
        m_turret.trackTarget(true);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        m_shooter.run();
        m_hood.run();
        m_turret.setAngle(m_drive.getPose());
        double currentTime = m_timer.get();
        
        SmartDashboard.putNumber("In loop", currentTime);
        SmartDashboard.putNumber("Distance", Limelight.getDistance());

        if(currentTime > 0.250 && Limelight.valid() && !robotMovingFast(m_drive.getChassisSpeed())){
            double dL = Limelight.getDistance()*0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getMeasurement()-Math.PI;
            double tL = -1.0*Limelight.tx();
    
            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);
    
            SmartDashboard.putNumber("Calc Xin", pose.getX()*39.37);
            SmartDashboard.putNumber("Calc Yin", pose.getY()*39.37);
    
              m_drive.setPose(pose);
    
        }
    }

    @Override
    public void end(boolean interrupted) {
      m_turret.trackTarget(false);
      m_shooter.stop();
      m_hood.stop();
      m_timer.stop();
    }
    
    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal){
        double tG = tR+tT+tL;
        double rX = goal.getX()-dL*Math.cos(tG);
        double rY = goal.getY()-dL*Math.sin(tG);
    
        return new Pose2d(rX,rY, new Rotation2d(-tR));
    }
    
    private boolean robotMovingFast(ChassisSpeeds input){
        double speed = Math.sqrt(Math.pow(input.vxMetersPerSecond,2)+Math.pow(input.vyMetersPerSecond, 2));
        double angleSpeed = Math.abs(input.omegaRadiansPerSecond);
    
        if(speed > 0.500 || angleSpeed > 0.020){
            return true;
        }
        else{
            return false;
        }
    
    }

}
