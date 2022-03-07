package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.Utilities.LinearInterpolationTable;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;
import java.awt.geom.Point2D;

public class RunShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Drivetrain m_drive;
    private final ShooterHood m_hood;
    private final Timer m_timer = new Timer();

    private static Point2D[] m_hoodPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,5.0),
            new Point2D.Double(50,5.0),
            new Point2D.Double(65,5.75),
            new Point2D.Double(80,10.0),
            new Point2D.Double(95,22),
            new Point2D.Double(110,24),
            new Point2D.Double(125,27),
            new Point2D.Double(140,30),
            new Point2D.Double(155,33.5),
            new Point2D.Double(170,36.4),
            new Point2D.Double(185,38.0),
            new Point2D.Double(200,38.0),
            new Point2D.Double(240,38.0)
        };
    private static LinearInterpolationTable m_hoodTable = new LinearInterpolationTable(m_hoodPoints);

    private static Point2D[] m_rpmPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,2350),
            new Point2D.Double(50,2350),
            new Point2D.Double(65,2400),
            new Point2D.Double(80,2450),
            new Point2D.Double(95,2475),
            new Point2D.Double(110,2500),
            new Point2D.Double(125,2550),
            new Point2D.Double(140,2615),
            new Point2D.Double(155,2775),
            new Point2D.Double(170,2925),
            new Point2D.Double(185,3125),
            new Point2D.Double(200,3275),
            new Point2D.Double(240,3500)
        };

    private static LinearInterpolationTable m_rpmTable = new LinearInterpolationTable(m_rpmPoints);

    

    public RunShooter(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood){
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        addRequirements(shooter, turret, hood);
    }

    @Override
    public void initialize(){
        m_turret.enable();
        m_turret.trackTarget(true);
        m_timer.reset();
        m_timer.start();
        SmartDashboard.putNumber("SetHoodAngle", 10.0);
        SmartDashboard.putNumber("SetShotRPM", 2500);
        SmartDashboard.putBoolean("Override LIT", false);
    }

    @Override
    public void execute(){
        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d())*39.37;
        SmartDashboard.putNumber("distance In shooter?", dist);
        if(SmartDashboard.getBoolean("Override LIT", false)){
            m_shooter.run(SmartDashboard.getNumber("SetShotRPM", 2500));
            m_hood.run(SmartDashboard.getNumber("SetHoodAngle", 10.0));
        }
        else{
        m_shooter.run(m_rpmTable.getOutput(dist));
        m_hood.run(m_hoodTable.getOutput(dist));
        }
        m_turret.setAngle(m_drive.getPose());
        double currentTime = m_timer.get();
        
        //SmartDashboard.putNumber("In loop", currentTime);
        SmartDashboard.putNumber("Distance", Limelight.getDistance());

        if(currentTime > 0.250 && Limelight.valid() && !robotMovingFast(m_drive.getChassisSpeed())){
            double dL = Limelight.getDistance()*0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getMeasurement()-Math.PI;
            double tL = -1.0*Limelight.tx();
    
            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);
    
            //SmartDashboard.putNumber("Calc Xin", pose.getX()*39.37);
            //SmartDashboard.putNumber("Calc Yin", pose.getY()*39.37);
    
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
