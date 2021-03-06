package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
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
    private final boolean m_updatePose;
    private final Timer m_timer = new Timer();

    private static Point2D[] m_hoodPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,0.0),
            new Point2D.Double(50,0.0),
            new Point2D.Double(65,0),
            new Point2D.Double(80,8.83),
            new Point2D.Double(95,15.3),
            new Point2D.Double(110,20.0),
            new Point2D.Double(125,24.5),
            new Point2D.Double(140,27.5),
            new Point2D.Double(155,29.7),
            new Point2D.Double(170,32.7),
            new Point2D.Double(185,35.03),
            new Point2D.Double(200,38.0),
            new Point2D.Double(240,38.0)
        };
    private static LinearInterpolationTable m_hoodTable = new LinearInterpolationTable(m_hoodPoints);

    private static Point2D[] m_rpmPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,2350),
            new Point2D.Double(50,2350),
            new Point2D.Double(65,2364),
            new Point2D.Double(80,2397),
            new Point2D.Double(95,2418),
            new Point2D.Double(110,2500),
            new Point2D.Double(125,2568),
            new Point2D.Double(140,2636),
            new Point2D.Double(155,2756),
            new Point2D.Double(170,2910),
            new Point2D.Double(185,3080),
            new Point2D.Double(200,3250),
            new Point2D.Double(240,3570)
        };

    private static LinearInterpolationTable m_rpmTable = new LinearInterpolationTable(m_rpmPoints);

    

    public RunShooter(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood, boolean updatePose){
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        m_updatePose = updatePose;
        addRequirements(shooter, turret, hood);
    }

    @Override
    public void initialize(){
        m_turret.enable();
        m_turret.trackTarget(true);
        m_timer.reset();
        m_timer.start();
        SmartDashboard.putNumber("SetHoodAdjust", 0.0);
        SmartDashboard.putNumber("SetShotAdjust", 0);
        SmartDashboard.putBoolean("Adjust Shot?", false);
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Shooter Running", true);
        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d())*39.37;
        SmartDashboard.putNumber("Calculated (in)", dist);
        if(Limelight.valid()&& dist>ShooterConstants.kDistWithoutLimelight){
            dist = Limelight.getDistance();
            SmartDashboard.putNumber("Limelight (in)", dist);
            if(SmartDashboard.getBoolean("Adjust Shot?", false)){
                m_shooter.run(m_rpmTable.getOutput(dist)+SmartDashboard.getNumber("SetShotAdjust", 0));
                m_hood.run(m_hoodTable.getOutput(dist)+SmartDashboard.getNumber("SetHoodAdjust", 0));
            } 
            else{
                m_shooter.run(m_rpmTable.getOutput(dist));
                m_hood.run(m_hoodTable.getOutput(dist));
            }
        }
        else{

            if(SmartDashboard.getBoolean("Adjust Shot?", false)){
                m_shooter.run(m_rpmTable.getOutput(dist)+SmartDashboard.getNumber("SetShotAdjust", 0));
                m_hood.run(m_hoodTable.getOutput(dist)+SmartDashboard.getNumber("SetHoodAdjust", 0));
            } 
            else{
                m_shooter.run(m_rpmTable.getOutput(dist));
                m_hood.run(m_hoodTable.getOutput(dist));
            }

        }
        m_turret.setAngle(m_drive.getPose());
        double currentTime = m_timer.get();
        
        if(currentTime > 0.250 && Limelight.valid() && !robotMovingFast(m_drive.getChassisSpeed())){
            double dL = Limelight.getDistance()*0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getMeasurement()-Math.PI;
            double tL = -1.0*Limelight.tx();
    
            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);
    
            if(m_updatePose){
                m_drive.setPose(pose);
            }
    
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
      m_turret.trackTarget(false);
      m_turret.disable();
      m_turret.stop();
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
    
        if(speed > 1.000 || angleSpeed > 0.050){
            return true;
        }
        else{
            return false;
        }
    
    }

}
