package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private CommandPS4Controller controller;

    private final Field2d m_field = new Field2d();
    private Pose2d startPose = new Pose2d(Units.inchesToMeters(177), Units.inchesToMeters(214), Rotation2d.fromDegrees(0));

    private final int limit = 5;
    private final SlewRateLimiter m_ForwardBackLimit = new SlewRateLimiter(limit);
    private final SlewRateLimiter m_SideSideLimit = new SlewRateLimiter(limit);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(limit);
    
    /**
     * Driver Control command
     * @param s_Swerve Swerve subsystem
     * @param controller PS4 controller
     * @param openLoop True
     */
    public TeleopSwerve(Swerve s_Swerve, CommandPS4Controller controller, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.openLoop = openLoop;


        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(startPose);        
    }

    @Override
    public void execute() {
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        /* Deadbands */
        
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;      
        
        /*
        rAxis *= 0.75;
        */

        yAxis = m_ForwardBackLimit.calculate(yAxis);
        xAxis = m_SideSideLimit.calculate(xAxis);
        rAxis = m_rotLimiter.calculate(rAxis);


        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, openLoop);

        m_field.setRobotPose(s_Swerve.getPose());

        if(controller.triangle().getAsBoolean()){
            s_Swerve.resetOdometry(s_Swerve.limePose());

            s_Swerve.zeroGyro(s_Swerve.limePose().getRotation().getDegrees() + ((DriverStation.getAlliance() == Alliance.Red)? 180:0));
        }

        s_Swerve.autoReset();
        
    }
}
