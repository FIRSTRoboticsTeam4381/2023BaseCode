package frc.lib.util;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class SwerveAuto{
    private String fileTree = "paths/output/";
    private String fileExtension = ".wpilib.json";

    private ProfiledPIDController thetaController;

    private Swerve s_Swerve;

    /*
     * TrajectoryConfig config = new TrajectoryConfig(
     *          Constants.AutoConstants.kMaxSpeedMetersPerSecond,
     *          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
     *                  .setKinematics(Constants.Swerve.swerveKinematics);
     * 
     * Not needed with pathweaver, constraints are set in project
     */

    /**
     * Class to be used to create autonomous routines using Pathweaver and SwerveDrive Odometry
     * @param autoName Name of Autonomous in path file names. eg. for "test1.wpilib.json" autoName is "test"
     * @param s_Swerve SwerveSubsystem
     */
    public SwerveAuto(String autoName, Swerve s_Swerve){
        this.s_Swerve = s_Swerve;

        fileTree += autoName;
        
        thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * @param num Number of leg in autonomous
     * @return Trajectory of respective path
     */
    private Trajectory generateTrajectory(int num){
        String trajectoryJSON = fileTree + num + fileExtension;
        Trajectory trajectory = new Trajectory();

        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }catch(IOException ex){
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        return trajectory;
    }

    /**
     * @param num Number of leg in autonomous
     * @return SwerveControllerCommand for respective leg in autonomous
     */
    private SwerveControllerCommand getLeg(int num){
        return new SwerveControllerCommand(
            generateTrajectory(num),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
    }

    /**
     * Command to drive the leg of autonomous.
     * <ol>
     * <li>Reset Odometry</li>
     * <li>Drive Leg</li>
     * <li>Stops robot</li>
     * </ol>
     * @param num Number of leg in command
     * @return Command composition of three commands
     */
    public Command driveLeg(int num){
        return 
            new InstantCommand(() -> s_Swerve.resetOdometry(generateTrajectory(num).getInitialPose()))
            .andThen(getLeg(num))
            .andThen( new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, false))
            );

    }

}
