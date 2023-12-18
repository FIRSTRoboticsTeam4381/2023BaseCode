package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.util.DriftCorrection;
import frc.robot.Constants;
import frc.robot.LogOrDash;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "DriveTrain");
        gyro.setYaw(0);
        zeroGyro(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

        // TODO check - auto
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.Swerve.holonomicConfig,
            this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Function used to actually drive the robot
     * @param translation XY drive values
     * @param rotation Rotation value
     * @param fieldRelative True -> fieldOriented
     * @param isOpenLoop True
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(DriftCorrection.driftCorrection(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation), 
                                swerveOdometry.getPoseMeters())
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // TODO check - auto
    public void drive(ChassisSpeeds robotRelativeSpeeds)
    {
        Translation2d translation = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vxMetersPerSecond);
        double rotation = robotRelativeSpeeds.omegaRadiansPerSecond;
        drive(translation, rotation, false, false);

    }

    // TODO check - auto
    /* Used by Pathplanner AutoBuilder */
    private ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /**
     * @return XY of robot on field
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d yaw) {
        swerveOdometry.resetPosition(yaw, getPositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return Swerve Module positions
     */
    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Use to reset angle to certain known angle or to zero
     * @param angle Desired new angle
     */
    public void zeroGyro(double angle){
        gyro.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }
    

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getPositions());
        
        
        /*SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Temp", mod.getTemp(1));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Temp", mod.getTemp(2));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Setpoint", mod.getDesired());
        }

        SmartDashboard.putString("XY Coord", "(" + getPose().getX() + ", " + getPose().getY() + ")");*/

        LogOrDash.logNumber("Gyro Angle", getYaw().getDegrees());

        SwerveModuleState[] currentStatus = new SwerveModuleState[4];
        double[] targetSpeeds = new double[4];
        double[] targetAngles = new double[4];
        double[] absoluteAngles = new double[4];
        
        for(SwerveModule mod : mSwerveMods){
            mod.sendTelemetry();
            currentStatus[mod.moduleNumber] = mod.getState();
            targetSpeeds[mod.moduleNumber] = mod.getDesiredSpeed();
            targetAngles[mod.moduleNumber] = mod.getDesiredAngle();
            absoluteAngles[mod.moduleNumber] = mod.getAngle().getDegrees() - mod.angleOffset;
        }

        // Compile swerve status for AdvantageScope
        double[] targetStateAdv = new double[8];
        double[] currentStateAdv = new double[8];
        double[] absoluteStateAdv = new double[8];
        for(int i=0; i<4;i++)
        {
            targetStateAdv[2*i] = targetAngles[i];
            targetStateAdv[2*i+1] = targetSpeeds[i];
            
            currentStateAdv[2*i] = currentStatus[i].angle.getDegrees();
            currentStateAdv[2*i+1] = currentStatus[i].speedMetersPerSecond;

            absoluteStateAdv[2*i] = absoluteAngles[i];
            absoluteStateAdv[2*i+1] = 8;//Arbitrary to make these easier to see
        }

        SmartDashboard.putNumberArray("swerve/status", currentStateAdv);
        SmartDashboard.putNumberArray("swerve/target", targetStateAdv);
        SmartDashboard.putNumberArray("swerve/absolute", absoluteStateAdv);


        LogOrDash.logNumber("Gyro Pitch", gyro.getPitch());
        LogOrDash.logNumber("Gyro Roll", gyro.getRoll());
        LogOrDash.logNumber("Gyro Yaw", gyro.getYaw());
        LogOrDash.logString("XY Coord", "(" + getPose().getX() + ", " + getPose().getY() + ")");

    }

    /*public void resetWheelAngles()
    {
        for(SwerveModule mod : mSwerveMods)
        {
            mod.resetToAbsolute();
        }
    }*/


    public Command configToFlash()
    {
        return new InstantCommand(() -> {
           for(SwerveModule mod : mSwerveMods)
            {
                mod.configToFlash();
            } 
        }, this);
    }
}