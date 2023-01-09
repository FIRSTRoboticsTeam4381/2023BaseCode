package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonSRXConfiguration swerveAngleSRXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    public CTREConfigs(){
        swerveAngleSRXConfig = new TalonSRXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        swerveAngleSRXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleSRXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleSRXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleSRXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleSRXConfig.peakCurrentLimit = Constants.Swerve.anglePeakCurrentLimit;
        swerveAngleSRXConfig.peakCurrentDuration = Constants.Swerve.anglePeakCurrentDuration;
        swerveAngleSRXConfig.continuousCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
        //Things to check:  Lost this functionality moving to TalonSRX, hopefully not an issue
        //swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}