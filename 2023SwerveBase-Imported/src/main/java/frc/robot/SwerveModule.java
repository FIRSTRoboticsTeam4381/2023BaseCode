package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.Faults;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

public class SwerveModule {
    public int moduleNumber;
    public double angleOffset;
    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private SparkMaxAnalogSensor analogSensor;

    private RelativeEncoder distanceEncoder;

    //private CANCoder angleEncoder;
    private double lastAngle;

    private double desiredAngle;
    private double lastSpeed;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        //angleEncoder = new CANCoder(moduleConstants.cancoderID, "DriveTrain");
        //configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushed);
        //configAngleMotor();
        
        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushed);
        //configDriveMotor();
        
        analogSensor = mAngleMotor.getAnalog(Mode.kAbsolute);
        // Set to degrees
        // TODO if default is 1V/rev, might need to include sensor's voltage range in this
        analogSensor.setPositionConversionFactor(360);

        mAngleMotor.getPIDController().setFeedbackDevice(analogSensor);


        distanceEncoder = mDriveMotor.getEncoder(Type.kHallSensor, Constants.NEO_TICKS_PER_REV);
        // Set to m/s for speed and m for distance
        distanceEncoder.setPositionConversionFactor(Constants.Swerve.wheelDiameter / Constants.Swerve.driveGearRatio);
        distanceEncoder.setVelocityConversionFactor(Constants.Swerve.wheelDiameter / Constants.Swerve.driveGearRatio / 60.0);

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
           // mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            //mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            mDriveMotor.getPIDController().setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)); 
        mAngleMotor.getPIDController().setReference(angle, ControlType.kPosition);
        desiredAngle = angle;
        lastAngle = angle;
    }

    /*public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        DriverStation.reportError("Module "+moduleNumber+" init error? "+mAngleMotor.setSelectedSensorPosition((int)absolutePosition).toString(), false);
        DriverStation.reportError("Module "+moduleNumber+" angle initialized to "+absolutePosition, false);
        DriverStation.reportError("Module "+moduleNumber+" read angle is "+getCanCoder().getDegrees(), false);
    }*/

    /**
     * Set settings for this motor controller and save them to its flash memory.
     * 
     * This is only inteded to be done when hardware is replaced or settings changed,
     * NOT on each boot! This prevents failed configuration or carryover from previous code.
     */
    public void configToFlash()
    {
        try
        {
            // Retain the absolute encoder/potentiometer's offset
            // double zeroOffset = mAngleMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle).getZeroOffset();
        
            // Drive motor
            LogOrDash.checkRevError("drive motor "+moduleNumber+" clear",
                mDriveMotor.restoreFactoryDefaults());
            
            mDriveMotor.wait(1000);

            SparkMaxPIDController pid = mDriveMotor.getPIDController();

            LogOrDash.checkRevError("drive motor "+moduleNumber+" kp",
                pid.setP(Constants.Swerve.driveKP));
            LogOrDash.checkRevError("drive motor "+moduleNumber+" ki",
                pid.setI(Constants.Swerve.driveKI));
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kd",
                pid.setD(Constants.Swerve.driveKD));
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kf",
                pid.setFF(Constants.Swerve.driveKF));
                
            LogOrDash.checkRevError("drive motor "+moduleNumber+" open loop ramp",
                mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp));

            LogOrDash.checkRevError("drive motor "+moduleNumber+" closed loop ramp",
                mDriveMotor.setOpenLoopRampRate(Constants.Swerve.closedLoopRamp));
            
            LogOrDash.checkRevError("drive motor "+moduleNumber+" current",
                mDriveMotor.setSmartCurrentLimit(Constants.Swerve.drivePeakCurrentLimit, Constants.Swerve.driveContinuousCurrentLimit));

            LogOrDash.checkRevError("drive motor "+moduleNumber+" idle mode",
                mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode));

            // This doesn't return a RevLibError apparently
            mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);

            mDriveMotor.wait(1000);
            LogOrDash.checkRevError("drive motor "+moduleNumber+" BURN",
                mDriveMotor.burnFlash());
            mDriveMotor.wait(1000);




            // Anale motor
            LogOrDash.checkRevError("angle motor "+moduleNumber+" clear",
                mAngleMotor.restoreFactoryDefaults());
            
            mAngleMotor.wait(1000);

            pid = mAngleMotor.getPIDController();

            LogOrDash.checkRevError("angle motor "+moduleNumber+" kp",
                pid.setP(Constants.Swerve.angleKP));
            LogOrDash.checkRevError("angle motor "+moduleNumber+" ki",
                pid.setI(Constants.Swerve.angleKI));
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kd",
                pid.setD(Constants.Swerve.angleKD));
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kf",
                pid.setFF(Constants.Swerve.angleKF));
            
            LogOrDash.checkRevError("angle motor "+moduleNumber+" current",
                mAngleMotor.setSmartCurrentLimit(Constants.Swerve.anglePeakCurrentLimit, Constants.Swerve.angleContinuousCurrentLimit));

            LogOrDash.checkRevError("angle motor "+moduleNumber+" idle mode",
                mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode));

            // This doesn't return a RevLibError apparently
            mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);

            mAngleMotor.wait(1000);
            LogOrDash.checkRevError("angle motor "+moduleNumber+" BURN",
                mAngleMotor.burnFlash());
            mAngleMotor.wait(1000);

        }
        catch(InterruptedException e)
        {
            DriverStation.reportError("Main thread interrupted while flashing swerve module!", e.getStackTrace());
        }
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(analogSensor.getPosition());
    }

    public Double getTemp(int motor){
        return (motor == 1)?mDriveMotor.getMotorTemperature():mAngleMotor.getMotorTemperature();
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }

    public double getDesiredSpeed(){
        return lastSpeed;
    }

    public SwerveModuleState getState(){
        //double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        double velocity = distanceEncoder.getVelocity(); //Units configured to m/s
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getPosition(){
        double distance = distanceEncoder.getPosition(); //Units configured to m
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(distance, angle);
    }


    public void sendTelemetry() {
        //LogOrDash.logNumber("swerve/m" + moduleNumber + "/cancoder", getAngle().getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/position", getState().angle.getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getState().speedMetersPerSecond);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getPosition().distanceMeters);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/setpoint", desiredAngle);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/setpoint", lastSpeed);
        
        LogOrDash.sparkMaxDiagnostics("swerve/m" + moduleNumber + "/angle", mAngleMotor);
        LogOrDash.sparkMaxDiagnostics("swerve/m" + moduleNumber + "/drive", mDriveMotor);
        
    }
}