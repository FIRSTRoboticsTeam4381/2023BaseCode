package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriftCorrection {
    
    /**
     * Drift correction PID used to correct our rotation in teleop
     * @param speeds Chassis speeds object from drive subsystem
     * @param pose Current pose from odometry
     * @return Updated Chassis Speeds
     */
    public static ChassisSpeeds driftCorrection(ChassisSpeeds speeds, Pose2d pose) {
        PIDController driftCorrectionPID = new PIDController(0.07, 0.00, 0.004);
        double desiredHeading= 0;
        double pXY = 0;
        ChassisSpeeds newSpeeds = speeds;

        double xy = Math.abs(newSpeeds.vxMetersPerSecond) + Math.abs(newSpeeds.vyMetersPerSecond);

        if (Math.abs(newSpeeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0)
            desiredHeading = pose.getRotation().getDegrees();
        else if (xy > 0)
            newSpeeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(pose.getRotation().getDegrees(),
                    desiredHeading);
        
        pXY = xy;

        return newSpeeds;
    }

}
