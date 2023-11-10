package frc.lib.math;

public class Conversions {

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    //Things to check:  might need to multiply by 3 here due to the new gear ratio.  We shouldnt but just in case
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    //Things to check:  might need to multiply by 3 here due to the new gear ratio.  We shouldnt but just in case
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Velocity in m/s
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positioncounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Distance in meters
     */
    public static double falconToMeters(double positioncounts, double circumference, double gearRatio){
        double distance = (positioncounts/(gearRatio*2048.0))*circumference;
        return distance;
    }

    
    /**
     * @param distance Distance in meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Position Counts
     */
    public static double metersToFalcon(double distance, double circumference, double gearRatio){
        double counts = (distance/circumference)*(gearRatio*2048.0);
        return counts;
    }

    /**
     * Used with 775pro motor
     * @param counts SRX Mag Counts
     * @param gearRatio Gear Ratio between Mag Encoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    //Things to check:  might need to multiply by 3 here due to the new gear ratio.  We shouldnt but just in case
    public static double MagToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * Used with 775pro motor
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Mag Encoder and Mechanism
     * @return SRX Mag Counts
     */
    //Things to check:  might need to multiply by 3 here due to the new gear ratio.  We shouldnt but just in case
    public static double degreesToMag(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 4096.0));
        return ticks;
    }
}