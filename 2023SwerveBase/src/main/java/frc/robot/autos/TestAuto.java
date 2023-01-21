package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.lib.util.SwerveAuto;

public class TestAuto extends SequentialCommandGroup{

    private String autoName = "test";

    /**
     * Testing Autonomous, now using SwerveAuto helper
     * @param s_Swerve Swerve Subsystem
     */
    public TestAuto(Swerve s_Swerve){
        
        SwerveAuto drive = new SwerveAuto(autoName, s_Swerve);

        addCommands(
            drive.driveLeg(1)
        );

    }

}
