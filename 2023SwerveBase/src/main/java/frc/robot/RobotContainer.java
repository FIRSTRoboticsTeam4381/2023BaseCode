// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.SwerveAuto;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  /* Driver Buttons */
  private final Trigger zeroSwerve = controller.options();

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /* Autonomouses */
  private final TestAuto testAuto = new TestAuto(s_Swerve);
  private final TemplateAuto templateAuto = new TemplateAuto(s_Swerve);

  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  //Change this and see what happens. Like auto for teleop.
  //private boolean openLoop = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, controller, true));

    m_AutoChooser.setDefaultOption("Test Auto", testAuto);
    m_AutoChooser.addOption("Template Auto", templateAuto);

    
    // Configure the button bindings
    configureButtonBindings();
    
    //Define event list for autonomous
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("action1", new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setInteger(3)));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      s_Swerve::getPose, // Pose2d supplier
      s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );


    List<PathPlannerTrajectory> pathPlanTestPaths = PathPlanner.loadPathGroup("PathPlannerTest", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    Command testPathPlannerAuto = autoBuilder.fullAuto(pathPlanTestPaths);

    m_AutoChooser.addOption("PathPlanner Test", testPathPlannerAuto);
    
    SmartDashboard.putData(m_AutoChooser);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setInteger(0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //Button to reset swerve odometry and angle
    zeroSwerve
      .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0))
      .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));

      /**
       * Note to self:
       * Teleop Swerve is a default command, meaning anything scheduled that uses drive will take over
       * Hence we can have a driver handoff button that will schedule a swervecontroller command to run a trajectory to drive in to the april
       * tag or specified location based off of the april tag.  We might be doing vision without green lights!
       */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoChooser.getSelected();
  }
}
