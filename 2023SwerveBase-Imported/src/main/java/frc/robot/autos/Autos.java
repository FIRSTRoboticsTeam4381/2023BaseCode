package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {

    // TODO register commands in subsystem constructors using NamedCommands.registerCommand(String name, Command command)
    // TODO use AutoBuilder.buildAutoChooser()   to choose autos??

    // TODO test of a full auto
    public static Command testAuto(){
        return new PathPlannerAuto("TestAuto");
    }

    // TODO test of a single path
    public static Command testPath(){
        PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath1");

        return AutoBuilder.followPathWithEvents(path);
    }

    // TODO test of a path group
    public static Command testPathGroup(){
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("TestAuto");
        Command[] pathGroup = new Command[paths.size()];
        for(int i = 0; i < pathGroup.length; i++){
            pathGroup[i] = AutoBuilder.followPathWithEvents(paths.get(i));
        }

        return new SequentialCommandGroup(pathGroup);
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static Command none(){
        return Commands.none();
    }

}
