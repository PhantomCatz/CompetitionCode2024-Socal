package frc.robot.Autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class ModifiableCmd extends Command{
    private String question;
    private SendableChooser<Command> chooser = new SendableChooser<>();
    private Command selectedCommand;

    public ModifiableCmd(String question, HashMap<String, Command> options){
        options.forEach((k,v) -> {
            //This code will make it so that the last item is the default option.
            //I wrote this just in case the driver forgot to select an option.
            chooser.setDefaultOption(k,v);
        });
        
        this.question = question;
    }

    public SendableChooser<Command> getChooser(){
        return chooser;
    }
    public String getQuestion(){
        return question;
    }

    @Override
    public void initialize(){
        // selectedCommand = chooser.getSelected();
        chooser.getSelected().schedule();
    }

    // @Override
    // public void execute(){
    //     //the error has to do with how this modifiable command is being run inside auto periodic??
    //     // selectedCommand.execute();
    // }

    // @Override
    // public boolean isFinished(){
    //     return selectedCommand.isFinished();
    // }

    // @Override
    // public void end(boolean interrupted){
    //     selectedCommand.end(interrupted);
    // }
}
