package frc.robot.Autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class ModifiableCmd extends Command{
    private String question;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    public ModifiableCmd(String question, HashMap<String, Command> options){
        options.forEach((k,v) -> {
            chooser.addOption(k,v);
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
        chooser.getSelected().schedule();
    }

    @Override
    public boolean isFinished(){
        return chooser.getSelected().isFinished();
    }
}
