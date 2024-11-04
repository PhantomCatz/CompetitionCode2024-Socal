package frc.robot.Autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class DashboardCmd extends Command{
    private String question;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    public DashboardCmd(String question, HashMap<String, Command> options){
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
    public void execute() {
        chooser.getSelected().execute();
    }

    @Override
    public void initialize(){
        chooser.getSelected().schedule();
    }

    @Override
    public boolean isFinished(){
        boolean tooEarly = chooser.getSelected().isFinished();
        System.out.println(tooEarly);
        return tooEarly;
    }
}
