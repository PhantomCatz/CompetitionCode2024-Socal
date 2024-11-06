package frc.robot.Autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class DashboardCmd extends Command{
    private String question;
    private SendableChooser<Command> chooser = new SendableChooser<>();

    private boolean isCommandSkipped = false;

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
        try {
            chooser.getSelected().execute();
        } catch (Exception e) {

        }
    }

    @Override
    public void initialize(){
        isCommandSkipped = false;
        try {
            chooser.getSelected().initialize();
        } catch (Exception e) {
            System.out.println("Command Skipped due to uninitialization");
            isCommandSkipped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isCommandSkipped || chooser.getSelected().isFinished(); // boolean is evaluated first to prevent crashing
        
    }
}
