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
    public void initialize(){
        isCommandSkipped = false;
        try {
            // forces all logic of the command to be located within this class
            // scheduling the command to run outside of this class will cause commands to finish too early
            chooser.getSelected().schedule();
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
