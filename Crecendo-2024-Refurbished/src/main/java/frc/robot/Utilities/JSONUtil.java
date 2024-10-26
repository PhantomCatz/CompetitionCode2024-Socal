// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import java.util.ArrayList;

/** Add your docs here. */
public class JSONUtil {
    public static ArrayList<Object> getCommandsFromPath(JSONObject json){
        return JSONUtil.getCommandsFromCommandGroup((Object) json.get("command"));
    }

    /**
     * This code will recursively loop through an auto so it can catch NamedCommands nested within Sequential or Parallel commands.
     * @param json
     * @return List of named commands as a JSON object
     */
    public static ArrayList<Object> getCommandsFromCommandGroup(Object json){
        ArrayList<Object> commandsList = new ArrayList<>();

        JSONArray commands = (JSONArray)((JSONObject)((JSONObject)json).get("data")).get("commands");
        for(Object command: commands){
            String commandType = JSONUtil.getCommandType(command);
            if(commandType.equals("sequential") || commandType.equals("parallel")){
                commandsList.addAll(getCommandsFromCommandGroup(command));
            } else {
                commandsList.add(command);
            }
        }

        return commandsList;
    }

    public static String getCommandName(Object o){
        return (String)((JSONObject)((JSONObject) o).get("data")).get("name");
    }

    public static String getCommandType(Object o){
        return (String)((JSONObject) o).get("type");
    }
}