package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.io.File;
import java.nio.file.Files;
import java.util.List;

public class VersionData {

    /**
     * reads build data from file and writes to driverstation plus dashboard
     */
    public static void WriteBuildInfoToDashboard() {
        DriverStation.reportWarning("Build Version ID: " + getInfo("VERSION_ID"), false);
        DriverStation.reportWarning("Author: " + getInfo("BUILD_AUTHOR") + (Constants.ENABLE_MP_TEST_MODE ? "! MP TEST MODE IS ENABLED!" : ""), false);
        SmartDashboard.putString("Build_Info/ID", getInfo("VERSION_ID"));
        SmartDashboard.putString("Build_Info/Author", getInfo("BUILD_AUTHOR"));
        SmartDashboard.putString("Build_Info/DATE", getInfo("BUILD_DATE"));
    }


    /**
     * gets build info from version.dat file
     *
     * @param key data entry to parse for
     * @return data contained by key or empty string if not found
     */
    public static String getInfo(String key) {
        try {
            File version;
            if (RobotBase.isReal()) version = new File(Filesystem.getDeployDirectory(), "version.dat");
            else version = new File(Filesystem.getLaunchDirectory(), "src\\main\\deploy\\version.dat");
            List<String> lines = Files.readAllLines(version.toPath());
            int i = 0;
            int equalsInd = 0;
            for (String line : lines) {
                if (line.indexOf(key) > -1) {
                    i = line.indexOf(key) + key.length();
                    while (line.charAt(i) != '=') {
                        i++;
                    }
                    equalsInd = i;
                    while (line.charAt(i) != ';') {
                        i++;
                    }
                    return line.substring(equalsInd + 1, i);
                }
            }
            DriverStation.reportWarning("Failed to discover " + key + " in Version.dat", false);
            return "";
        } catch (Exception e) {
            DriverStation.reportError("Failed to read version.dat in deploy directory!", e.getStackTrace());
            return "";
        }
    }
}