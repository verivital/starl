package edu.illinois.mitra.starlSim.main;

import java.util.ArrayList;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.illinois.mitra.starlSim.data.CsvWriter;
import edu.illinois.mitra.starlSim.draw.RobotData;

/**
 * Global logger for ground truth --- basically the same as the DrawFrame
 * viewer, but logs to text file
 * 
 * @author tjohnson
 * 
 */
public class GlobalLogger {
	private FileWriter fileWriter;

    private CsvWriter writer = null;

	/**
	 * 
	 * @param dir
	 *            directory for the log file
	 * @param filename
	 *            name for the log
	 */
	public GlobalLogger(String dir, String filename) {
		File file = new File(dir + filename);
		if(!file.exists()) {
			try {
				file.createNewFile();
			} catch(IOException e) {
			}
		}

        try {
            writer = new CsvWriter("test5.csv", "N Robots",
                    "Execution duration", "Robot ID", "getX",
                    "getY", "Pose", "Local time");
        } catch (IOException e) {
            e.printStackTrace();
        }

		try {
			fileWriter = new FileWriter(file);
		} catch(IOException e) {
			e.printStackTrace();
		}
	}

	public void updateData(ArrayList<RobotData> data, long time) {
		if(fileWriter == null)
			return;

		try {
			fileWriter.write(Long.toString(time) + "\n");
			for(RobotData d : data) {
				fileWriter.write(d.getName() + "," + d.getX() + "," + d.getY() + "," + d.getDegrees() + "," + d.getTime() + "\n");

                // todo: get real number of robots from settings, etc.
                writer.commit(4, Long.toString(time), d.getName(), d.getX(), d.getY(), d.getDegrees(), d.getTime());
			}
			fileWriter.write("\n\n");
			fileWriter.flush();


		} catch(IOException e) {
			e.printStackTrace();
		} catch(NullPointerException e) {
			// Catch the NPE
		}
	}
}
