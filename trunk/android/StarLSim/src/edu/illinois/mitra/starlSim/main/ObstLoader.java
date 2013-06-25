package edu.illinois.mitra.starlSim.main;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
//comment
import edu.illinois.mitra.starl.objects.*;


public class ObstLoader {

	private ObstLoader() {
	}
	public static ObstacleList loadObspoints(String file) {
		ObstacleList Obspoints = new ObstacleList();
		BufferedReader in = null;
		try {
			in = new BufferedReader(new FileReader(file));
		} catch (FileNotFoundException e) {
			System.err.println("File " + file + " not found! No Obspoints loaded.");
			return new ObstacleList();
		}
		
		String line;
		try {
			while((line = in.readLine()) != null) {
				String[] parts = line.replace(" ", "").replace(";", ",").split(",");
					Obstacles point = new Obstacles();
					if(parts[0].equals("Obstacle")) {
						for(int j = 1; j<((parts.length)-1); j+=2)
						
						point.add(Integer.parseInt(parts[j]),Integer.parseInt(parts[j+1]));
						
					}
					Obspoints.ObList.add(point);
			}
			in.close();
		} catch (IOException e) {
			System.out.println("Error reading Obspoints file!");
		}
		return Obspoints;
	}
	
}