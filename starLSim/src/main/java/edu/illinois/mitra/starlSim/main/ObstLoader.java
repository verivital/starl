package edu.illinois.mitra.starlSim.main;
import java.io.*;
//comment
import edu.illinois.mitra.starl.objects.*;


public class ObstLoader {

	private ObstLoader() {
	}
	public static ObstacleList loadObspoints(String file) {
		ObstacleList Obspoints = new ObstacleList();
		BufferedReader in = null;
        InputStream inputStream =
                WptLoader.class.getClassLoader().getResourceAsStream(file);
        if(inputStream != null){
            in = new BufferedReader(new InputStreamReader(inputStream));
        }else{

            try {
                in = new BufferedReader(new FileReader(file));
            } catch (FileNotFoundException e) {
                System.err.println("File " + file + " not found! No waypoints loaded.");
                return new ObstacleList();
            }
        }
		
		String line;
		try {
			while((line = in.readLine()) != null) {
				String[] parts = line.replace(" ", "").replace(";", ",").split(",");
					Obstacles point = new Obstacles();
					if(parts[0].equals("Obstacle")) {
						int j;
						for(j = 1; j<((parts.length)-2); j+=2)
						{
							point.add(Integer.parseInt(parts[j]),Integer.parseInt(parts[j+1]));
						}
						point.timeFrame = Integer.parseInt(parts[j]);
						point.hidden = false;
						Obspoints.ObList.add(point);
					}
					if(parts[0].equals("Hidden")) {
						int j;
						for(j = 1; j<((parts.length)-2); j+=2)
						{
							point.add(Integer.parseInt(parts[j]),Integer.parseInt(parts[j+1]));
						}
						point.timeFrame = Integer.parseInt(parts[j]);
						point.hidden = true;
						Obspoints.ObList.add(point);
					}
					
			}
			in.close();
		} catch (IOException e) {
			System.out.println("Error reading Obspoints file!");
		}
		return Obspoints;
	}
	
}
