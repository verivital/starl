package edu.illinois.mitra.starlSim.main;

//comment
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PositionList;

import java.io.*;
public final class WptLoader {
	
	private WptLoader() {
	}
	
	public static PositionList<ItemPosition> loadWaypoints(String file) {
		PositionList<ItemPosition> waypoints = new PositionList<ItemPosition>();
		BufferedReader in = null;
        InputStream inputStream =
                WptLoader.class.getClassLoader().getResourceAsStream(file);
        if(inputStream != null){
            in = new BufferedReader(new InputStreamReader(inputStream));
        }else{

            try {
            	File f = new File("waypoints" + File.separator + file);
                in = new BufferedReader(new FileReader(f));

            } catch (IOException e) {
                System.err.println("File " + file + " not found! No waypoints loaded.");
                return new PositionList<ItemPosition>();
            }
        }
		String line;
		try {
			while((line = in.readLine()) != null) {
				String[] parts = line.replace(" ", "").split(",");
				if(parts[0].equals("WAY") && parts.length == 6) {
					ItemPosition wpt = new ItemPosition(parts[4], Integer.parseInt(parts[1]), Integer.parseInt(parts[2]), Integer.parseInt(parts[3]), Integer.parseInt(parts[5]));
					waypoints.update(wpt);
				}
				if(parts[0].equals("WAY") && parts.length == 5) {
					ItemPosition wpt = new ItemPosition(parts[4], Integer.parseInt(parts[1]), Integer.parseInt(parts[2]), Integer.parseInt(parts[3]));
					waypoints.update(wpt);
				}
			}
			in.close();
		} catch (IOException e) {
			System.out.println("Error reading waypoints file!");
		}
		return waypoints;
	}
}
