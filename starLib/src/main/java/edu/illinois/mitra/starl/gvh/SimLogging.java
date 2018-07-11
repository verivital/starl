package edu.illinois.mitra.starl.gvh;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;

public class SimLogging extends Logging {

	private StringBuilder simlog;
	private String name;
	private GlobalVarHolder gvh;
	
	public SimLogging(String name, GlobalVarHolder gvh) {
		this.name = name;
		this.gvh = gvh;
		simlog = new StringBuilder();
	}
	
	@Override
	public void e(String tag, String msg) {
		simlog.append(name).append("\t").append(gvh.time()).append("\te\t").append(tag)
				.append(" : ").append(msg).append("\n");
	}

	@Override
	public void i(String tag, String msg) {
		simlog.append(name).append("\t").append(gvh.time()).append("\ti\t").append(tag)
				.append(" : ").append(msg).append("\n");
	}

	@Override
	public void d(String tag, String msg) {
		simlog.append(name).append("\t").append(gvh.time()).append("\td\t").append(tag)
				.append(" : ").append(msg).append("\n");
	}

	@Override
	public String getLog() {
		return simlog.toString();
	}
	
	@Override
	public boolean saveLogFile(){
		File tmp = new File("logs/" + name);
		tmp.getParentFile().mkdirs();
		
		try {
			tmp.createNewFile();
			PrintWriter out = new PrintWriter("logs/" + name);
			out.write(getLog());
			out.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return false;
		}
		return true;
	}
}
