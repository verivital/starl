package edu.illinois.mitra.template;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;

import android.os.AsyncTask;
import android.util.Log;

public class IdentityLoader {

	public static String[][] loadIdentities(String url) {
		DownloadServerTask task = new DownloadServerTask();
		task.execute(url);
		String result = null;
		try {
			result = task.get();
		} catch(InterruptedException e) {
			e.printStackTrace();
		} catch(ExecutionException e) {
			e.printStackTrace();
		}
		
		if(result == null)
			return null;
		
		String[] lines = result.split("\r\n");
		
		List<String[]> identityRows = new ArrayList<String[]>();
		
		int linesize = -1;
		
		for(String line : lines) {
			if(line.startsWith("%") || line.trim().isEmpty())
				continue;
			
			String[] components = line.split("[\\s]*,[\\s]*");
			if(linesize < 0) {
				linesize = components.length;
			} else if(components.length != linesize) {
				Log.e("IdentityLoader", "Malformed entry: " + line);
				return null;
			}
			
			if(identityRows.size() < 3) {
				for(String c : components)
					c = c.trim();
				identityRows.add(components);
			}else {
				Log.e("IdentityLoader", "Too many arguments per ID!");
				return null;
			}
		}
		
		if(identityRows.isEmpty())
			return null;

		String[][] retval = {identityRows.get(0),identityRows.get(1),identityRows.get(2)};
		return retval;
	}

	/**
	 * Connect to a URL and attempt to download a file
	 * 
	 * @author azimmerman
	 */
	private static class DownloadServerTask extends AsyncTask<String, Integer, String> {

		@Override
		protected String doInBackground(String... params) {
			URL url = null;
			try {
				url = new URL(params[0]);
			} catch(MalformedURLException e1) {
				e1.printStackTrace();
				return null;
			}

			int retries = 0;
			while(retries < 5) {
				try {
					Log.d("Downloader", "Connecting...");
					HttpURLConnection conn = (HttpURLConnection) url.openConnection();
					
					int bytesRead = 0;
					BufferedInputStream in = new BufferedInputStream(conn.getInputStream());
					ByteArrayOutputStream out = new ByteArrayOutputStream();

					if(conn.getResponseCode() != HttpURLConnection.HTTP_NOT_FOUND) {
						int toRead = conn.getContentLength();
						byte[] data = new byte[1024];
						int read = 0;
						while((read = in.read(data, 0, 1024)) >= 0) {
							out.write(data, 0, read);
							bytesRead += read;
						}
						Log.d("Downloader", "Should have " + toRead + " bytes");
						Log.d("Downloader", "Wrote " + bytesRead + " bytes");
						if(bytesRead < toRead)
							Log.e("Downloader", "Download incomplete!");
						out.close();
						conn.disconnect();
						return out.toString();
					} else {
						Log.e("Downloader", "404'd!");
						retries++;
						conn.disconnect();
					}
				} catch(IOException e) {
					Log.e("Downloader", "IO Error!");
					retries++;
					e.printStackTrace();
				}
			}
			return null;
		}

	}
}
