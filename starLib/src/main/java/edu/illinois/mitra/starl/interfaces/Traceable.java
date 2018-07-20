package edu.illinois.mitra.starl.interfaces;

import java.util.HashMap;

/**
 * Traceable objects are capable of providing a HashMap (String -&gt; Object) of important attributes to trace. HashMap keys represent tag names, values represent tag values.
 * 
 * @author Adam Zimmerman
 * @version 1.0
 * 
 */
public interface Traceable {
	
	/**
	 * @return a HashMap (String -&gt; Object) representing all of the key/value pairs to be written to the trace file in XML form.
	 * For example, putting("color", "orange") will write "&lt;color&gt; orange &lt;/color&gt;" in the XML file.
	 */
	public HashMap<String,Object> getXML();
}
