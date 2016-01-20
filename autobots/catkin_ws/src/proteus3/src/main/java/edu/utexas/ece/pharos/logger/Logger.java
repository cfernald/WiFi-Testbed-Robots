package edu.utexas.ece.pharos.logger;

/**
 * Logs messages.
 * 
 * @author Chien-Liang Fok
 */
public class Logger {

	private static FileLogger flogger = null;
	
	/**
	 * Generates a prefix that should be added to the 
	 * beginning of a log statement.  The prefix is the 
	 * name of the calling class and method.
	 * 
	 * @return the log prefix.
	 */
	private static String getPrefix() {
		// Add the name of the calling method to the beginning of the message.
		StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
		StackTraceElement caller = stackTraceElements[3];
		String callingClassName = caller.getClassName();
		String callingMethodName = caller.getMethodName();
		
		String result = callingClassName + ": " + callingMethodName + ": ";
		return result;
	}
	
	/**
	 * Sets the file logger that this logger uses.  
	 * 
	 * @param flogger The file logger to use.  This may be null.
	 */
	public static void setFileLogger(FileLogger flogger) {
		Logger.flogger = flogger;
	}
	
	/**
	 * Logs a message to StdErr.
	 * The caller's class and method name is added to the front of the message.
	 * 
	 * @param msg The message to log.
	 */
	public static void logErr(String msg) {
		String result = getPrefix() + "ERROR: " + msg;
		System.err.println(result);
		if (Logger.flogger != null)
			Logger.flogger.log(result);
	}
	
	/**
	 * Logs a warning message to StdErr.
	 * The caller's class and method name is added to the front of the message.
	 * 
	 * @param msg The message to log.
	 */
	public static void logWarn(String msg) {
		String result = getPrefix() + "WARNING: " + msg;
		System.err.println(result);
		if (Logger.flogger != null)
			Logger.flogger.log(result);
	}
	
	/**
	 * Logs a message to StdOut.
	 * The caller's class and method name is added to the front of the message.
	 * 
	 * @param msg The message to log.
	 */
	public static void log(String msg) {
		String result = getPrefix() + msg;
		System.out.println(result);
		if (Logger.flogger != null)
			Logger.flogger.log(result);
	}
	
	/**
	 * Logs a message to StdOut if debug mode is enabled.
	 * Debug mode is enabled by the system property "PharosMiddleware.debug" being set
	 * to a value that is not null.
	 * The caller's class and method name is added to the front of the message.
	 * 
	 * @param msg The message to log.
	 */
	public static void logDbg(String msg) {
		String result = getPrefix() + msg;
		if (System.getProperty ("PharosMiddleware.debug") != null)
			System.out.println(result);
		if (Logger.flogger != null)
			Logger.flogger.log(result);
	}
}
