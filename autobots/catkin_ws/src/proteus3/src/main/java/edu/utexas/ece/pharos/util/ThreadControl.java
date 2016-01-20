package edu.utexas.ece.pharos.util;

/**
 * Provides useful methods for controlling the 
 * execution of threads.
 * 
 * @author Chien-Liang Fok
 */
public class ThreadControl {
	
	public static void pause(Object lock, long duration) {
		try {
			synchronized(lock) {
				lock.wait(duration);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
