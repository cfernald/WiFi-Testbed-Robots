package edu.utexas.ece.pharos.exceptions;

/**
 * Thrown when no valid data is available.
 * 
 * @author Chien-Liang Fok
 */
public class NoValidDataException extends Exception {

	private static final long serialVersionUID = 5675324428626340950L;

	public NoValidDataException() {
		super();
	}
	
	public NoValidDataException(String msg) {
		super(msg);
	}
}
