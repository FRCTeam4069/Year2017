
/*
package org.usfirst.frc.team4069.robot;
import java.awt.BorderLayout;
import java.awt.Container;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;

public class Window extends JFrame{
	
	public static final int WINDOW_WIDTH = 1200, WINDOW_HEIGHT = 900;
	
	private ImageIcon displayIcon;
	
	public Window(){
		
		super("OpenCV Vision Example");
		
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		this.setSize(WINDOW_WIDTH, WINDOW_HEIGHT);
		
		Container contentPane = this.getContentPane();
		
		displayIcon = new ImageIcon();
		
		JLabel display = new JLabel(displayIcon);
		
		contentPane.add(display);
		
		this.setVisible(true);
		
	}
	
	public ImageIcon getDisplayIcon(){
		
		return displayIcon;
		
	}
	
	public static void main(String[] args){
		
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		new VisionExample(new Window());
		
	}
	
}
*/