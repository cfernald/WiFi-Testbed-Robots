package edu.utexas.ece.pharos.proteus3.sensors;

/* ===========================================================
 * JFreeChart : a free chart library for the Java(tm) platform
 * ===========================================================
 *
 * (C) Copyright 2000-2004, by Object Refinery Limited and Contributors.
 *
 * Project Info:  http://www.jfree.org/jfreechart/index.html
 *
 * This library is free software; you can redistribute it and/or modify it under the terms
 * of the GNU Lesser General Public License as published by the Free Software Foundation;
 * either version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this
 * library; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 * [Java is a trademark or registered trademark of Sun Microsystems, Inc. 
 * in the United States and other countries.]
 *
 * -------------------
 * LineChartDemo6.java
 * -------------------
 * (C) Copyright 2004, by Object Refinery Limited and Contributors.
 *
 * Original Author:  David Gilbert (for Object Refinery Limited);
 * Contributor(s):   -;
 *
 * $Id: LineChartDemo6.java,v 1.5 2004/04/26 19:11:55 taqua Exp $
 *
 * Changes
 * -------
 * 27-Jan-2004 : Version 1 (DG);
 * 
 */



import java.awt.Color;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.data.Range;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;
//import org.jfree.ui.Spacer;
import org.ros.message.MessageListener;

//import edu.utexas.ece.pharos.logger.Logger;

import proteus3_compass.CompassMsg;

/**
 * A simple demonstration application showing how to create a line chart using data from an
 * {@link XYDataset}.
 *
 */
public class CompassChartGUI extends ApplicationFrame implements MessageListener<CompassMsg> {

	private static final long serialVersionUID = 4854956736415948890L;
	
	XYSeries dataSeries;
	long startTime = System.currentTimeMillis();
	
	
    /**
     * Creates a new demo.
     *
     * @param title  the frame title.
     */
    public CompassChartGUI(final String title) {

        super(title);

        final XYDataset dataset = createDataset();
        final JFreeChart chart = createChart(dataset);
        
        final XYPlot plot = chart.getXYPlot();
        ValueAxis axis = plot.getDomainAxis();
        axis.setAutoRange(true);
        axis.setFixedAutoRange(60);  // 60 seconds
        
        final ChartPanel chartPanel = new ChartPanel(chart);
        chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
        setContentPane(chartPanel);
        
//        new Thread(new ChartUpdater(chart.getXYPlot())).start();
    }
    
    @Override
	public void onNewMessage(CompassMsg message) {
		double time = (System.currentTimeMillis() - startTime) / 1000.0;
		dataSeries.add(time, message.getHeading());
	}
    
//    public void addData(double time, double value) {
//    	dataSeries.add(time, value);
//    }
    
//    private class ChartUpdater implements Runnable {
//    	XYPlot plot;
//    	
//    	public ChartUpdater(XYPlot plot) {
//    		this.plot = plot;	
//    	}
//    	
//    	public void run() {
//    		synchronized(this) {
//    			try {
//					this.wait(1000*4);
//				} catch (InterruptedException e) {
//					e.printStackTrace();
//				}
//    		}
//    		
//    		XYSeries series1 = new XYSeries("First");
//            series1.add(1.0, 8.0);
//            series1.add(2.0, 7.0);
//            series1.add(3.0, 6.0);
//            series1.add(4.0, 5.0);
//            series1.add(5.0, 4.0);
//            series1.add(6.0, 3.0);
//            series1.add(7.0, 2.0);
//            series1.add(8.0, 1.0);
//            XYSeriesCollection dataset = new XYSeriesCollection();
//            dataset.addSeries(series1);
            
//            plot.setDataset(dataset);
//    		series1.add(9.0,2.0);
//    		series1.add(50,1.4);
//    	}
//    }
    
    /**
     * Creates a sample dataset.
     * 
     * @return a sample dataset.
     */
    private XYDataset createDataset() {
        
    	dataSeries = new XYSeries("Compass Measurements");
//        series1.add(2.0, 1.0);
//        series1.add(3.0, 2.0);
//        series1.add(4.0, 3.0);
//        series1.add(5.0, 4.0);
//        series1.add(6.0, 5.0);
//        series1.add(7.0, 6.0);
//        series1.add(8.0, 7.0);
//        series1.add(9.0, 8.0);

        final XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(dataSeries);
                
        return dataset;
        
    }
    
    /**
     * Creates a chart.
     * 
     * @param dataset  the data for the chart.
     * 
     * @return a chart.
     */
    private JFreeChart createChart(final XYDataset dataset) {
        
        // create the chart...
        final JFreeChart chart = ChartFactory.createXYLineChart(
            "Proteus III Compass Measurements",      // chart title
            "Time (s)",                      // x axis label
            "Angle (degrees)",                      // y axis label
            dataset,                  // data
            PlotOrientation.VERTICAL,
            false,                     // include legend
            true,                     // tooltips
            false                     // urls
        );

        // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
        chart.setBackgroundPaint(Color.white);

//        final StandardLegend legend = (StandardLegend) chart.getLegend();
  //      legend.setDisplaySeriesShapes(true);
        
        // get a reference to the plot for further customization...
        final XYPlot plot = chart.getXYPlot();
        plot.setBackgroundPaint(Color.lightGray);
    //    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
        plot.setDomainGridlinePaint(Color.white);
        plot.setRangeGridlinePaint(Color.white);
        
        final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
        renderer.setSeriesLinesVisible(0, false);
        renderer.setSeriesShapesVisible(1, false);
        plot.setRenderer(renderer);
        
        final NumberAxis domainAxis = (NumberAxis)plot.getDomainAxis();
        domainAxis.setRange(new Range(0,140));

        
        final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
     // change the auto tick unit selection to integer units only...
//        rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
        //rangeAxis.setRange(new Range(-Math.PI, Math.PI));
        rangeAxis.setRange(new Range(-180, 180));
                
        return chart;
        
    }

    // ****************************************************************************
    // * JFREECHART DEVELOPER GUIDE                                               *
    // * The JFreeChart Developer Guide, written by David Gilbert, is available   *
    // * to purchase from Object Refinery Limited:                                *
    // *                                                                          *
    // * http://www.object-refinery.com/jfreechart/guide.html                     *
    // *                                                                          *
    // * Sales are used to provide funding for the JFreeChart project - please    * 
    // * support us so that we can continue developing free software.             *
    // ****************************************************************************
    
    /**
     * Starting point for the demonstration application.
     *
     * @param args  ignored.
     */
    public static void main(final String[] args) {
        final CompassChartGUI demo = new CompassChartGUI("Proteus III Compass Data vs. Time");
        demo.pack();
        RefineryUtilities.centerFrameOnScreen(demo);
        demo.setVisible(true);
    }

}