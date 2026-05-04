package com.mycompany.agvparking;

import java.awt.Color;
import java.awt.Graphics;
import javax.swing.JPanel;

/**
 *
 * @author KTS - Grupp 2
 */
public class MapPanel extends JPanel {

    DataStore ds;
    
    private java.awt.Image agvImage; 
    private java.awt.Image agvLoadedImage; 

    MapPanel(DataStore ds) {
        this.ds = ds; 
        
        try { 
            agvImage = javax.imageio.ImageIO.read(new java.io.File("agvBild.png"));
            agvLoadedImage = javax.imageio.ImageIO.read(new java.io.File("agvLoaded.png")); 
        } catch (Exception e) { 
            System.out.println("Hittade inte agv.png eller agvLoaded.png, använder triangel istället."); 
        } 
        
        // Din manuella initiering av hinder
//        ds.ObstacleMatrix[11][11] = 1;
//        ds.ObstacleMatrix[11][12] = 1;
//        ds.ObstacleMatrix[11][13] = 1;
//        ds.ObstacleMatrix[12][13] = 1;
//        ds.ObstacleMatrix[13][13] = 1;
//        ds.ObstacleMatrix[13][12] = 1;
//        ds.ObstacleMatrix[13][11] = 1;
//        ds.ObstacleMatrix[12][11] = 1;

        // Horisontella parkeringsrutor (Fixad loop)
        for (int k = 0; k < ds.Hspaces; k++) {
            int xpos = (int) (ds.HLocationX[k] / ds.gridsize);
            int ypos = (int) (ds.HLocationY[k] / ds.gridsize);
            for (int i = 0; i < (int) (120 / ds.gridsize); i = i + 1) {
                for (int j = 0; j < (int) (60 / ds.gridsize); j = j + 1) {
                    ds.ObstacleMatrix[xpos + i][ypos + j] = 1;
                }
            }
        }

        // Vertikala parkeringsrutor (Fixad loop)
        for (int k = 0; k < ds.Vspaces; k++) {
            int xpos = (int) (ds.VLocationX[k] / ds.gridsize);
            int ypos = (int) (ds.VLocationY[k] / ds.gridsize);
            for (int i = 0; i < (int) (60 / ds.gridsize); i = i + 1) {
                for (int j = 0; j <= (int) (120 / ds.gridsize); j = j + 1) {
                    ds.ObstacleMatrix[xpos + i][ypos + j] = 1;
                }
            }
        }
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Innan vi gör något annat, fyll hela panelen med golvfärgen.
        // Då blir "gliporna" osynliga eftersom det är grått under allt.
        g.setColor(new Color(220, 220, 220)); // Din LIGHT_COLOR
        g.fillRect(0, 0, getWidth(), getHeight());
        // -----------------
        // --- DEFINIERA FÄRGER ---
        final Color LIGHT_COLOR = new Color(220, 220, 220);
        final Color DARK_COLOR = new Color(180, 180, 180);
        final Color BLACK_COLOR = new Color(0, 0, 0);
        final Color RED_COLOR = new Color(255, 0, 0);
        final Color GREEN_COLOR = new Color(0, 200, 0);
        // Jag lade till denna för värde 4 (Ljusare röd/rosa)
        final Color LRED_COLOR = new Color(255, 100, 100);
        final Color AGV_COLOR = new Color(255, 160, 165);

        int x, y;
        final int circlesize = 10;
        final int AGVsize = 6;

        // Compute scale factor
        int height = getHeight();
        int width = getWidth();
        double xscale = 1.0 * width / ds.xsize;
        double yscale = 1.0 * height / ds.ysize;

        if (ds.DataAvail == true) {

            // Draw grid
            int maxI = Math.min((int) (ds.xsize / ds.gridsize), ds.ObstacleMatrix.length);
            int maxJ = Math.min((int) (ds.ysize / ds.gridsize), ds.ObstacleMatrix[0].length);

            for (int i = 0; i < maxI; i++) {
                for (int j = 0; j < maxJ; j++) {

                    // Om det är ett HINDER (värde 1, 3 eller 4)
                    if (ds.ObstacleMatrix[i][j] != 0) {

                        // Välj färg baserat på värdet i matrisen
                        if (ds.ObstacleMatrix[i][j] == 1) {
                            g.setColor(DARK_COLOR);
                        } else if (ds.ObstacleMatrix[i][j] == 2) {
                            g.setColor(RED_COLOR);  // ENDAST besökta rutor blir RÖDA
                        } else if (ds.ObstacleMatrix[i][j] == 4) {
                            g.setColor(LRED_COLOR);
                        } else if (ds.ObstacleMatrix[i][j] == 3) {
                            g.setColor(BLACK_COLOR);
                        }

                            // Rita ut hindret med +1 pixel i bredd/höjd för att täppa till glipor
                            g.fillRect((int) (i * xscale * ds.gridsize),
                                    (int) (j * yscale * ds.gridsize),
                                    (int) (ds.gridsize * xscale) + 1,
                                    (int) (ds.gridsize * yscale) + 1);

                        } else {
                            // Om det är GOLV (värde 0) - rita ut rutnätet
                            // Vi använder DARK_COLOR här eftersom den redan finns definierad
                            g.setColor(DARK_COLOR);
                            g.drawRect((int) (i * xscale * ds.gridsize),
                                    (int) (j * yscale * ds.gridsize),
                                    (int) (ds.gridsize * xscale),
                                    (int) (ds.gridsize * yscale));
                        }
                    }
                }

                // Rita ut den nuvarande rutten (svart) "ovanpå" rutnätet om den finns
                if (ds.currentPath != null) {
                    g.setColor(BLACK_COLOR);
                    for (Vertex v : ds.currentPath) {
                        int id = Integer.parseInt(v.getId());
                        int col = id % ds.columns; // X
                        int row = id / ds.columns; // Y
                        g.fillRect((int) (col * xscale * ds.gridsize), (int) (row * yscale * ds.gridsize), (int) (ds.gridsize * xscale), (int) (ds.gridsize * yscale));
                    }
                }

                // Draw first location as filled red circle
                g.setColor(RED_COLOR);
                x = (int) (ds.LocationX[0] * xscale);
                y = (int) (ds.LocationY[0] * yscale);
                g.fillOval(x - (circlesize / 2), y - circlesize / 2, circlesize, circlesize);

                // Draw following locations as filled green circles
                g.setColor(GREEN_COLOR);
                for (int i = 1; i < ds.locations; i++) {
                    x = (int) (ds.LocationX[i] * xscale);
                    y = (int) (ds.LocationY[i] * yscale);
                    g.fillOval(x - (circlesize / 2), y - circlesize / 2, circlesize, circlesize);
                }

                // AGV
                // g.setColor(AGV_COLOR);
                // x = (int) (ds.robotX * xscale);
                // y = (int) (ds.robotY * yscale);
                // g.fillOval((int) (x - AGVsize / 2), (int) (y - AGVsize / 2), AGVsize, AGVsize);
                
                // Nytt, BILD & ROTATION 
                x = (int) (ds.robotX * xscale);
                y = (int) (ds.robotY * yscale);

                java.awt.Graphics2D g2d = (java.awt.Graphics2D) g;
                java.awt.geom.AffineTransform oldTransform = g2d.getTransform();

                // Flytta "pennan"/"pilen" till robotens position på skärmen
                g2d.translate(x, y);
                
                // Rotera pennan med robotens vinkel
                g2d.rotate(ds.robotAngle);
                g2d.rotate(ds.robotAngle); 
                
                java.awt.Image currentImage = ds.isLoaded ? agvLoadedImage : agvImage; 

                if (currentImage != null) {
                    // Om bilden laddades, rita den!
                    // Ändra imgWidth och imgHeight för att ändra storlek på AGV:n i GUI:t
                    int imgWidth = ds.isLoaded ? 50 : 30; 
                    int imgHeight = 20;
                    g2d.drawImage(currentImage, -imgWidth / 2, -imgHeight / 2, imgWidth, imgHeight, null);
                } else {
                    // RESERVLÖSNING: Ritar en gul triangel om agv.png saknas
                    g2d.setColor(AGV_COLOR);
                    g2d.fillPolygon(new int[]{-8, -8, 12}, new int[]{-8, 8, 0}, 3);
                    g2d.setColor(BLACK_COLOR);
                    g2d.drawPolygon(new int[]{-8, -8, 12}, new int[]{-8, 8, 0}, 3);
                }

                // Återställ pennan till original-läget så att inte resten av kartan roterar!
                g2d.setTransform(oldTransform); 

            }

        } // end paintComponent
    }
