package com.mycompany.autopark;

import java.awt.Color;
import java.awt.Graphics;
import javax.swing.JPanel;

/**
 *
 * @author clary35
 */
public class MapPanel extends JPanel {

    DataStore ds;

    MapPanel(DataStore ds) {
        this.ds = ds;

        // Din manuella initiering av hinder
        ds.ObstacleMatrix[11][11] = 1;
        ds.ObstacleMatrix[11][12] = 1;
        ds.ObstacleMatrix[11][13] = 1;
        ds.ObstacleMatrix[12][13] = 1;
        ds.ObstacleMatrix[13][13] = 1;
        ds.ObstacleMatrix[13][12] = 1;
        ds.ObstacleMatrix[13][11] = 1;
        ds.ObstacleMatrix[12][11] = 1;

        // Horisontella parkeringsrutor (Fixad loop)
        for (int k = 0; k < ds.Hspaces; k++) {
            int xpos = (int) (ds.HLocationX[k] / ds.gridsize);
            int ypos = (int) (ds.HLocationY[k] / ds.gridsize);
            for (int i = 0; i < (int) (110 / ds.gridsize); i = i + 1) {
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
                for (int j = 0; j <= (int) (110 / ds.gridsize); j = j + 1) {
                    ds.ObstacleMatrix[xpos + i][ypos + j] = 1;
                }
            }
        }
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // --- DEFINIERA FÄRGER ---
        final Color LIGHT_COLOR = new Color(220, 220, 220);
        final Color DARK_COLOR = new Color(180, 180, 180);
        final Color BLACK_COLOR = new Color(0, 0, 0);
        final Color RED_COLOR = new Color(255, 0, 0);
        final Color GREEN_COLOR = new Color(0, 200, 0);
        // Jag lade till denna för värde 4 (Ljusare röd/rosa)
        final Color LRED_COLOR = new Color(255, 100, 100);
        final Color AGV_COLOR = new Color(255, 255, 0);

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
            for (int i = 0; i < (int) (ds.xsize / ds.gridsize); i = i + 1) {
                for (int j = 0; j < (int) (ds.ysize / ds.gridsize); j = j + 1) {

                    // --- HÄR ÄR ÄNDRINGEN DU BAD OM ---
                    if (ds.ObstacleMatrix[i][j] == 0) {
                        g.setColor(LIGHT_COLOR);
                    } else if (ds.ObstacleMatrix[i][j] == 1) {
                        g.setColor(DARK_COLOR);
                    } else if (ds.ObstacleMatrix[i][j] == 4) {
                        g.setColor(LRED_COLOR);
                    } else if (ds.ObstacleMatrix[i][j] == 3) {
                        g.setColor(BLACK_COLOR);
                    }
                    // -----------------------------------

                    // Fyll rutan med vald färg
                    g.fillRect((int) (i * xscale * ds.gridsize), (int) (j * yscale * ds.gridsize), (int) (ds.gridsize * xscale), (int) (ds.gridsize * yscale));

                    // Rita rutnäts-kant (Förslagsvis mörkgrå så man ser rutnätet även på ljusa rutor)
                    g.setColor(DARK_COLOR);
                    g.drawRect((int) (i * xscale * ds.gridsize), (int) (j * yscale * ds.gridsize), (int) (ds.gridsize * xscale), (int) (ds.gridsize * yscale));
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
            g.setColor(AGV_COLOR);
            x = (int) (ds.robotX * xscale);
            y = (int) (ds.robotY * yscale);
            g.fillOval((int) (x - AGVsize / 2), (int) (y - AGVsize / 2), AGVsize, AGVsize);

        }

    } // end paintComponent
}
