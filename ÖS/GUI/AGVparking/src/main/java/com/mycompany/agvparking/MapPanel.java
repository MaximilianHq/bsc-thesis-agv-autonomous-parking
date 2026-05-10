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
    
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        
        g.setColor(new Color(220, 220, 220)); 
        g.fillRect(0, 0, getWidth(), getHeight());

        final Color LIGHT_COLOR = new Color(220, 220, 220);
        final Color DARK_COLOR = new Color(180, 180, 180);
        final Color BLACK_COLOR = new Color(0, 0, 0);
        final Color RED_COLOR = new Color(255, 0, 0);
        final Color GREEN_COLOR = new Color(0, 200, 0);
        final Color LRED_COLOR = new Color(255, 100, 100);

        int x, y;
        final int circlesize = 10;

        if (ds.DataAvail) {
            // --- LÅS SKALAN TILL 1:1 (PERFEKTA KVADRATER) ---
            double scale = Math.min(1.0 * getWidth() / ds.xsize, 1.0 * getHeight() / ds.ysize);

            int maxI = Math.min((int) (ds.xsize / ds.gridsize), ds.ObstacleMatrix.length);
            int maxJ = Math.min((int) (ds.ysize / ds.gridsize), ds.ObstacleMatrix[0].length);

            // 1. RITA RUTNÄTET
            for (int i = 0; i < maxI; i++) {
                for (int j = 0; j < maxJ; j++) {
                    if (ds.ObstacleMatrix[i][j] != 0) {
                        if (ds.ObstacleMatrix[i][j] == 1) g.setColor(DARK_COLOR);
                        else if (ds.ObstacleMatrix[i][j] == 2) g.setColor(RED_COLOR); 
                        else if (ds.ObstacleMatrix[i][j] == 4) g.setColor(LRED_COLOR);
                        else if (ds.ObstacleMatrix[i][j] == 3) g.setColor(BLACK_COLOR);

                        // +1 pixel i bredd och höjd för att ta bort "glipor" mellan hindren
                        g.fillRect((int) (i * scale * ds.gridsize), (int) (j * scale * ds.gridsize),
                                (int) (ds.gridsize * scale) + 1, (int) (ds.gridsize * scale) + 1);
                    } else {
                        g.setColor(DARK_COLOR);
                        g.drawRect((int) (i * scale * ds.gridsize), (int) (j * scale * ds.gridsize),
                                (int) (ds.gridsize * scale), (int) (ds.gridsize * scale));
                    }
                }
            }

            // 2. RITA SVARTA RUTTEN
            if (ds.currentPath != null) {
                g.setColor(BLACK_COLOR);
                for (Vertex v : ds.currentPath) {
                    int id = Integer.parseInt(v.getId());
                    int col = id % ds.columns;
                    int row = id / ds.columns;
                    g.fillRect((int) (col * scale * ds.gridsize), (int) (row * scale * ds.gridsize), 
                               (int) (ds.gridsize * scale), (int) (ds.gridsize * scale));
                }
            }

            // 3. RITA BLÅ KINEMATIK-LINJEN
            if (ds.robotTrajectory != null && !ds.robotTrajectory.isEmpty()) {
                g.setColor(Color.BLUE);
                java.awt.Graphics2D g2d = (java.awt.Graphics2D) g;
                g2d.setStroke(new java.awt.BasicStroke(3)); 
                
                for (int i = 0; i < ds.robotTrajectory.size() - 1; i++) {
                    Point2D p1 = ds.robotTrajectory.get(i);
                    Point2D p2 = ds.robotTrajectory.get(i + 1);
                    g2d.drawLine((int) (p1.x * scale), (int) (p1.y * scale),
                                 (int) (p2.x * scale), (int) (p2.y * scale));
                }
                g2d.setStroke(new java.awt.BasicStroke(1));
            }

            // 4. RITA START OCH MÅL
            g.setColor(RED_COLOR);
            x = (int) (ds.LocationX[0] * scale);
            y = (int) (ds.LocationY[0] * scale);
            g.fillOval(x - (circlesize / 2), y - circlesize / 2, circlesize, circlesize);

            g.setColor(GREEN_COLOR);
            for (int i = 1; i < ds.locations; i++) {
                x = (int) (ds.LocationX[i] * scale);
                y = (int) (ds.LocationY[i] * scale);
                g.fillOval(x - (circlesize / 2), y - circlesize / 2, circlesize, circlesize);
            }

            // --- 5. RITA EKIPAGET SOM EN STEL KROPP ---
            java.awt.Graphics2D g2d = (java.awt.Graphics2D) g.create();
            
            // Vi rör oss till AGV:n och roterar miljön
            g2d.translate(ds.robotX * scale, ds.robotY * scale);
            g2d.rotate(ds.robotAngle);

            if (ds.isLoaded) {
                g2d.setColor(Color.MAGENTA);
                g2d.setStroke(new java.awt.BasicStroke(2));
                
                // Räkna ut avståndet i pixlar baserat på den perfekta skalan
                int L_pixels = (int) (ds.agvOffset * ds.gridsize * scale);
                
                // Rita dragstången och axeln relativt till AGV:n
                g2d.drawLine(0, 0, -L_pixels, 0); 
                g2d.fillOval(-L_pixels - 5, -5, 10, 10); 
            }

            // Rita AGV:n
            g2d.setColor(Color.BLUE);
            g2d.fillOval(-7, -7, 14, 14);
            
            // Riktning (Nos)
            g2d.setColor(Color.WHITE);
            g2d.drawLine(0, 0, 7, 0);
            
            g2d.dispose();
        }
    }
    
    
    }
