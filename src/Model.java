// The contents of this file are dedicated to the public domain.
// (See http://creativecommons.org/publicdomain/zero/1.0/)

import java.awt.Graphics;
import java.io.File;
import java.util.Random;
import java.util.ArrayList;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import javax.imageio.ImageIO;

class Model {
	public static final double EPSILON = 0.0001f; // A small number
	public static final double XMAX = 1200.0f - EPSILON; // The maximum horizontal screen position. (The minimum is 0.)
	public static final double YMAX = 600.0f - EPSILON; // The maximum vertical screen position. (The minimum is 0.)

	private Controller controller;
	private byte[] terrain;
	private ArrayList<Sprite> sprites;
	Model() {

	}

	Model(Controller c) {
		this.controller = c;
	}

	Model getModel() { return this; }


	void initGame() throws Exception {
		BufferedImage bufferedImage = ImageIO.read(new File("terrain.png"));
		if(bufferedImage.getWidth() != 60 || bufferedImage.getHeight() != 60)
			throw new Exception("Expected the terrain image to have dimensions of 60-by-60");
		terrain = ((DataBufferByte)bufferedImage.getRaster().getDataBuffer()).getData();
		sprites = new ArrayList<Sprite>();
		sprites.add(new Sprite(100, 100));
	}

	// These methods are used internally. They are not useful to the agents.
	byte[] getTerrain() { return this.terrain; }
	ArrayList<Sprite> getSprites() { return this.sprites; }

	void update() {
		// Update the agents
		for(int i = 0; i < sprites.size(); i++)
			sprites.get(i).update();
	}

	// 0 <= x < MAP_WIDTH.
	// 0 <= y < MAP_HEIGHT.
	double getTravelSpeed(double x, double y) {
			int xx = (int)(x * 0.1f);
			int yy = (int)(y * 0.1f);
			if(xx >= 60)
			{
				xx = 119 - xx;
				yy = 59 - yy;
			}

			int pos = 4 * (60 * yy + xx);
/*			if (pos > terrain.length){
				if(pos < terrain.length){

				}
			}*/
			return Math.max(0.2f, Math.min(3.5f, -0.01f * (terrain[pos + 1] & 0xff) + 0.02f * (terrain[pos + 3] & 0xff)));

	}

	Controller getController() { return controller; }
	double getX() { return sprites.get(0).x; }
	double getY() { return sprites.get(0).y; }
	double getDestinationX() { return sprites.get(0).xDestination; }
	double getDestinationY() { return sprites.get(0).yDestination; }

	void setDestination(double x, double y) {
		Sprite s = sprites.get(0);
		s.xDestination = x;
		s.yDestination = y;
	}

	double getDistanceToDestination(int sprite) {
		Sprite s = sprites.get(sprite);
		return Math.sqrt((s.x - s.xDestination) * (s.x - s.xDestination) + (s.y - s.yDestination) * (s.y - s.yDestination));
	}

	class Sprite {
		double x;
		double y;
		double xDestination;
		double yDestination;

		Sprite(double x, double y) {
			this.x = x;
			this.y = y;
			this.xDestination = x;
			this.yDestination = y;
		}

		void update() {
			double speed = Model.this.getTravelSpeed(this.x, this.y);
			double dx = this.xDestination - this.x;
			double dy = this.yDestination - this.y;
			double dist = (float)Math.sqrt(dx * dx + dy * dy);
			double t = speed / Math.max(speed, dist);
			dx *= t;
			dy *= t;
			this.x += dx;
			this.y += dy;
			this.x = Math.max(0.0f, Math.min(XMAX, this.x));
			this.y = Math.max(0.0f, Math.min(YMAX, this.y));
		}
	}
}
