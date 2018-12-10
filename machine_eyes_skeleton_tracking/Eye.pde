/*  Adapted from: The Processing Foundation
 *  Source: https://processing.org/examples/arctangent.html
 *  Contributors: Emilio Ovalles-Misterman & Fiona Sullivan
 */

class Eye {
  int x, y;
  int size;
  float angle = 0.0;
  PImage iris;
  
  Eye(int tx, int ty, int ts) {
    x = tx;
    y = ty;
    size = ts;
  }

  void update(int mx, int my) {
    // if this is flipped when using projector, revert to angle = atan2(my-y, mx-x);
    angle = atan2(my-y, mx-x);
  }
  
  void display() {
    pushMatrix();
    translate(x, y);
    fill(255);
    ellipse(0, 0, size, size);
    rotate(angle);
    fill(153, 204, 0);
    ellipse(size/4, 0, size/2, size/2);
    popMatrix();
  }
  void displayRealEye() {
    iris = loadImage("./images/eye2.png");
    pushMatrix();
    translate(x, y);
    fill(255);
    ellipse(0, 0, size, size);
    rotate(angle);
    // load image here
    // scale(0.27);
    image(iris, -20, -20, size/1.8, size/1.8);
    // fill(153, 204, 0);
    // ellipse(size/4, 0, size/2, size/2);
    popMatrix();
  }
}
