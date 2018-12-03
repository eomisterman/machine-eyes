class Eye {
  int x, y;
  int size;
  float angle = 0.0;
  
  PImage iris;
  iris = image('/path');
  
  Eye(int tx, int ty, int ts) {
    x = tx;
    y = ty;
    size = ts;
 }

  void update(int mx, int my) {
    angle = atan2(my-y, mx-x);
    println("angle:\t" + Float.toString(angle));
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
    pushMatrix();
    translate(x, y);
    fill(255);
    ellipse(0, 0, size, size);
    rotate(angle);
    // load image here
    // fill(153, 204, 0);
    // ellipse(size/4, 0, size/2, size/2);
    popMatrix();
  }
}
