// boundaries of map
const double XMIN = -76.0;
const double XMAX = -74.0;
const double YMIN = 39.0;
const double YMAX = 40.0;

struct Point {
  double x;
  double y;

  Point() {
    x = 0.0;
    y = 0.0;
  }

  Point(double newx, double newy) {
    x = newx;
    y = newy;
  }

  bool operator==(const Point& other) const {
    return this->x == other.x && this->y == other.y;
  }

  bool operator!=(const Point& other) const {
    return !(*this == other); // Utilizing the already defined equality operator
  }
};

// Abstract superclass for obstacles
class Obstacle {
    public:
        virtual bool intersect(Point A, Point B) = 0;
};

// Subclass representing line obstacles
class Line : public Obstacle {
    public:
        Point p1;
        Point p2;

        Line(){}

        Line(Point p1, Point p2) {
            this->p1 = p1;
            this->p2 = p2;
        }

        // Determines if two line segments intersect by the direction of rotations
        // If you travel along one line segment, and then towards ONE of the endpoints of the other line segment,
        // the direction of rotation must be different than if you traveled along the same line segment, but towards the OTHER endpoint of the other line segment
        // Repeat this process for the second line segment.
        bool intersect(Point A, Point B) override {
            return (ccw(A, p1, p2) != ccw(B, p1, p2)) && (ccw(A, B, p1) != ccw(A, B, p2)) && !onLine(A) && !onLine(B);
        }

    private:
        static bool ccw(Point A, Point B, Point C) {
            return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
        }

        // determines if a given point p lies on the current line
        bool onLine(Point p) {
            //first checks endpoints
            if (p.y == p1.y)
                return p.x == p1.x;

            if (p.y == p2.y)
                return p.x == p2.x;

            // checks if the ratio between the given point and one endpoint is the same as the given point and the other endpoint
            return ((p.x - p1.x) / (p.y - p1.y) - (p2.x - p.x) / (p2.y - p.y)) == 0;
        }
};

class Quad : public Obstacle {
    public:
        Line l1;
        Line l2;

        Quad(Point p1, Point p2, Point p3, Point p4) {
            this->l1 = Line(p1, p3);
            this->l2 = Line(p2, p4);
        }

        // calls intersect on both lines
        bool intersect(Point A, Point B) override {
            return l1.intersect(A, B) || l2.intersect(A, B);
        }
};

// determines if an obstacle is within a cone of error of the traversal (currently 8 degrees)
bool inError(Point start, Point target, Obstacle &obstacle) {
    double m = (target.y - start.y) / (target.x - start.x);

    double theta = degrees(atan(m));
    if (target.x - start.x < 0) {
        theta += 180;
    }

    double theta1 = theta + 8;
    double theta2 = theta - 8;

    double L = sqrt(pow(target.x - start.x, 2) + pow(target.y - start.y, 2));

    Point p1(start.x + L * cos(radians(theta1)), start.y + L * sin(radians(theta1)));

    Point p2(start.x + L * cos(radians(theta2)), start.y + L * sin(radians(theta2)));

    return obstacle.intersect(start, p1) || obstacle.intersect(start, p2);
}

// determines the closest target point to the starting location
Point closest(Point start, Point targets[], int targetslen) {
    Point toRet = targets[0];
    double shortest = sqrt(pow(targets[0].x - start.x, 2) + pow(targets[0].y - start.y, 2));
    for (int i = 0; i < targetslen; i++) {
        double dist = sqrt(pow(targets[i].x - start.x, 2) + pow(targets[i].y - start.y, 2));
        if (dist < shortest) {
            toRet = targets[i];
            shortest = dist;
        }
    }
    return toRet;
}

