//Liam Bockelmann ID: 108613490

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glut.h>

#ifndef myHeader
#define myHeader

class Vector
{
public:
	Vector();
	Vector(double mX, double mY, double mZ);

	double x;
	double y;
	double z;
};

Vector::Vector()
{
	x = 0;
	y = 0;
	z = 0;
}

Vector::Vector(double mX, double mY, double mZ)
{
	x = mX;
	y = mY;
	z = mZ;
}

void normalize(Vector &v)
{
	GLdouble d = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (d == 0.0) {
		printf("zero length Vector");
		return;
	}
	v.x /= d;
	v.y /= d;
	v.z /= d;
}

Vector normalize2(Vector v)
{
	GLdouble d = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (d == 0.0) {
		printf("zero length Vector");
		return v;
	}
	v.x /= d;
	v.y /= d;
	v.z /= d;
	return v;
}

Vector subtractVec(Vector a, Vector b)
{
	Vector result = Vector();
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return result;
}

Vector addVec(Vector a, Vector b)
{
	Vector result = Vector();
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	return result;
}

double multiply(const Vector &a, const Vector &b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector crossProduct(const Vector &a, const Vector &b)
{
	return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

double dotProduct(const Vector &a, const Vector &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector scaleVec(double a, Vector &b)
{
	return Vector(b.x*a, b.y*a, b.z*a);
}

double squaredLength(Vector a)
{
	return (a.x * a.x) + (a.y * a.y) + (a.z * a.z);
}

class Tetra
{
public:
	Tetra();

	GLdouble vertex1x;
	GLdouble vertex1y;
	GLdouble vertex1z;
	GLdouble vertex2x;
	GLdouble vertex2y;
	GLdouble vertex2z;
	GLdouble vertex3x;
	GLdouble vertex3y;
	GLdouble vertex3z;
	GLdouble vertex4x;
	GLdouble vertex4y;
	GLdouble vertex4z;
};

Tetra::Tetra()
{
	vertex1x = 0.0;
	vertex1y = 1.0;
	vertex1z = 0.0;
	vertex2x = -1.0;
	vertex2y = -1.0;
	vertex2z = 0.0; 
	vertex3x = 1.0;
	vertex3y = -1.0;
	vertex3z = 0.0;
	vertex4x = 0.0;
	vertex4y = -1.0;
	vertex4z = -1.0;
}

class PLANE {
public:
	double equation[4];
	Vector origin;
	Vector normal;
	PLANE(const Vector& origin, const Vector& normal);
	PLANE(const Vector& p1, const Vector& p2, const Vector& p3);
	bool isFrontFacingTo(const Vector& direction) const;
	double signedDistanceTo(const Vector& point) const;
};

PLANE::PLANE(const Vector& origin, const Vector& normal) {
	this->normal = normal;
	this->origin = origin;
	equation[0] = normal.x;
	equation[1] = normal.y;
	equation[2] = normal.z;
	equation[3] = -(normal.x*origin.x + normal.y*origin.y + normal.z*origin.z);
}

// Construct from triangle:
PLANE::PLANE(const Vector& p1, const Vector& p2,const Vector& p3)
{
	normal = crossProduct(subtractVec(p2, p1), subtractVec(p3, p1));
	normalize(normal);
	origin = p1;
	equation[0] = normal.x;
	equation[1] = normal.y;
	equation[2] = normal.z;
	equation[3] = -(normal.x*origin.x + normal.y*origin.y + normal.z*origin.z);
}
bool PLANE::isFrontFacingTo(const Vector& direction) const {
	double dot = dotProduct(normal, direction);
	return (dot <= 0);
}
double PLANE::signedDistanceTo(const Vector& point) const {
	return (dotProduct(point, normal)) + equation[3];
}

class CollisionPacket {
public:
	Vector eRadius; // ellipsoid radius
					// Information about the move being requested: (in R3)
	Vector R3Velocity;
	Vector R3Position;
	// Information about the move being requested: (in eSpace)
	Vector velocity;
	Vector normalizedVelocity;
	Vector basePoint;
	// Hit information
	bool foundCollision;
	double nearestDistance;
	Vector intersectionPoint;
	CollisionPacket(Vector radius, Vector rVelocity, Vector rPosition, Vector eVelocity, Vector eNormalizedVelocity, Vector eBasePoint, bool collision);
};

CollisionPacket::CollisionPacket(Vector radius, Vector rVelocity, Vector rPosition, Vector eVelocity, Vector eNormalizedVelocity, Vector eBasePoint, bool collision)
{
	eRadius = radius;
	R3Velocity = rVelocity;
	R3Position = rPosition;
	velocity = eVelocity;
	normalizedVelocity = eNormalizedVelocity;
	basePoint = eBasePoint;
	foundCollision = collision;
	nearestDistance = 999999999;
}

typedef unsigned int uint32;
#define in(a) ((uint32&) a)
bool checkPointInTriangle(const Vector& point,
	const Vector& pa, const Vector& pb, const Vector& pc)
{
	Vector e10 = subtractVec(pb, pa);
	Vector e20 = subtractVec(pc, pa);
	float a = dotProduct(e10,e10);
	float b = dotProduct(e10,e20);
	float c = dotProduct(e20,e20);
	float ac_bb = (a*c) - (b*b);
	Vector vp(point.x - pa.x, point.y - pa.y, point.z - pa.z);
	float d = dotProduct(vp,e10);
	float e = dotProduct(vp,e20);
	float x = (d*c) - (e*b);
	float y = (e*a) - (d*b);
	float z = x + y - ac_bb;
	return ((in(z)& ~(in(x) | in(y))) & 0x80000000);
}
bool getLowestRoot(float a, float b, float c, float maxR,
	float* root) {
	// Check if a solution exists
	float determinant = b * b - 4.0f*a*c;
	// If determinant is negative it means no solutions.
	if (determinant < 0.0f) return false;
	// calculate the two roots: (if determinant == 0 then
	// x1==x2 but let’s disregard that slight optimization)
	float sqrtD = sqrt(determinant);
	float r1 = (-b - sqrtD) / (2 * a);
	float r2 = (-b + sqrtD) / (2 * a);
	// Sort so x1 <= x2
	if (r1 > r2) {
		float temp = r2;
		r2 = r1;
		r1 = temp;
	}
	// Get lowest root:
	if (r1 > 0 && r1 < maxR) {
		*root = r1;
		return true;
	}
	// It is possible that we want x2 - this can happen
	// if x1 < 0
	if (r2 > 0 && r2 < maxR) {
		*root = r2;
		return true;
	}
	// No (valid) solutions
	return false;
}

// Assumes: p1,p2 and p3 are given in ellisoid space:
void checkTriangle(CollisionPacket* colPackage,
	const Vector& p1, const Vector& p2, const Vector& p3)
{
	// Make the plane containing this triangle.
	PLANE trianglePlane(p1, p2, p3);
	// Is triangle front-facing to the velocity Vector?
	// We only check front-facing triangles
	// (your choice of course)

	//if (trianglePlane.isFrontFacingTo(
		//colPackage->normalizedVelocity)) {

		// Get interval of plane intersection:
		double t0, t1;
		bool embeddedInPlane = false;
		// Calculate the signed distance from sphere
		// position to triangle plane
		double signedDistToTrianglePlane =
			trianglePlane.signedDistanceTo(colPackage->basePoint);
		// cache this as we’re going to use it a few times below:
		float normalDotVelocity =
			dotProduct(trianglePlane.normal, colPackage->velocity);
		// if sphere is travelling parrallel to the plane:
		if (normalDotVelocity == 0.0f) {
			if (fabs(signedDistToTrianglePlane) >= 1.0f) {
				// Sphere is not embedded in plane.
				// No collision possible:
				return;
			}
			else {
				// sphere is embedded in plane.
				// It intersects in the whole range [0..1]
				embeddedInPlane = true;
				t0 = 0.0;
				t1 = 1.0;
			}
		}
		else {
			// N dot D is not 0. Calculate intersection interval:
			t0 = (-1.0 - signedDistToTrianglePlane) / normalDotVelocity;
			t1 = (1.0 - signedDistToTrianglePlane) / normalDotVelocity;
			// Swap so t0 < t1
			if (t0 > t1) {
				double temp = t1;
				t1 = t0;
				t0 = temp;
			}
			// Check that at least one result is within range:
			if (t0 > 1.0f || t1 < 0.0f) {
				// Both t values are outside values [0,1]
				// No collision possible:
				return;
			}
			// Clamp to [0,1]
			if (t0 < 0.0) t0 = 0.0;
			if (t1 < 0.0) t1 = 0.0;
			if (t0 > 1.0) t0 = 1.0;
			if (t1 > 1.0) t1 = 1.0;
		}
		// OK, at this point we have two time values t0 and t1
		// between which the swept sphere intersects with the
		// triangle plane. If any collision is to occur it must
		// happen within this interval.
		Vector collisionPoint;
		bool foundCollison = false;
		float t = 1.0;
		// First we check for the easy case - collision inside
		// the triangle. If this happens it must be at time t0
		// as this is when the sphere rests on the front side
		// of the triangle plane. Note, this can only happen if
		// the sphere is not embedded in the triangle plane.
		if (!embeddedInPlane) {
			Vector planeIntersectionPoint = addVec(subtractVec(colPackage->basePoint, trianglePlane.normal) , scaleVec(t0, colPackage->velocity));
			if (checkPointInTriangle(planeIntersectionPoint,p1, p2, p3))
			{
				foundCollison = true;
				t = t0;
				collisionPoint = planeIntersectionPoint;
			}
		}
		// if we haven’t found a collision already we’ll have to
		// sweep sphere against points and edges of the triangle.
		// Note: A collision inside the triangle (the check above)
		// will always happen before a vertex or edge collision!
		// This is why we can skip the swept test if the above
		// gives a collision!
		if (foundCollison == false) {
			// some commonly used terms:
			Vector velocity = colPackage->velocity;
			Vector base = colPackage->basePoint;
			float velocitySquaredLength = squaredLength(velocity);
			float a, b, c; // Params for equation
			float newT;
			// For each vertex or edge a quadratic equation have to
			// be solved. We parameterize this equation as
			// a*t^2 + b*t + c = 0 and below we calculate the
			// parameters a,b and c for each test.
			// Check against points:
			a = velocitySquaredLength;

			// P1
			b = 2.0*dotProduct(velocity, subtractVec(base, p1));
			c = squaredLength(subtractVec(p1, base)) - 1.0;
			if (getLowestRoot(a, b, c, t, &newT)) {
				t = newT;
				foundCollison = true;
				collisionPoint = p1;
			}
			// P2
			b = 2.0*dotProduct(velocity,subtractVec(base, p2));
			c = squaredLength(subtractVec(p2, base)) - 1.0;
			if (getLowestRoot(a, b, c, t, &newT)) {
				t = newT;
				foundCollison = true;
				collisionPoint = p2;
			}
			// P3
			b = 2.0*dotProduct(velocity,subtractVec(base, p3));
			c = squaredLength(subtractVec(p3, base)) - 1.0;
			if (getLowestRoot(a, b, c, t, &newT)) {
				t = newT;
				foundCollison = true;
				collisionPoint = p3;
			}
			// Check agains edges:
			// p1 -> p2:
			Vector edge = subtractVec(p2, p1);
			Vector baseToVertex = subtractVec(p1, base);
			float edgeSquaredLength = squaredLength(edge);
			float edgeDotVelocity = dotProduct(edge,velocity);
			float edgeDotBaseToVertex = dotProduct(edge,baseToVertex);
			// Calculate parameters for equation
			a = edgeSquaredLength * -velocitySquaredLength +
				edgeDotVelocity * edgeDotVelocity;
			b = edgeSquaredLength * (2 * dotProduct(velocity,baseToVertex)) -
				2.0*edgeDotVelocity*edgeDotBaseToVertex;
			c = edgeSquaredLength * (1 - squaredLength(baseToVertex)) +
				edgeDotBaseToVertex * edgeDotBaseToVertex;
			// Does the swept sphere collide against infinite edge?
			if (getLowestRoot(a, b, c, t, &newT)) {
				// Check if intersection is within line segment:
				float f = (edgeDotVelocity*newT - edgeDotBaseToVertex) /
					edgeSquaredLength;
				if (f >= 0.0 && f <= 1.0) {
					// intersection took place within segment.
					t = newT;
					foundCollison = true;
					collisionPoint = addVec(p1, scaleVec(f, edge));
				}
			}
			// p2 -> p3:
			edge = subtractVec(p3, p2);
			baseToVertex = subtractVec(p2, base);
			edgeSquaredLength = squaredLength(edge);
			edgeDotVelocity = dotProduct(edge, velocity);
			edgeDotBaseToVertex = dotProduct(edge, baseToVertex);
			a = edgeSquaredLength * -velocitySquaredLength +
				edgeDotVelocity * edgeDotVelocity;
			b = edgeSquaredLength * (2 * dotProduct(velocity, baseToVertex)) -
				2.0*edgeDotVelocity*edgeDotBaseToVertex;
			c = edgeSquaredLength * (1 - squaredLength(baseToVertex)) +
				edgeDotBaseToVertex * edgeDotBaseToVertex;
			if (getLowestRoot(a, b, c, t, &newT)) {
				float f = (edgeDotVelocity*newT - edgeDotBaseToVertex) /
					edgeSquaredLength;
				if (f >= 0.0 && f <= 1.0) {
					t = newT;
					foundCollison = true;
					collisionPoint = addVec(p2, scaleVec(f, edge));
				}
			}
			// p3 -> p1:
			edge = subtractVec(p1, p3);
			baseToVertex = subtractVec(p3, base);
			edgeSquaredLength = squaredLength(edge);
			edgeDotVelocity = dotProduct(edge, velocity);
			edgeDotBaseToVertex = dotProduct(edge, baseToVertex);
			a = edgeSquaredLength * -velocitySquaredLength +
				edgeDotVelocity * edgeDotVelocity;
			b = edgeSquaredLength * (2 * dotProduct(velocity, baseToVertex)) -
				2.0*edgeDotVelocity*edgeDotBaseToVertex;
			c = edgeSquaredLength * (1 - squaredLength(baseToVertex)) +
				edgeDotBaseToVertex * edgeDotBaseToVertex;
			if (getLowestRoot(a, b, c, t, &newT)) {
				float f = (edgeDotVelocity*newT - edgeDotBaseToVertex) /
					edgeSquaredLength;
				if (f >= 0.0 && f <= 1.0) {
					t = newT;
					foundCollison = true;
					collisionPoint = addVec(p3, scaleVec(f, edge));
				}
			}
		}
		// Set result:
		if (foundCollison == true) {
			// distance to collision: ’t’ is time of collision
			float distToCollision = t * sqrt(squaredLength(colPackage->velocity));
			// Does this triangle qualify for the closest hit?
			// it does if it’s the first hit or the closest
			if (colPackage->foundCollision == false || distToCollision < colPackage->nearestDistance) {
				// Collision information nessesary for sliding
				colPackage->nearestDistance = distToCollision;
				colPackage->intersectionPoint = collisionPoint;
				colPackage->foundCollision = true;
			}
		}
	//} // if not backface
}

#endif
