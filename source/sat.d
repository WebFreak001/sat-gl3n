module sat;

import gl3n.linalg;
import std.conv;
import std.file;
import std.range;
import std.math : cos, sin;
import std.typecons;

private mixin template Transformation2D()
{
	/// Transformation matrix
	mat3 transformation = mat3.identity;

	/// Translates the shape.
	typeof(this) translate(vec2 offset) @safe @nogc nothrow pure
	{
		return translate(offset.x, offset.y);
	}

	/// Translates the shape.
	typeof(this) translate(float x, float y) @safe @nogc nothrow pure
	{
		transformation *= mat3(1, 0, x, 0, 1, y, 0, 0, 1);
		return this;
	}

	/// Scales the shape.
	typeof(this) scale(vec2 amount) @safe @nogc nothrow pure
	{
		return scale(amount.x, amount.y);
	}

	/// Scales the shape.
	typeof(this) scale(float x, float y) @safe @nogc nothrow pure
	{
		transformation *= mat3(x, 0, 0, 0, y, 0, 0, 0, 1);
		return this;
	}

	/// Rotates the shape.
	typeof(this) rotate(float theta) @safe @nogc nothrow pure
	{
		immutable c0 = cos(theta);
		immutable s0 = sin(theta);
		transformation *= mat3(c0, -s0, 0, s0, c0, 0, 0, 0, 1);
		return this;
	}

	typeof(this) reset() @safe @nogc nothrow pure
	{
		transformation = mat3.identity;
		return this;
	}
}

/// Represents a circle in 2D space with origin at (0|0). +X = right, +Y = up
struct Circle
{
	/// Radius of the circle
	float radius;
	/// Position of the circle
	vec2 position;
}

/// Represents a convex polygon with n vertices in 2D space with origin at (0|0). +X = right, +Y = up
struct Polygon
{
	/// Clockwise array of two-dimenstional vectors representing the corners of the convex polygon.
	vec2[] vertices;

	mixin Transformation2D;

	/// Generates the normals for all vertices
	auto normals() @property @safe const nothrow pure @nogc
	in(vertices.length > 2, "Any polygon must have at least 3 vertices")
	{
		return .normals(vertices, transformation);
	}
}

/// Generates normals of a 2D shape on the fly
struct Normals(Vertices)
{
	Vertices vertices;
	vec2 first, last;
	mat3 transformation;
	bool didLast;

	static if (isForwardRange!Vertices)
		auto save()
		{
			return Normals!Vertices(vertices.save, first, last);
		}

	static if (hasLength!Vertices)
		auto length() @property const
		{
			return vertices.length;
		}

	void popFront()
	{
		if (vertices.empty && !didLast)
			didLast = true;
		else
		{
			last = vertices.front;
			vertices.popFront;
		}
	}

	bool empty() @property const
	{
		return vertices.empty && didLast;
	}

	vec2 front() @property const @safe
	{
		vec2 a;
		if (vertices.empty)
			a = (transformation * vec3(first, 1)).xy;
		else
			a = (transformation * vec3(vertices.front, 1)).xy;
		const b = (transformation * vec3(last, 1)).xy;
		return vec2(b.y - a.y, a.x - b.x).normalized;
	}
}

Normals!Vertices normals(Vertices)(Vertices vertices, mat3 transformation) @safe nothrow pure @nogc
{
	Normals!Vertices ret;
	ret.vertices = vertices;
	ret.first = ret.last = vertices.front;
	ret.transformation = transformation;
	vertices.popFront;
	ret.popFront;
	return ret;
}

/// Creates an axis aligned rectangle as polygon.
Polygon rectangleAABB(vec2 center, vec2 halfDimensions) @safe nothrow pure
{
	return Polygon([
			center + vec2(halfDimensions.x, halfDimensions.y),
			center + vec2(halfDimensions.x, -halfDimensions.y),
			center + vec2(-halfDimensions.x, -halfDimensions.y),
			center + vec2(-halfDimensions.x, halfDimensions.y)
			]);
}

/// Checks for an intersection between two polygons
bool intersects(in Polygon a, in Polygon b) @safe nothrow pure @nogc
{
	Tuple!(float, "min", float, "max") rangeA;
	Tuple!(float, "min", float, "max") rangeB;
	int i;
	foreach (norm; a.normals)
	{
		rangeA = project(a, norm);
		rangeB = project(b, norm);
		if (rangeA.max < rangeB.min || rangeB.max < rangeA.min)
			return false;
	}
	foreach (norm; b.normals)
	{
		rangeA = project(a, norm);
		rangeB = project(b, norm);
		if (rangeA.max < rangeB.min || rangeB.max < rangeA.min)
			return false;
	}
	return true;
}

unittest
{
	// poly <> poly without transformation
	//dfmt off
	// Simple shapes touching
	assert(intersects(Polygon([
		vec2(-1, -1),
		vec2(1, -1),
		vec2(-1, 1)]),
		rectangleAABB(vec2(0, 0), vec2(1, 1))));
	// Simple shapes not touching
	assert(!intersects(Polygon(
		[vec2(-1, -1),
		vec2(1, -1),
		vec2(-1, 1)]),
		rectangleAABB(vec2(2, 2), vec2(1, 1))));
	// Complex shapes not touching
	assert(!intersects(Polygon(
		[vec2(-2, 2),
		vec2(-1, 2),
		vec2(2, 0),
		vec2(2, -1),
		vec2(-2, -1)]
	), Polygon(
		[vec2(1, 1),
		vec2(0, 3),
		vec2(3, 3),
		vec2(4, 0)]
	)));
	// shape inside shape
	assert(intersects(
		rectangleAABB(vec2(0, 0), vec2(2, 2)),
		rectangleAABB(vec2(0, 0), vec2(1, 1))
	));
	//dfmt on
}

unittest
{
	// poly <> poly with transformation
	//dfmt off
	// Simple shapes touching
	assert(intersects(Polygon([
		vec2(-1, -1),
		vec2(1, -1),
		vec2(-1, 1)]).translate(5, 5),
		rectangleAABB(vec2(0, 0), vec2(1, 1)).translate(5, 5)));
	// Simple shapes not touching
	assert(!intersects(Polygon(
		[vec2(-1, -1),
		vec2(1, -1),
		vec2(-1, 1)]).scale(0.5f, 0.5f),
		rectangleAABB(vec2(2, 2), vec2(1, 1)).scale(2, 2)));
	// Complex shapes touching
	assert(intersects(Polygon(
		[vec2(-2, 2),
		vec2(-1, 2),
		vec2(2, 0),
		vec2(2, -1),
		vec2(-2, -1)]
	), Polygon(
		[vec2(1, 1),
		vec2(0, 3),
		vec2(3, 3),
		vec2(4, 0)]
	).translate(-1, 0)));
	// shape inside shape
	assert(intersects(
		rectangleAABB(vec2(1, 0), vec2(1, 1)).translate(-1, 0),
		rectangleAABB(vec2(0, 0), vec2(1, 1))
	));
	//dfmt on
}

/// Checks for an intersection between a polygon and a circle
bool intersects(in Polygon a, in Circle b) @safe nothrow pure @nogc
{
	Tuple!(float, "min", float, "max") rangeA;
	Tuple!(float, "min", float, "max") rangeB;
	int i;
	for (i = 0; i < a.vertices.length; i++)
	{
		const axis = (a.vertices[i] - b.position).normalized;
		rangeA = project(a, axis);
		rangeB = project(b, axis);
		if (rangeA.max < rangeB.min || rangeB.max < rangeA.min)
			return false;
	}
	foreach (norm; a.normals)
	{
		rangeA = project(a, norm);
		rangeB = project(b, norm);
		if (rangeA.max < rangeB.min || rangeB.max < rangeA.min)
			return false;
	}
	return true;
}

/// ditto
bool intersects(in Circle b, in Polygon a) @safe nothrow pure @nogc
{
	return intersects(a, b);
}

unittest
{
	// poly <> circle
	//dfmt off
	assert(!intersects(Polygon([
		vec2(-1, 2),
		vec2(1, -1),
		vec2(-1, -1)
	]), Circle(1, vec2(1, 1))));
	assert(intersects(Polygon([
		vec2(-1, 1.3),
		vec2(2, -1),
		vec2(-1, -1)
	]), Circle(1, vec2(1, 1))));
	//dfmt on
}

/// Checks for an intersection between two circles
bool intersects(in Circle a, in Circle b) @safe nothrow pure @nogc
{
	return (a.position - b.position).length_squared < (a.radius + b.radius) * (a.radius + b.radius);
}

unittest
{
	// circle <> circle
	//dfmt off
	assert(intersects(Circle(1, vec2(0, 0)), Circle(1, vec2(0.5f, 0.5f))));
	assert(intersects(Circle(1, vec2(0, 0)), Circle(1, vec2(1, 1))));
	assert(!intersects(Circle(1, vec2(0, 0)), Circle(1, vec2(2, 1))));
	//dfmt on
}

/// Projects a polygon onto an axis.
/// Returns: [mininum, maximum]
Tuple!(float, "min", float, "max") project(in Polygon poly, vec2 axis) @safe nothrow pure @nogc
{
	float min = (poly.transformation * vec3(poly.vertices[0], 1)).xy.dot(axis);
	float max = (poly.transformation * vec3(poly.vertices[0], 1)).xy.dot(axis);
	for (int i = 1; i < poly.vertices.length; i++)
	{
		immutable curr = (poly.transformation * vec3(poly.vertices[i], 1)).xy.dot(axis);
		if (curr < min)
			min = curr;
		if (curr > max)
			max = curr;
	}
	Tuple!(float, "min", float, "max") ret;
	ret.min = min;
	ret.max = max;
	return ret;
}

/// Projects a polygon onto an axis.
/// Returns: [mininum, maximum]
Tuple!(float, "min", float, "max") project(in Circle circle, vec2 axis) @safe nothrow pure @nogc
in
{
	assert(circle.radius > 0, "A circle must have a positive radius");
}
body
{
	immutable center = circle.position.dot(axis);
	Tuple!(float, "min", float, "max") ret;
	ret.min = center - circle.radius;
	ret.max = center + circle.radius;
	return ret;
}
