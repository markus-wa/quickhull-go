package quickhull

import (
	"fmt"
	"math/rand"
	"reflect"
	"testing"

	"github.com/golang/geo/r3"
)

// https://stackoverflow.com/questions/36000487/check-for-equality-on-slices-without-order
func elementsMatch(xs, ys []r3.Vector) bool {
	if len(xs) != len(ys) {
		return false
	}

	// create a map of string -> int
	diff := make(map[r3.Vector]int, len(xs))
	for _, x := range xs {
		// 0 value for int is 0, so just increment a counter for the string
		diff[x]++
	}

	for _, y := range ys {
		// If the string y is not in diff bail out early
		if _, ok := diff[y]; !ok {
			return false
		}

		diff[y]--

		if diff[y] == 0 {
			delete(diff, y)
		}
	}

	return len(diff) == 0
}

func assertElementsMatch(t *testing.T, xs, ys []r3.Vector, msgAndArgs ...interface{}) {
	t.Helper()

	if !elementsMatch(xs, ys) {
		msg := fmt.Sprint(msgAndArgs...)

		if msg == "" {
			msg = "assertion failed"
		}

		t.Errorf("%s: elements do not match: %v != %v", msg, xs, ys)
	}
}

func assertEqual(t *testing.T, expected, actual interface{}) {
	t.Helper()

	if !reflect.DeepEqual(expected, actual) {
		t.Errorf("assertion failed: %v != %v", expected, actual)
	}
}

// Simple 2D test (square, all points on a plane)
func TestConvexHull2DSquare(t *testing.T) {
	// Construct a square as 'point cloud' that looks roughly like this.
	// All points are on a single plane (Z=1).
	// E is on the border but should not be part of the Convex Hull.
	// F should be inside the Convex Hull.
	//
	// C - - - - - - D
	// |             |
	// |             |
	// E      F      |
	// |             |
	// |             |
	// A - - - - - - B
	//
	pointCloud := []r3.Vector{
		{X: 0, Y: 0, Z: 1},
		{X: 0, Y: 5, Z: 1},
		{X: 0, Y: 10, Z: 1},
		{X: 10, Y: 0, Z: 1},
		{X: 10, Y: 10, Z: 1},
		{X: 5, Y: 5, Z: 1}, // This point is inside the hull
	}

	expectedHull := []r3.Vector{
		{X: 0, Y: 0, Z: 1},
		{X: 0, Y: 10, Z: 1},
		{X: 10, Y: 0, Z: 1},
		{X: 10, Y: 10, Z: 1},
	}

	assertElementsMatch(t, expectedHull, convexHull(pointCloud).Vertices, "ConvexHull should be as expected")
}

// Simple 2D test (triangle, all points on a plane)
func TestConvexHull2DTriangle(t *testing.T) {
	// Construct a triangular as 'point cloud' that looks roughly like this.
	// All points are on a single plane (Z=1).
	// D should be inside the Convex Hull.
	//
	//         C
	//       /   \
	//     /   D   \
	//   /           \
	// A - - - - - - - B
	//
	pointCloud := []r3.Vector{
		{X: 1, Y: 2, Z: 1},
		{X: 4, Y: 7, Z: 1},
		{X: 7, Y: 2, Z: 1},
		{X: 4, Y: 4, Z: 1}, // This point is inside the hull
	}

	expectedHull := []r3.Vector{
		{X: 1, Y: 2, Z: 1},
		{X: 4, Y: 7, Z: 1},
		{X: 7, Y: 2, Z: 1},
	}

	assertElementsMatch(t, expectedHull, convexHull(pointCloud).Vertices, "ConvexHull should be as expected")
}

// Simple 3D test (one point in a box)
func TestConvexHull3D(t *testing.T) {
	// Construct a point cloud that looks roughly like this.
	// A should be inside the Convex Hull.
	/*
		@ + + + + + + + + + + + @
		+\                      +\
		+ \                     + \
		+  \                    +  \
		+   \                   +   \
		+    @ + + + + + + + + +++ + @
		+    +                  +    +
		+    +                  +    +
		+    +                  +    +
		+    +         A        +    +
		+    +                  +    +
		+    +                  +    +
		@ + +++ + + + + + + + + @    +
		 \   +                   \   +
		  \  +                    \  +
		   \ +                     \ +
		    \+                      \+
		     @ + + + + + + + + + + + @
	*/
	pointCloud := []r3.Vector{
		{X: 0, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 10},
		{X: 0, Y: 10, Z: 0},
		{X: 0, Y: 10, Z: 10},
		{X: 10, Y: 0, Z: 0},
		{X: 10, Y: 0, Z: 10},
		{X: 10, Y: 10, Z: 0},
		{X: 10, Y: 10, Z: 10},
		{X: 5, Y: 5, Z: 5},
	}

	actual := convexHull(pointCloud).Vertices

	expected := []r3.Vector{
		{X: 0, Y: 0, Z: 0},
		{X: 0, Y: 0, Z: 10},
		{X: 0, Y: 10, Z: 0},
		{X: 0, Y: 10, Z: 10},
		{X: 10, Y: 0, Z: 0},
		{X: 10, Y: 0, Z: 10},
		{X: 10, Y: 10, Z: 0},
		{X: 10, Y: 10, Z: 10},
	}

	assertEqual(t, 8, len(actual))
	assertElementsMatch(t, expected, actual, "ConvexHull should be as expected")
}

func TestHalfEdgeOutput(t *testing.T) {
	var pointCloud []r3.Vector

	for i := 0; i < 1000; i++ {
		pointCloud = append(pointCloud, r3.Vector{
			X: randF64(-1, 1),
			Y: randF64(-1, 1),
			Z: randF64(-1, 1),
		})
	}

	twoOrNegativeTwo := func(i, and int) float64 {
		if i&and > 0 {
			return -2
		}
		return 2
	}

	for i := 0; i < 8; i++ {
		pointCloud = append(pointCloud, r3.Vector{
			X: twoOrNegativeTwo(i, 1),
			Y: twoOrNegativeTwo(i, 2),
			Z: twoOrNegativeTwo(i, 4),
		})
	}

	mesh := new(QuickHull).ConvexHullAsMesh(pointCloud, 0)

	assertEqual(t, 12, len(mesh.Faces))
	assertEqual(t, 36, len(mesh.HalfEdges))
	assertEqual(t, 8, len(mesh.Vertices))
}

func randF64(min, max float64) float64 {
	return (max-min)*rand.Float64() + min
}

func TestPlanes(t *testing.T) {
	m := r3.Vector{X: 1, Y: 0, Z: 0}
	n := r3.Vector{X: 2, Y: 0, Z: 0}
	p := newPlane(m, n)

	dist := signedDistanceToPlane(r3.Vector{X: 3, Y: 0, Z: 0}, p)
	assertEqual(t, 1.0, dist)

	dist = signedDistanceToPlane(r3.Vector{X: 1, Y: 0, Z: 0}, p)
	assertEqual(t, -1.0, dist)

	m = r3.Vector{X: 2, Y: 0, Z: 0}
	p = newPlane(m, n)

	dist = signedDistanceToPlane(r3.Vector{X: 6, Y: 0, Z: 0}, p)

	assertEqual(t, 8.0, dist)
}

func convexHull(pointCloud []r3.Vector) ConvexHull {
	return new(QuickHull).ConvexHull(pointCloud, true, false, 0)
}
