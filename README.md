# quickhull-go

A 3D [Quickhull](https://en.wikipedia.org/wiki/Quickhull) implementation in Go.

Big thanks to [@akuukka](https://github.com/akuukka) for their public domain [C++ implementation](https://github.com/akuukka/quickhull) which was used as a reference.

This is different from Google's S2 Geometry implementation because it works in ℝ³ instead of S².

[![GoDoc](https://godoc.org/github.com/markus-wa/quickhull-go?status.svg)](https://godoc.org/github.com/markus-wa/quickhull-go)
[![Build Status](https://travis-ci.org/markus-wa/quickhull-go.svg?branch=master)](https://travis-ci.org/markus-wa/quickhull-go)
[![codecov](https://codecov.io/gh/markus-wa/quickhull-go/branch/master/graph/badge.svg)](https://codecov.io/gh/markus-wa/quickhull-go)
[![Go Report](https://goreportcard.com/badge/github.com/markus-wa/quickhull-go)](https://goreportcard.com/report/github.com/markus-wa/quickhull-go)
[![License](https://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE.md)
[![FOSSA Status](https://app.fossa.io/api/projects/git%2Bgithub.com%2Fmarkus-wa%2Fquickhull-go.svg?type=shield)](https://app.fossa.io/projects/git%2Bgithub.com%2Fmarkus-wa%2Fquickhull-go?ref=badge_shield)

## Go Get

	go get -u github.com/markus-wa/quickhull-go

## Example

Example with a simple, planar point-cloud.

```go
package main

import (
	"fmt"

	r3 "github.com/golang/geo/r3"
	quickhull "github.com/markus-wa/quickhull-go"
)

func main() {
	pointCloud := []r3.Vector{
		{X: 1, Y: 2, Z: 1},
		{X: 4, Y: 7, Z: 1},
		{X: 7, Y: 2, Z: 1},
		{X: 4, Y: 4, Z: 1}, // This point is inside the hull
	}

	hull := new(QuickHull).ConvexHull(pointCloud, true, false, 0)

	fmt.Println(hull.Vertices) // does not contain (4,4,1)
	fmt.Println(hull.Triangles()) // triangles that make up the convex hull - [][3]r3.Vector, where each vector is a corner of the triangle
}
```


## License

This project is licensed under the [MIT license](LICENSE.md).

[![FOSSA Status](https://app.fossa.io/api/projects/git%2Bgithub.com%2Fmarkus-wa%2Fquickhull-go.svg?type=large)](https://app.fossa.io/projects/git%2Bgithub.com%2Fmarkus-wa%2Fquickhull-go?ref=badge_large)
