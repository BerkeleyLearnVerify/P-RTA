
width = 2.5
length = 2.5
workspace = Workspace(RectangularRegion(0 @ 0, 0, width, length))

class Robot:
	width: 0.01
	height: 0.01
	webotsType: 'Robot'

class Slave:
	width: 0.01
	height: 0.01
	webotsType: 'Slave'

class Box:
	width: 0.01
	height: 0.01
	webotsType: 'Box'

class Charger:
	width: 0.01
	height: 0.01
	webotsType: 'Charger'

ego = Robot

Charger at OrientedPoint offset by (-0.75, 0.75) @ (-0.75, 0.75)

Slave at OrientedPoint offset by (-1.0, 1.0) @ (-1.0, 1.0)

Slave at OrientedPoint offset by (-1.0, 1.0) @ (-1.0, 1.0)

Box at OrientedPoint offset by (-1.0, 1.0) @ (-1.0, 1.0)

Box at OrientedPoint offset by (-1.0, 1.0) @ (-1.0, 1.0)

Box at OrientedPoint offset by (-1.0, 1.0) @ (-1.0, 1.0)

