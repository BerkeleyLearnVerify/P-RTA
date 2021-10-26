
width = 1.5
length = 1.5
workspace = Workspace(RectangularRegion(0 @ 0, 0, width, length))

# types of objects

class Robot:
	width: 0.01
	height: 0.01
	webotsType: 'Robot'

class Charger:
	width: 0.01
	height: 0.01
	webotsType: 'Charger'



ego = Charger at 0 @ 0


charger = Robot at OrientedPoint offset by (-0.50, 0.50) @ (-0.50, 0.50)

