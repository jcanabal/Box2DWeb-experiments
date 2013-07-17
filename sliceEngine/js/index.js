// Declare all the commonly used objects as variables
var b2Vec2 = Box2D.Common.Math.b2Vec2;
var b2BodyDef = Box2D.Dynamics.b2BodyDef;
var b2Body = Box2D.Dynamics.b2FixtureDef;
var b2Body = Box2D.Dynamics.b2Body;
var b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
var b2Fixture = Box2D.Dynamics.b2Fixture;
var b2World = Box2D.Dynamics.b2World;
var b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
var b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef;
// Segment
var b2Segment = Box2D.Collision.b2Segment;

var ctx;
var world;
var canvas;
var scale = 30; //30 pixels on our canvas is equal 1m in box2D

var timeStep = 1/60;
var velocityIterations = 8;
var positionIterations = 3;

var bodies;
var drawing = false;
var laser;
var laserSegment;
var entryPoints;
var affectedByLaser;

function init(){
    canvas = document.getElementById('id-canvas');
	
	canvas.onselectstart = function () { return false; }
	canvas.addEventListener('mousedown', function(e){mouseDown(e)}, false);
	canvas.addEventListener('mousemove', function(e){mouseMove(e)}, false);
	canvas.addEventListener('mouseup', function(e){mouseUp(e)}, false);
	
	document.getElementById('reset-button').addEventListener('click', reset , false);
	
	/* set up the box2D world than will do
	the most physics calculations */
	var gravity = new b2Vec2(0, 9.8);
	var allowSleep = true;
	
	world = new b2World(gravity, allowSleep);
	
    bodies = [];
	createFloor();
	createRegularPolygon(150, 150, 90, 6, b2Body.b2_staticBody);
	createRegularPolygon(450, 150, 90, 16, b2Body.b2_staticBody);
	setupDebugDraw();
	
	update();
}

function createFloor(){
	
	/* A body definition holds all the data
	needed to construct a rigid body */
	var bodyDef = new b2BodyDef();
	bodyDef.type = b2Body.b2_staticBody;
	bodyDef.position.x = (canvas.width/2)/scale;
	bodyDef.position.y = (canvas.height - 15)/scale;
	
	/* A fixture is used to attach a shape to a body
	for collision detection */
	var fixtureDef = new b2FixtureDef();
	fixtureDef.density = 1.0;
	fixtureDef.friction = 0.5;
	fixtureDef.restitution = 0.0;
	
	fixtureDef.shape = new b2PolygonShape();
	// canvas width wide and 30px tall
	fixtureDef.shape.SetAsBox((canvas.width/2)/scale, 15/scale);
	
	var body = world.CreateBody(bodyDef);
	var fixture = body.CreateFixture(fixtureDef);
}

function createRegularPolygon(px, py, radius, sides, type){
	var polygonVector = [];
	radius /= scale;
	for(var i=0; i<sides; i++){
		var x = radius * Math.cos(2*Math.PI/sides * i);
		var y = radius * Math.sin(2*Math.PI/sides * i);
		polygonVector.push(new b2Vec2(x,y));
	}
	
	var bodyDef = new b2BodyDef();
	bodyDef.type = type;
	bodyDef.position.x = px/scale;
	bodyDef.position.y = py/scale;
	
	var fixtureDef = new b2FixtureDef();
	fixtureDef.density = 1.0;
	fixtureDef.friction = 0.5;
	fixtureDef.restitution = 0.0;
	fixtureDef.shape = new b2PolygonShape();
	fixtureDef.shape.SetAsVector(polygonVector, sides);
	
	var body = world.CreateBody(bodyDef);
	var fixture = body.CreateFixture(fixtureDef);
    
    bodies.push(body);
} 

function createComplexBody(px, py, type){
}

/* Set up the debug draw of our box2D world */
function setupDebugDraw(){
    if(ctx == null) ctx = canvas.getContext('2d');
    
    var debugDraw = new b2DebugDraw();
    
    // Use the canvas context for drawing the debugging screen
    debugDraw.SetSprite(ctx);
    // Set the scale
    debugDraw.SetDrawScale(scale);
    // Set the alpha transparency for boxes
    debugDraw.SetFillAlpha(0.5);
    // Draw lines with a  thickness of 1.0
    debugDraw.SetLineThickness(1.0);
    // Display all shapes and joints
    debugDraw.SetFlags(
        b2DebugDraw.e_shapeBit |
		// b2DebugDraw.e_centerOfMassBit |
        b2DebugDraw.e_jointBit 
    );
    
    // Start using the debug draw in the world
    world.SetDebugDraw(debugDraw);
}

function update(){
	world.Step(
		timeStep,
		velocityIterations,
		positionIterations
	);
	
	world.ClearForces();
	world.DrawDebugData();
	
	if(laser && drawing) drawLaser();
	// If there is a laser segment and we finished drawing it
	if(laserSegment && !drawing){
		affectedByLaser = [];
		entryPoints = [];
		world.RayCast(laserFired, laserSegment.p1, laserSegment.p2);
		world.RayCast(laserFired, laserSegment.p2, laserSegment.p1);
		laserSegment=null;
	}
	
	requestAnimationFrame(update, canvas);
}

// callback function that executes when we finish drawing the laser
function laserFired(fixture, point, normal, fraction){
	
	var affectedBody = fixture.GetBody();
	var affectedPolygon = fixture.GetShape();
	var fixtureIndex = affectedByLaser.indexOf(affectedBody);
	if(fixtureIndex == -1){
		affectedByLaser.push(affectedBody);
		entryPoints.push(point);
	}else{
		var entryPoint = entryPoints[fixtureIndex];
		var rayCenter = new b2Vec2((point.x + entryPoint.x)/2, (point.y + entryPoint.y)/2);
		var rayAngle = Math.atan2(entryPoint.y - point.y, entryPoint.x - point.x);
		var polyVertices = affectedPolygon.GetVertices();
		drawCircle(entryPoint, 'white');
		drawCircle(rayCenter, 'aqua');
		drawCircle(point, 'white');
		
		// Polygon information
		var currentPoly = 0;
		var newPolyVertices1 = [];
		var newPolyVertices2 = [];
		var cutPlaced1 = false;
		var cutPlaced2 = false;
		
		for(var i=0, len=polyVertices.length; i<len; i++){
			var worldPoint = affectedBody.GetWorldPoint(polyVertices[i]);
			var cutAngle = Math.atan2(worldPoint.y - rayCenter.y, worldPoint.x - rayCenter.x) - rayAngle;
			cutAngle = (cutAngle < -1*Math.PI)? (cutAngle + 2*Math.PI):cutAngle;
			if(cutAngle > 0 && cutAngle <= Math.PI){
				if(currentPoly == 2){
					cutPlaced1 = true;
					newPolyVertices1.push(point);
					newPolyVertices1.push(entryPoint);
				}
				newPolyVertices1.push(worldPoint);
				currentPoly = 1;
				//drawCircle(worldPoint, 'white');
			}else{
				if(currentPoly == 1){
					cutPlaced2 = true;
					// We add the points in reverse order
					newPolyVertices2.push(entryPoint);
					newPolyVertices2.push(point);	
				}
				newPolyVertices2.push(worldPoint);
				currentPoly = 2;
				//drawCircle(worldPoint, 'orangered');
			}
		}
		// Add the entry and exit point in each polygon
		if(!cutPlaced1){
			newPolyVertices1.push(point);
			newPolyVertices1.push(entryPoint);
		}
		if(!cutPlaced2){
			newPolyVertices2.push(entryPoint);
			newPolyVertices2.push(point);
		}
		// Create the slices and destroy the affected body
		createSlice(newPolyVertices1, fixture);
		createSlice(newPolyVertices2, fixture);
		world.DestroyBody(affectedBody);
	}
	return 1;
}

function findCentroid(vertices){
	var count = vertices.length;
	var c = new b2Vec2();
    var area = 0.0;
	var p1X = 0.0;
	var p1Y = 0.0;
	var inv3 = 1.0/3.0;
	for (var i= 0; i < count; ++i) {
		var p2 = vertices[i];
		var p3 = (i+1<count)? vertices[i+1]:vertices[0];
		var e1X = p2.x-p1X;
		var e1Y = p2.y-p1Y;
		var e2X = p3.x-p1X;
		var e2Y = p3.y-p1Y;
		var D = (e1X * e2Y - e1Y * e2X);
		var triangleArea=0.5*D;
		area += triangleArea;
		c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
		c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
	}
	c.x *= 1.0/area;
	c.y *= 1.0/area;
	return c;
}

function createSlice(vertices, affectedFixture){
	var numVertices = vertices.length;
	var centre = findCentroid(vertices);
	
	for(var i=0; i<numVertices; i++){
		vertices[i].Subtract(centre);
	}
	
	var sliceBody = new b2BodyDef();
	sliceBody.position.x = centre.x;
	sliceBody.position.y = centre.y;
	sliceBody.type = b2Body.b2_dynamicBody;
	
	var sliceFixture = new b2FixtureDef();
	sliceFixture.shape = new b2PolygonShape();
	sliceFixture.shape.SetAsVector(vertices, numVertices);
	sliceFixture.density = 1.0;
	
	var body = world.CreateBody(sliceBody);
	var fixture = body.CreateFixture(sliceFixture);
	
	for(var i=0; i<numVertices; i++){
		vertices[i].Add(centre);
	}
    
    bodies.push(body);
}

function drawCircle(point, color){
	ctx.beginPath();
	ctx.arc(point.x*scale, point.y*scale, 5, 0, 2*Math.PI, false);
	ctx.strokeStyle = color;
	ctx.stroke();
}

function mouseDown(e){

	drawing = true;
	mouse = relMouseCoords(e);
	
	laserSegment = new b2Segment();
	laserSegment.p1 = new b2Vec2(mouse.x/scale, mouse.y/scale);
	laser = {p1:{x:mouse.x, y:mouse.y}, p2:{x:mouse.x, y:mouse.y}}
}

function mouseMove(e){
	if(drawing){
		if(drawing)	mouse = relMouseCoords(e);
		laser.p2.x = mouse.x;
		laser.p2.y = mouse.y;
	}
}

function mouseUp(e){
	drawing = false;
	mouse = relMouseCoords(e);
	laserSegment.p2 = new b2Vec2(mouse.x/scale, mouse.y/scale);
}

function drawLaser(){
	ctx.beginPath();
	ctx.moveTo(laser.p1.x,laser.p1.y);
	ctx.lineTo(laser.p2.x, laser.p2.y);
		
	ctx.strokeStyle = 'white';
	ctx.stroke();
}

function reset(){
    for(var i=0, len=bodies.length; i<len; i++){
        world.DestroyBody(bodies[i]);
        bodies[i] = null;
    }
    
    bodies = [];
    createRegularPolygon(150, 150, 90, 6, b2Body.b2_staticBody);
	createRegularPolygon(450, 150, 90, 16, b2Body.b2_staticBody);
}

/* Get mouse coords */
function relMouseCoords(e){
    var rect = canvas.getBoundingClientRect();
    return {
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
    };
}

window.onload = init;

/* Cross browser requestAnimationFrame and
cancelAnimationFrame implementations */
(function(){
	var handler;
	window.requestAnimationFrame = (function(){
		return  window.requestAnimationFrame       || 
				window.webkitRequestAnimationFrame || 
				window.mozRequestAnimationFrame    || 
				window.oRequestAnimationFrame      || 
				window.msRequestAnimationFrame     || 
				function(callback, element){
					handler = window.setTimeout(callback, 1000 / 60);
				};
	})();

	window.cancelAnimationFrame = (function(){
		return  window.cancelAnimationFrame       || 
				window.webkitCancelAnimationFrame || 
				window.mozCancelAnimationFrame    || 
				window.oCancelAnimationFrame      || 
				window.msCancelAnimationFrame     || 
				clearTimeout(handler)
	})();
}());
