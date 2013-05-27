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
//Declare the bouyancy controller
var b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController

var ctx;
var world;
var canvas;
var scale = 30; //30 pixels on our canvas is equal 1m in box2D

var timeStep = 1/60;
var velocityIterations = 8;
var positionIterations = 3;

var balls = [];
var waterSensor;
var buoyancyController;

var fixtureProperties = {
	density: 1.0,
	friction: 0.5,
	restitution: 0.3
}

function init(){
    canvas = document.getElementById('id-canvas');
	
	canvas.onselectstart = function () { return false; }
	canvas.addEventListener('click', function(e){createNewBody(e);}, false);
	
	// set up the input range listeners
	
	/*document.getElementById('density-input').onchange = function(){fixtureProperties.density = this.value/100;}
    	document.getElementById('friction-input').onchange = function(){fixtureProperties.friction = this.value/100;}
	document.getElementById('restitution-input').onchange = function(){fixtureProperties.restitution = this.value/100;}*/
	
	document.getElementById('clear-button').addEventListener('click', function(){
		for(var i=0, len = balls.length; i<len; i++){
			world.DestroyBody(balls[i]);
			balls[i] = null;
		}
		balls = [];
	}, false);
	
	/* set up the box2D world than will do
	the most physics calculations */
	var gravity = new b2Vec2(0, 9.8);
	var allowSleep = true;
	
	world = new b2World(gravity, allowSleep);
	
	createFloor();
	createPool();
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

function createPool(){
	var width = canvas.width;
	var height = canvas.height;
	// The limits of the pool
	createBoxBody(150, height-170, 20, 240, b2Body.b2_staticBody, false);
	createBoxBody(width/2, height-40, canvas.width - 280, 20, b2Body.b2_staticBody, false);
	createBoxBody(width-150, height-170, 20, 240, b2Body.b2_staticBody, false);
	// Set up the buoyancy controller
	buoyancyController = new b2BuoyancyController();
	buoyancyController.normal.Set(0, -1);
	buoyancyController.offset = -200/scale;
	buoyancyController.useDensity = true;
	buoyancyController.density = 2.0;
	buoyancyController.linearDrag = 5;
    buoyancyController.angularDrag = 2;
	// Add the controller to the world
	world.AddController(buoyancyController);
	
	// The water sensor
	waterSensor = createBoxBody(width/2, height-150, canvas.width - 320, 200, b2Body.b2_staticBody, true);
}

function createBoxBody(px, py, width, height, bodyType, isSensor){
    var bodyDef = new b2BodyDef();
    bodyDef.type = bodyType;
    bodyDef.position.x = px/scale;
    bodyDef.position.y = py/scale;
    
    var fixtureDef = new b2FixtureDef();
	fixtureDef.isSensor = isSensor;
    fixtureDef.density = fixtureProperties.density;
    fixtureDef.friction = fixtureProperties.friction;
    fixtureDef.restitution = fixtureProperties.restitution;
	
    fixtureDef.shape = new b2PolygonShape();
    fixtureDef.shape.SetAsBox(width/2/scale, height/2/scale);
    var body = world.CreateBody(bodyDef);
    var fixture = body.CreateFixture(fixtureDef);
	
	return body;
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
	//writeFixtureInfo();
	requestAnimationFrame(update, canvas);
}

/* Fixture properties info text */
/*function writeFixtureInfo(){
	ctx.font = '12pt Helvetica';
	ctx.fillStyle = 'white';
	ctx.fillText("Fixture info", 10, 25);
	ctx.font = '10pt Helvetica';
	ctx.fillText("density: " + fixtureProperties.density, 10, 50);
	ctx.fillText("friction: " + fixtureProperties.friction, 10, 70);
	ctx.fillText("restitution: " + fixtureProperties.restitution, 10, 90);
}*/

/* Create a new box body with a random width
and height between 10 and 60 in the mouse coords*/
function createNewBody(e){
	var mouseCoords = relMouseCoords(e);
	var body = createBoxBody(mouseCoords.x, mouseCoords.y, Math.random() * 50 + 10, Math.random() * 50 + 10, b2Body.b2_dynamicBody, false);
	balls.push(body);
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
