import pyglet
from pyglet import gl
from Box2D import (
    b2Vec2, b2PolygonDef, b2World,
    b2BodyDef, b2AABB, b2MouseJointDef
)

FPS = 60

W = 100
H = 72

SCALE = 0.1    # World units - screen units conversion factor

world = None  # let's keep world as a global for now

# List of blocks that have been created
blocks = []


def screen_to_world(pos):
    sx, sy = pos
    return b2Vec2(sx * SCALE, sy * SCALE)


def world_to_screen(pos):
    wx, wy = pos
    return (wx / SCALE, wy / SCALE)


def sprite_scale(self):
    return 0.1 / SCALE


def setup_world():
    world_bounds = b2AABB()
    world_bounds.lowerBound = (-200, -1000)
    world_bounds.upperBound = (200, 200)
    world = b2World(
        world_bounds,
        b2Vec2(0, -30.0),  # Gravity vector
        True  # Use "sleep" optimisation
    )

    wallsdef = b2BodyDef()
    walls = world.CreateBody(wallsdef)
    walls.userData = 'Blocks'

    WALLS = [
        (W / 2, 1, (W / 2, -1), 0),  # floor
        (W / 2, 1, (W / 2, H + 1), 0),  # ceiling
        (1, 600, (-1, -500), 0),  # left wall
        (1, 600, (W + 1, -500), 0),  # right wall
    ]

    for wall in WALLS:
        shape = b2PolygonDef()
        shape.SetAsBox(*wall)
        walls.CreateShape(shape)

    return world


BOX_SIZE = 2
POINTS = [
    (-BOX_SIZE, BOX_SIZE),
    (BOX_SIZE, BOX_SIZE),
    (BOX_SIZE, -BOX_SIZE),
    (-BOX_SIZE, -BOX_SIZE),
]


def create_block(pos):
    bodydef = b2BodyDef()
    bodydef.position = pos
    body = world.CreateBody(bodydef)
    shape = b2PolygonDef()
    shape.SetAsBox(BOX_SIZE, BOX_SIZE, (0, 0), 0)
    shape.density = 0.1
    shape.restitution = 0.2
    shape.friction = 0.5

    body.CreateShape(shape)
    body.SetMassFromShapes()
    return body


def draw_block(block):
    transformed = [world_to_screen(block.GetWorldPoint(p)) for p in POINTS]
    gl.glColor3f(1.0, 0.1, 0)
    gl.glBegin(gl.GL_LINE_LOOP)
    for p in transformed:
        gl.glVertex2f(*p)
    gl.glEnd()


def on_mouse_press(x, y, button, modifiers):
    p = screen_to_world((x, y))
    blocks.append(
        create_block(p)
    )


def update(dt):
    world.Step(1.0 / FPS, 10, 8)


def on_draw():
    gl.glClear(gl.GL_COLOR_BUFFER_BIT)
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()

    for block in blocks:
        draw_block(block)


if __name__ == '__main__':
    world = setup_world()
    window = pyglet.window.Window(
        width=int(W / SCALE),
        height=int(H / SCALE)
    )
    window.event(on_mouse_press)
    window.event(on_draw)

    pyglet.clock.schedule(update)
    pyglet.app.run()
