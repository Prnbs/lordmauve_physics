import math
import pyglet
from pyglet import gl
from Box2D import (
    b2Vec2, b2PolygonDef, b2World,
    b2BodyDef, b2AABB, b2CircleDef,
    b2ContactListener
)

FPS = 60
TIMESTEP = 1.0 / FPS
W = 100
H = 72
FLOOR = 5

SCALE = 0.1    # World units - screen units conversion factor

world = None  # let's keep world as a global for now


def load_image_centered(filename):
    """Load an image and set its anchor point to the middle."""
    im = pyglet.image.load(filename)
    im.anchor_x = im.width // 2
    im.anchor_y = im.height // 2
    return im


# Load sprites
tank_body_image = load_image_centered('textures/tank-body.png')
tank_barrel_image = pyglet.image.load('textures/tank-barrel.png')
earth_tex = pyglet.image.load('textures/earth-tex.png').get_mipmapped_texture()

# Load sounds
muzzle_sound = pyglet.media.load('sounds/muzzle.wav', streaming=False)
explosion_sound = pyglet.media.load('sounds/explosion.wav', streaming=False)


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
        b2Vec2(0, -30),  # Gravity vector
        True  # Use "sleep" optimisation
    )

    wallsdef = b2BodyDef()
    walls = world.CreateBody(wallsdef)
    walls.userData = 'Blocks'

    WALLS = [
        (W, FLOOR * 0.5, (W / 2, FLOOR * 0.5), 0),  # floor
        #(W / 2, 1, (W / 2, H + 1), 0),  # ceiling
        #(1, 600, (-1, -500), 0),  # left wall
        #(1, 600, (W + 1, -500), 0),  # right wall
    ]

    for wall in WALLS:
        shape = b2PolygonDef()
        shape.SetAsBox(*wall)
        walls.CreateShape(shape)

    return world


class GraphicalObject(object):
    # Load image here
    IMAGE = None

    def __init__(self, pos):
        self.create_sprite(pos)

    def create_sprite(self, pos):
        x, y = world_to_screen(pos)
        self.sprite = pyglet.sprite.Sprite(self.IMAGE, x, y, batch=batch)

    def update(self, dt):
        pass

    def destroy(self):
        objects.remove(self)


class PhysicalObject(GraphicalObject):
    BULLET = False
    SHAPEDEF = None

    def __init__(self, pos):
        self.create_body(pos)
        super(PhysicalObject, self).__init__(pos)

    def create_body(self, pos):
        if self.SHAPEDEF is None:
            return

        bodydef = b2BodyDef()
        bodydef.position = b2Vec2(*pos)
        body = world.CreateBody(bodydef)
        body.CreateShape(self.SHAPEDEF)
        body.SetBullet(self.BULLET)
        body.SetMassFromShapes()
        self.body = body
        body.userData = self

    def update(self, dt):
        self.sprite.position = world_to_screen(self.body.position)
        self.sprite.rotation = -math.degrees(self.body.angle)

    def destroy(self):
        world.DestroyBody(self.body)
        objects.remove(self)


class Brick(PhysicalObject):
    """Just another brick in the wall."""
    W = 4.7
    H = 2
    IMAGE = load_image_centered('textures/brick.png')

    SHAPEDEF = b2PolygonDef()
    SHAPEDEF.SetAsBox(W * 0.5, H * 0.5, (0, 0), 0)
    SHAPEDEF.density = 0.5
    SHAPEDEF.restitution = 0.1
    SHAPEDEF.friction = 0.5


class HalfBrick(Brick):
    W = 2.2
    H = 2
    IMAGE = load_image_centered('textures/half-brick.png')

    SHAPEDEF = b2PolygonDef()
    SHAPEDEF.SetAsBox(W * 0.5, H * 0.5, (0, 0), 0)
    SHAPEDEF.density = 0.5
    SHAPEDEF.restitution = 0.1
    SHAPEDEF.friction = 0.5


class Cannonball(PhysicalObject):
    RADIUS = 0.8
    IMAGE = load_image_centered('textures/cannonball.png')

    SHAPEDEF = b2CircleDef()
    SHAPEDEF.radius = RADIUS
    SHAPEDEF.density = 1.0
    SHAPEDEF.restitution = 0.1
    SHAPEDEF.friction = 0.5

    exploding = False

    @classmethod
    def fire(self, pos, velocity):
        c = Cannonball(pos)
        c.body.SetLinearVelocity(b2Vec2(*velocity))
        objects.append(c)
        return c

    def explode(self):
        self.exploding = True
        explosion_sound.play()

    def update(self, dt):
        if self.exploding and self.body:
            self.destroy()
            objects.append(Explosion(self.body.position))
        else:
            super(Cannonball, self).update(dt)


class Explosion(PhysicalObject):
    IMAGE = load_image_centered('textures/explosion.png')
    ANGULAR_VELOCITY = 720
    MAX_AGE = 0.4
    RADIUS = 4
    MAX_FORCE = 100000
    FALLOFF = 2
    age = 0

    SHAPEDEF = b2CircleDef()
    SHAPEDEF.radius = RADIUS
    SHAPEDEF.isSensor = True

    def __init__(self, pos):
        super(Explosion, self).__init__(pos)
        self.affecting = set()

    def create_sprite(self, pos):
        super(Explosion, self).create_sprite(pos)
        self.sprite.scale = 0.5

    def update(self, dt):
        self.age += dt
        if self.age > self.MAX_AGE:
            self.destroy()
            return
        self.sprite.scale *= 500.0 ** dt  # grow
        self.sprite.rotation += self.ANGULAR_VELOCITY * dt  # spin
        self.sprite.opacity = 255.0 * (1.0 - self.age / self.MAX_AGE)  # fade
        pos = self.body.position
        for o in self.affecting:
            p = o.body.position
            v = p - pos
            l = v.Normalize()
            force = v * self.MAX_FORCE / (l ** self.FALLOFF + 1)
            o.body.ApplyForce(force, p)


class ContactListener(b2ContactListener):
    def Add(self, point):
        o1 = point.shape1.GetBody().userData
        o2 = point.shape2.GetBody().userData

        for o, a in zip((o1, o2), (o2, o1)):
            if isinstance(o, Cannonball) and not isinstance(a, Explosion):
                o.explode()
            if isinstance(o, Explosion):
                o.affecting.add(a)

    def Remove(self, point):
        o1 = point.shape1.GetBody().userData
        o2 = point.shape2.GetBody().userData
        for o, a in zip((o1, o2), (o2, o1)):
            if isinstance(o, Explosion):
                o.affecting.discard(a)


tank_pos = b2Vec2(10, FLOOR + 2.5)
barrel_angle = 0


def on_mouse_press(x, y, button, modifiers):
    """Fire in the hole!"""
    angle = math.radians(barrel_angle)
    v = b2Vec2(math.cos(angle), math.sin(angle))
    pos = tank_pos + b2Vec2(0, 2) + v * 3
    Cannonball.fire(pos, v * 100)
    muzzle_sound.play()


def clamp(val, minimum, maximum):
    return max(minimum, min(maximum, val))


def on_mouse_motion(x, y, dx, dy):
    global barrel_angle
    p = screen_to_world((x, y))
    dx, dy = p - tank_pos
    barrel_angle = math.degrees(math.atan2(dy, dx))
    barrel_angle = clamp(barrel_angle, 0, 80)


def on_key_press(symbol, modifiers):
    global slowmo
    if symbol == pyglet.window.key.S:
        slowmo = not slowmo


batch = None
tank = None
tank_barrel = None
objects = []
slowmo = False


def update(dt):
    rate = 0.2 if slowmo else 1.0
    world.Step(TIMESTEP * rate, 20, 16)
    tank_barrel.rotation = -barrel_angle
    for b in objects:
        b.update(dt * rate)


def build_wall(x, y, layers):
    MORTAR = 0.0
    for i in xrange(layers):
        cy = y + i * Brick.H
        if i % 2:
            objects.extend([
                HalfBrick((x - (HalfBrick.W + Brick.W) * 0.5 - MORTAR, cy)),
                Brick((x, cy)),
                HalfBrick((x + (HalfBrick.W + Brick.W) * 0.5 + MORTAR, cy)),
            ])
        else:
            objects.extend([
                Brick((x - Brick.W * 0.5 - MORTAR, cy)),
                Brick((x + Brick.W * 0.5 + MORTAR, cy)),
            ])


def setup_scene():
    global batch, tank, tank_barrel
    batch = pyglet.graphics.Batch()

    # Gradient sky
    l, b = world_to_screen((0, FLOOR))
    r, t = world_to_screen((W, H))
    horizon = 177 / 255.0, 202 / 255.0, 1.0
    zenith = 68 / 255.0, 0.5, 1.0
    batch.add(4, gl.GL_QUADS, None,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('c3f', sum([horizon, zenith, zenith, horizon], ())),
    )

    # Create the ground
    group = pyglet.sprite.SpriteGroup(
        earth_tex, gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA
    )
    l, b = world_to_screen((0, 0))
    r, t = world_to_screen((W, FLOOR))
    batch.add(4, gl.GL_QUADS, group,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('t2f', [0, 0.5, 0, 1, 10, 1, 10, 0.5]),
    )

    # Create tank barrel sprite
    tank_barrel = pyglet.sprite.Sprite(tank_barrel_image, batch=batch)
    tank_barrel.position = world_to_screen(tank_pos + b2Vec2(0, 1))

    # Create tank sprite
    tank = pyglet.sprite.Sprite(tank_body_image, batch=batch)
    tank.position = world_to_screen(tank_pos)

    build_wall(70, FLOOR, 20)


def on_draw():
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    batch.draw()


if __name__ == '__main__':
    world = setup_world()
    contact_listener = ContactListener()
    world.SetContactListener(contact_listener)
    setup_scene()

    # Warm up the wall physics
    for i in range(200):
        world.Step(0.01, 20, 16)
    # Then freeze the wall in place
    for o in objects:
        o.body.PutToSleep()

    window = pyglet.window.Window(
        width=int(W / SCALE),
        height=int(H / SCALE)
    )
    window.event(on_mouse_press)
    window.event(on_mouse_motion)
    window.event(on_draw)
    window.event(on_key_press)

    pyglet.clock.schedule(update)
    pyglet.app.run()
