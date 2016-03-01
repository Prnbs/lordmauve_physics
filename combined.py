import math
import pyglet
from pyglet import gl
from pyglet.window import key
from Box2D import (
    b2Vec2, b2PolygonDef, b2World,
    b2BodyDef, b2AABB, b2CircleDef,
    b2MouseJointDef, b2RevoluteJointDef, b2ContactListener,
    b2DistanceJointDef, b2BuoyancyControllerDef
)
from xml.etree.ElementTree import parse

FPS = 60
TIMESTEP = 1.0 / FPS
W = 100
H = 72
FLOOR = 15
WATER_LEVEL = 3


SCALE = 0.1    # World units - screen units conversion factor

world = None  # let's keep world as a global for now
mouse_joint = None


def load_image_centered(filename):
    """Load an image and set its anchor point to the middle."""
    im = pyglet.image.load(filename)
    im.anchor_x = im.width // 2
    im.anchor_y = im.height // 2
    return im


# Load sprites
heli_image = load_image_centered('textures/heli.png')
earth_tex = pyglet.image.load('textures/ground.png')


def screen_to_world(pos):
    sx, sy = pos
    return b2Vec2(sx * SCALE, sy * SCALE) + camera - b2Vec2(W * 0.5, H * 0.5)


def world_to_screen(pos):
    wx, wy = pos
    return (wx / SCALE, wy / SCALE)


def sprite_scale(self):
    return 0.1 / SCALE


def setup_world():
    global buoyancy
    world_bounds = b2AABB()
    world_bounds.lowerBound = (-200, -100)
    world_bounds.upperBound = (1000, 1000)
    world = b2World(
        world_bounds,
        b2Vec2(0, -30),  # Gravity vector
        True  # Use "sleep" optimisation
    )

    wallsdef = b2BodyDef()
    walls = world.CreateBody(wallsdef)
    walls.userData = 'Blocks'

    WALLS = [
        #(W, FLOOR * 0.5, (W / 2, FLOOR * 0.5), 0),  # floor
        #(W / 2, 1, (W / 2, H + 1), 0),  # ceiling
        (1, 600, (-1, -500), 0),  # left wall
        #(1, 600, (W + 1, -500), 0),  # right wall
    ]

    for wall in WALLS:
        shape = b2PolygonDef()
        shape.SetAsBox(*wall)
        walls.CreateShape(shape)

    for shape in read_shapes_from_svg('shapes/ground.svg'):
        walls.CreateShape(shape)

    buoyancydef = b2BuoyancyControllerDef()
    buoyancydef.normal = b2Vec2(0, 1)
    buoyancydef.offset = WATER_LEVEL
    buoyancydef.density = 2.5
    buoyancydef.angularDrag = 0.5
    buoyancydef.linearDrag = 3
    buoyancy = world.CreateController(buoyancydef)

    return world


def read_shapes_from_svg(fname):
    """Dirty way to read polygons from a subset of SVG."""
    doc = parse(fname)
    shapes = []

    h = float(doc.getroot().get('height'))
    for p in doc.findall('.//{http://www.w3.org/2000/svg}path'):
        path = p.get('d') or ''
        coords = []
        last = b2Vec2(0, h)
        for cmd in path.split():
            if ',' in cmd:
                x, y = [float(c) for c in cmd.split(',')]
                c = b2Vec2(x, -y) + last
                last = c
                coords.append(c)
        shape = b2PolygonDef()
        shape.setVertices(tuple(coords))
        shapes.append(shape)
    return shapes


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


def box_def(w, h,
        center=(0, 0),
        angle=0,
        density=1,
        restitution=0.1,
        friction=2,
        groupindex=0):
    s = b2PolygonDef()
    s.SetAsBox(w * 0.5, h * 0.5, b2Vec2(*center), angle)
    s.density = density
    s.restitution = restitution
    s.friction = friction
    s.filter.groupIndex = groupindex
    return s


def circle_def(radius,
        center=(0, 0),
        density=1,
        restitution=0.1,
        friction=2,
        groupindex=0):
    s = b2CircleDef()
    s.radius = radius
    s.localPosition = b2Vec2(*center)
    s.density = density
    s.restitution = restitution
    s.friction = friction
    s.filter.groupIndex = groupindex
    return s


class PhysicalObject(GraphicalObject):
    BULLET = False
    SHAPEDEFS = []

    def __init__(self, pos):
        super(PhysicalObject, self).__init__(pos)
        self.create_body(pos)

    def create_body(self, pos):
        if not self.SHAPEDEFS:
            return

        bodydef = b2BodyDef()
        bodydef.position = b2Vec2(*pos)
        body = world.CreateBody(bodydef)
        for shape in self.SHAPEDEFS:
            body.CreateShape(shape)
        body.SetBullet(self.BULLET)
        body.SetMassFromShapes()
        self.body = body
        buoyancy.AddBody(body)
        body.userData = self

    def update(self, dt):
        self.sprite.position = world_to_screen(self.body.position)
        self.sprite.rotation = -math.degrees(self.body.angle)

    def destroy(self):
        world.DestroyBody(self.body)
        objects.remove(self)


class Crate(PhysicalObject):
    IMAGE = load_image_centered('textures/crate.png')
    SHAPEDEFS = [
        box_def(4, 4, density=2)
    ]


class Hook(PhysicalObject):
    IMAGE = load_image_centered('textures/hook.png')
    SHAPEDEFS = [
        box_def(1.3, 0.8, density=3, groupindex=-1)
    ]

    joint = None
    to_pick_up = None

    def set_pick_up(self, body, pos):
        """Schedule a pick up.

        Creating the joint can't happen in the collision listener
        so wait until update() to call it.

        """
        if self.joint:
            return
        self.to_pick_up = (body, pos)

    def cancel_pick_up(self):
        self.to_pick_up = None

    def pick_up(self, body, pos):
        if self.joint:
            return
#        jd = b2DistanceJointDef()
#        jd.Initialize(self.body, body, b2Vec2(0.65, 0), b2Vec2(0, 4))
#        jd.length = 0.5

        jd = b2RevoluteJointDef()
        jd.Initialize(self.body, body, self.body.position)
        self.joint = world.CreateJoint(jd).getAsType()

    def drop(self):
        if self.joint:
            world.DestroyJoint(self.joint)
            self.joint = None

    def update(self, dt):
        if self.to_pick_up:
            self.pick_up(*self.to_pick_up)
            self.to_pick_up = None
        super(Hook, self).update(dt)


class Heli(PhysicalObject):
    IMAGE = load_image_centered('textures/heli.png')
    SHAPEDEFS = [
        box_def(14, 7, groupindex=-1),  # FIXME: more accurate shapedef
    ]

    ROTOR_ACCEL = 100
    ROTOR_DECEL = 0.2
    LIFT = b2Vec2(0, 150)
    ANGULAR_DAMPING = 10
    ANGLE_DAMPING = 10
    MAX_TORQUE = 5000

    rotoraccel = 0
    pitchaccel = 0
    rotorspeed = 0
    pitch = 0

    def up(self):
        self.rotoraccel = 1

    def left(self):
        self.pitchaccel = 1

    def right(self):
        self.pitchaccel = -1

    def update(self, dt):
        self.rotorspeed += self.rotoraccel * self.ROTOR_ACCEL ** dt
        self.rotorspeed *= self.ROTOR_DECEL ** dt
        self.rotoraccel = 0
        self.body.ApplyForce(
            self.body.GetWorldVector(self.rotorspeed * self.LIFT),  # force
            self.body.GetWorldPoint(b2Vec2(0, 3.5))  # position
        )
        self.pitchaccel += -self.ANGULAR_DAMPING * self.body.angularVelocity
        self.pitchaccel += -self.ANGLE_DAMPING * self.body.angle
        self.body.ApplyTorque(
            self.pitchaccel * self.MAX_TORQUE
        )
        self.pitchaccel = 0
        super(Heli, self).update(dt)


class Cable(PhysicalObject):
    length = 5    # length of the cable, in segments
    radius = 0.3  # radius of the cable
    PIECE_LENGTH = 1  # half the length of each segment of the cable

    TEXTURE = pyglet.image.load('textures/rope-tex.png').texture

    density = 0.3

    def create_left_end(self, pos):
        """Subclasses should implement this method to return an end object
        or None if no end should be connected.
        """
        return None

    def create_right_end(self, pos):
        """Subclasses should implement this method to return an end object
        or None if no end should be connected.
        """
        return Hook(b2Vec2(*pos) + b2Vec2(0.6, 0))

    def create_body(self, pos):
        x, y = pos
        self.lend = self.create_left_end(pos)
        self.segments = []
        self.ends = []

        if self.lend:
            prevBody = self.lend.body
            self.ends.append(self.lend)
        else:
            prevBody = None

        for i in range(self.length):
            sd = b2PolygonDef()
            sd.SetAsBox(self.PIECE_LENGTH, self.radius)
            sd.density = self.density
            sd.friction = 1
            sd.restitution = 0
            sd.filter.groupIndex = -1

            bd = b2BodyDef()
            bd.linearDamping = 0
            bd.angularDamping = 0.2
            bd.position = (x + self.PIECE_LENGTH * 2 * i - self.PIECE_LENGTH, y)
            body = world.CreateBody(bd)
            body.CreateShape(sd)
            body.SetMassFromShapes()
            self.segments.append(body)

            if prevBody:
                jd = b2RevoluteJointDef()
                anchor = (x + self.PIECE_LENGTH * (2 * i - 1), y)
                jd.Initialize(prevBody, body, anchor)
                world.CreateJoint(jd).getAsType()

            prevBody = body

        self.rend = self.create_right_end((x + self.PIECE_LENGTH * (self.length - 1) * 2, y))
        if self.rend:
            jd = b2RevoluteJointDef()
            anchor = (x + self.PIECE_LENGTH * (self.length - 1) * 2, y)
            jd.Initialize(prevBody, self.rend.body, anchor)
            world.CreateJoint(jd).getAsType()
            self.ends.append(self.rend)

    def destroy(self):
        super(Cable, self).destroy()
        for s in self.segments:
            world.DestroyBody(s)

    def update(self, dt):
        self.recompute_vertices()
        for e in self.ends:
            e.update(dt)

    def recompute_vertices(self):
        points = []
        points.append(self.segments[0].GetWorldPoint((-self.PIECE_LENGTH, 0)))
        for p in self.segments:
            points.append(p.GetWorldPoint((self.PIECE_LENGTH, 0)))

        if len(points) < 2:
            return

        v = None  # last segment vector
        verts = []
        for p1, p2 in zip(points, points[1:] + [None]):
            if p2:
                v2 = p2 - p1
                v2.Normalize()
            else:
                v2 = v
            if v is not None:
                sx, sy = (v2 + v) * 0.5
            else:
                sx, sy = v2
            v = v2
            off = b2Vec2(-sy, sx) * self.radius
            verts.extend(world_to_screen(p1 + off))
            verts.extend(world_to_screen(p1 - off))
        self.vlist.vertices = verts

    def create_sprite(self, pos):
        """Not really a sprite; create a vertex list."""
        group = pyglet.sprite.SpriteGroup(
            self.TEXTURE, gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA
        )
        texcoords = []
        for i in xrange(self.length + 1):
            texcoords.extend([
                self.TEXTURE.tex_coords[0], i,
                self.TEXTURE.tex_coords[3], i,
            ])
        count = 2 * (self.length + 1)
        verts = [0, 0] * count  # set vertices later from body
        self.vlist = batch.add(
            count, gl.GL_TRIANGLE_STRIP, group,
            ('v2f', verts),
            ('t2f', texcoords)
        )


class Wheel(PhysicalObject):
    IMAGE = load_image_centered('textures/wheel.png')
    SHAPEDEFS = [
        circle_def(1.6, friction=50.0)
    ]


class Truck(PhysicalObject):
    IMAGE = load_image_centered('textures/truck.png')
    SHAPEDEFS = [
        box_def(5.3, 6.1, (6.0, -0.4)),
        box_def(17.3, 1.4, (0.0, -2.7), friction=0.5),
        box_def(0.8, 1.4, (-8.4, -1.3)),
    ]

    WHEEL_POSITIONS = [
        b2Vec2(-6.8, -4),
        b2Vec2(-3.2, -4),
        b2Vec2(6.3, -4),
    ]

    # Angular acceleration
    ACCEL = 20
    motoraccel = 0
    motorspeed = 0

    def create_body(self, pos):
        pos = b2Vec2(*pos)
        super(Truck, self).create_body(pos)
        self.wheels = []
        self.joints = []
        for i, p in enumerate(self.WHEEL_POSITIONS):
            p = pos + p
            w = Wheel(p)
            jdef = b2RevoluteJointDef()
            jdef.enableMotor = i < 2
            jdef.motorSpeed = 0
            jdef.maxMotorTorque = 6000
            jdef.Initialize(self.body, w.body, p)
            j = world.CreateJoint(jdef).asRevoluteJoint()
            self.joints.append(j)
            self.wheels.append(w)

    def left(self):
        self.motoraccel = 1

    def right(self):
        self.motoraccel = -1

    def update(self, dt):
        self.motorspeed += self.motoraccel * self.ACCEL ** dt
        self.motorspeed *= 0.01 ** dt
        self.motoraccel = 0
        for j in self.joints[:2]:
            j.SetMotorSpeed(self.motorspeed)
        super(Truck, self).update(dt)
        for w in self.wheels:
            w.update(dt)

    def destroy(self):
        super(Truck, self).destroy()
        for w in self.wheels:
            w.destroy()
        self.wheels[:] = []
        self.joints[:] = []


class ContactListener(b2ContactListener):
    def Add(self, point):
        o1 = point.shape1.GetBody().userData
        o2 = point.shape2.GetBody().userData

        for o, a in zip((o1, o2), (o2, o1)):
            if isinstance(o, Hook) and isinstance(a, Crate):
                o.set_pick_up(a.body, point.position)

    def Remove(self, point):
        o1 = point.shape1.GetBody().userData
        o2 = point.shape2.GetBody().userData
        for o, a in zip((o1, o2), (o2, o1)):
            if isinstance(o, Hook) and isinstance(a, Crate):
                o.cancel_pick_up()


def clamp(val, minimum, maximum):
    return max(minimum, min(maximum, val))


def on_mouse_press(x, y, button, modifiers):
    global mouse_joint
    if mouse_joint:
        return

    p = screen_to_world((x, y))

    # Create a mouse joint on the selected body (assuming it's dynamic)

    # Make a small box.
    aabb = b2AABB()
    aabb.lowerBound = p - (0.001, 0.001)
    aabb.upperBound = p + (0.001, 0.001)

    # Query the world for overlapping shapes.
    body = None
    k_maxCount = 10  # maximum amount of shapes to return

    (count, shapes) = world.Query(aabb, k_maxCount)
    for shape in shapes:
        shapeBody = shape.GetBody()
        if not shapeBody.IsStatic() and shapeBody.GetMass() > 0.0:
            if shape.TestPoint(shapeBody.GetXForm(), p):  # is it inside?
                body = shapeBody
                break

    if body:
        print "Block clicked"
        # A body was selected, create the mouse joint
        md = b2MouseJointDef()
        md.body1 = world.GetGroundBody()
        md.body2 = body
        md.target = p
        md.maxForce = 100000.0
        mouse_joint = world.CreateJoint(md).getAsType()
        body.WakeUp()


def on_mouse_release(x, y, button, modifiers):
    global mouse_joint
    if mouse_joint:
        world.DestroyJoint(mouse_joint)
        mouse_joint = None


def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    global mouse_joint
    if mouse_joint:
        p = screen_to_world((x, y))
        mouse_joint.SetTarget(p)


batch = None
heli = None
truck = None
hook = None
objects = []
ground = None

controlling = None

camera = b2Vec2(W * 0.5, H * 0.5)


def track_controlled(dt):
    global camera
    if controlling and controlling.body:
        p = controlling.body.position
        cx, cy = camera + (p - camera) * (1.0 - 0.1 ** dt)
        cx = max(cx, W * 0.5)
        cy = max(cy, H * 0.5)
        camera = b2Vec2(cx, cy)


def update(dt):
    world.Step(TIMESTEP, 20, 16)
    for b in objects:
        b.update(dt)

    if controlling is heli:
        if keyboard[key.UP]:
            heli.up()
        if keyboard[key.DOWN]:
            hook.drop()
        if keyboard[key.RIGHT]:
            heli.right()
        elif keyboard[key.LEFT]:
            heli.left()
    elif controlling is truck:
        if keyboard[key.RIGHT]:
            truck.right()
        elif keyboard[key.LEFT]:
            truck.left()

    track_controlled(dt)


def setup_scene():
    global batch, heli, hook, truck, controlling, ground
    batch = pyglet.graphics.Batch()

    # Create the ground
    ground = pyglet.sprite.Sprite(earth_tex, 0, 0, batch=batch)
#    group = pyglet.sprite.SpriteGroup(
#        earth_tex, gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA
#    )

#    l, b = world_to_screen((0, 0))
#    r, t = world_to_screen((W, FLOOR))
#    batch.add(4, gl.GL_QUADS, group,
#        ('v2f', [l, b, l, t, r, t, r, b]),
#        ('t2f', [0, 0.5, 0, 1, 10, 1, 10, 0.5]),
#    )

    spawn_heli(100, 4)
    objects.append(Crate((11, FLOOR + 0.5)))

    truck = Truck((10, FLOOR))
    objects.append(truck)

    controlling = truck


def spawn_heli(hx, hy):
    global heli, cable, hook
    heli = Heli((hx, hy + 3.5))

    cable = Cable((hx + 3, hy + 1.5))

    jd = b2RevoluteJointDef()
    jd.Initialize(heli.body, cable.segments[0], (hx + 1, hy + 1.5))
    heli.cable_joint = world.CreateJoint(jd).getAsType()

    objects.append(heli)
    objects.append(cable)

    hook = cable.rend


def on_draw():
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()

    # Gradient sky
    l, b = world_to_screen((0, 0))
    r, t = world_to_screen((W, H))
    horizon = 177 / 255.0, 202 / 255.0, 1.0
    zenith = 68 / 255.0, 0.5, 1.0
    pyglet.graphics.draw(4, gl.GL_QUADS,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('c3f', sum([horizon, zenith, zenith, horizon], ())),
    )

    cx, cy = camera
    tx, ty = world_to_screen((-cx + W * 0.5, -cy + H * 0.5))
    gl.glTranslatef(tx, ty, 0)
    batch.draw()

    # Water
    l, b = world_to_screen((0, 0))
    r, t = world_to_screen((1000, WATER_LEVEL))
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    pyglet.graphics.draw(4, gl.GL_QUADS,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('c4f', [0, 0.2, 0.8, 0.5] * 4),
    )


def on_key_press(symbol, modifiers):
    global controlling
    keyboard.on_key_press(symbol, modifiers)
    if symbol == key.TAB:
        if controlling is heli:
            controlling = truck
        else:
            controlling = heli


if __name__ == '__main__':
    world = setup_world()
    contact_listener = ContactListener()
    world.SetContactListener(contact_listener)
    setup_scene()

    window = pyglet.window.Window(
        width=int(W / SCALE),
        height=int(H / SCALE)
    )
    keyboard = key.KeyStateHandler()
    window.push_handlers(keyboard)

    window.event(on_draw)
    window.event(on_mouse_press)
    window.event(on_mouse_release)
    window.event(on_mouse_drag)
    window.event(on_key_press)

    pyglet.clock.schedule(update)
    pyglet.app.run()
