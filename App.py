# -*- coding: utf-8 -*-
"""
Created on Tue Mar 08 15:50:22 2016

@author: John

Notes/To-do:
        
    Improve efficiency. Numpy cross products in particular are expensive.
            
"""


import Physics
import Solver
import Level
import Collisions
import numpy as np
import struct
import pybullet as pyb
import os
import pyglet as pyg
import pyglet.window.key as key
import ModernGL as mgl
from ModernGL.ext.obj import Obj
from pyrr import Matrix44

currentWorkingDirectory = os.getcwd()
imagesDirectory = currentWorkingDirectory + '\\images\\'
objDirectory = currentWorkingDirectory + '\\objects\\'

# Choose levelNumber
levelNumber = 10

# Choose Framerate
fps = 120

speedMultiplier = 1/250

class App(object):
    """ Do the thing.
    """
    def __init__(self):
        
        self.initializePyglet()
        self.initializeModernGL()
        self.initializeSimulation()
        self.pressAndReleaseHandler() #for keys that are pressed/released
        
    def initializePyglet(self):
        """Open a window, initialize pushed key handler.
        """
        self.wnd = pyg.window.Window(width=1280, height=720)
        
        #Handle Handlers
        self.held = key.KeyStateHandler()
        self.wnd.push_handlers(self.held) #For keys that are held
        
    def initializeModernGL(self):
        """Initialize all things related to running modernGL
        """
        
        self.ctx = mgl.create_context()
            
        self.prog = self.ctx.program(
                vertex_shader = '''
                #version 330
                
                uniform mat4 Mvp;
                
                // Per vertex
                in vec3 in_vert_c;
                //in vec3 in_norm_c;
                in vec2 in_text_c;
                
                in vec3 in_vert_s;
                //in vec3 in_norm_s;
                in vec2 in_text_s;
                
                // Per object
                in vec3 in_pos;
                in mat3 in_rotMat;
                in vec3 in_size;
                in float texindex;
                in int shape;
                
                out vec3 v_vert;
                flat out int out_texindex;
                //out vec3 v_norm;
                out vec2 v_text;
                
                vec3 in_vert;
                vec2 in_text;
                //vec3 in_norm;
                                
                void main() {
                    // if shape is 0, it's a cube; if it's 1, it's a sphere
                    if (shape==1) {
                        in_vert = in_vert_s;
                        v_text = in_text_s;
                        //in_norm = in_norm_s;
                        }
                    else {
                        in_vert = in_vert_c;
                        v_text = in_text_c;
                        //in_norm = in_norm_c;
                        }
                    v_vert = (in_vert * in_size) * in_rotMat + in_pos;
                    //v_norm = (in_norm * in_size) * in_rotMat + in_pos;
                    gl_Position = Mvp * vec4(v_vert, 1.0);
                    out_texindex = int(texindex);

                }
            ''',
            fragment_shader = '''
                #version 330
                uniform sampler2D textures[2];
                //uniform vec3 Light;
                
                flat in int out_texindex;
                in vec3 v_vert;
                //in vec3 v_norm;
                in vec2 v_text;
                
                out vec4 f_color;
                
                void main() {
                    
                    //float lum = clamp(dot(normalize(Light - v_vert), normalize(v_norm)), 0.0, 1.0) * 0.8 + 0.2;
                    //vec3 base = vec3(0.5, 0.5, 0.5) * lum;
                    //vec3 spec = vec3(1.0, 1.0, 1.0) * pow(lum, 6);
                    vec4 tex = texture(textures[out_texindex], v_text);
                    //f_color = vec4(base * 0.1 + tex.rgb * lum + spec, tex.a);
                    f_color = vec4(tex.rgb, tex.a);
                }
            ''',
        )
        
        # Assign self.mvp to the uniform Mvp in the modernGL program
        self.mvp = self.prog['Mvp']
        
        # This should be programmatically determined, but basically I'm telling
        # the program that I've loaded two textures which can be accessed with
        # either a 0 or a 1 in the fragment shader.
        self.prog['textures'].value = [0,1]

        # Haven't totally figured out how to use this stuff yet
#        self.light = self.prog.uniforms['Light']
#        self.light.value = (0,0,0) #-140.0, -300.0, 350.0
        
        # Uses an .obj file for vertex, texture, and surface normal information
        self.cubeobj = Obj.open(objDirectory + '\\cube\\cube.obj')
        self.sphereobj = Obj.open(objDirectory + '\\sphere\\Sphere.obj')
        
        # Each of these buffers provides the vertex and texture position info 
        # to the vertex shader. Note that I should really be using a separate
        # program for cubes and spheres, but I've found a way to make this
        # work for now.
        # Only one gets used when rendering each entity.
        cubeVertBuffer = self.ctx.buffer(self.cubeobj.pack('vx vy vz tx ty'))
        sphereVertBuffer = self.ctx.buffer(self.sphereobj.pack('vx vy vz tx ty'))

        # entityBuffer is set up to provide the position, orientation, size,
        # texture index (each texture is loaded in Level), and shape type 
        # (cube or sphere).
        self.entityBuffer = self.ctx.buffer(reserve=1024 * 1024)

        # Connect all of the buffers to a vertex array object which will be in
        # charge of rendering.
        vao_content = [(cubeVertBuffer,    '3f 2f', 'in_vert_c', 'in_text_c'),
                       (sphereVertBuffer,  '3f 2f', 'in_vert_s', 'in_text_s'),
                       (self.entityBuffer, '3f 9f 3f 1f 1i/i', 'in_pos',
                            'in_rotMat', 'in_size', 'texindex', 'shape'),]

        self.vao = self.ctx.vertex_array(self.prog, vao_content)
        
    def calcPosCoM(self):
        """Calculates the position of the center of mass of the system.
        """
        
#        # Calculate the coordinates of the CoM for pygame
        masses  = np.array([s.body.mass for s in self.listOfEntities if s.body is not None])

        # Calculate the position of the CoM in meters
        xVal = np.array([s.body.x.x for s in self.listOfEntities if s.body is not None])
        yVal = np.array([s.body.x.y for s in self.listOfEntities if s.body is not None])
        zVal = np.array([s.body.x.z for s in self.listOfEntities if s.body is not None])

        x = np.sum(masses*xVal)/np.sum(masses)
        y = np.sum(masses*yVal)/np.sum(masses)
        z = np.sum(masses*zVal)/np.sum(masses)

        return np.array([x,y,z])

    def initializeSimulation(self):
        """ Runs the basic necessities to get the simulation up and running.
        """
        # Miscellaneous bits

        # Determines how quickly the simulation runs. Larger means bigger jumps.
        self.speedMultiplier=speedMultiplier
        self.nextTime = 0
                
        self.currentTime=0
        self.colFlag=False
        
        # Pick the level which will construct the listOfSprites and decide 
        # where the walls are if box==True
        level = Level.Level(imagesDirectory,levelNumber, self.ctx)
        self.listOfEntities, l,r,t,b,f,a, box, g = level.getAttributes()
        self.box = box
        
        # Initialize simulation shtuff
        self.physics = Physics.nBodyRotation(self.listOfEntities)
#        self.physics = Physics.newtonian(self.listOfEntities)
        
        # Choose a solver. The SciPy one is more accurate but can occasionally
        # hang if it doesn't converge to a sufficiently low error. They seem
        # to run at roughly equal speeds.
#        solver = Solver.RK45_SciPy(self.physics.diffEq)
        solver = Solver.RK4(self.physics.diffEq)
        self.physics.solver = solver

        if not g: 
            self.physics.G=0
            print("gravity off")
        
        if box is True:
            print('Box is on')
            self.t = t # top wall
            self.b = b # bottom wall
            self.r = r # right wall
            self.l = l # left wall
            self.a = a # aft wall
            self.f = f # fore wall
        else:
            print('Box is off')
            self.t = None
            self.b = None
            self.r = None
            self.l = None   
            self.a = None
            self.f = None

        # Search is not yet truly implemented
        search = 'newton' # 'newton', 'bisect', or 'golden'
        
        # Currently testing out two different styles of collision handling
        self.colHandler = Collisions.CollisionHandler(self.listOfEntities, search, l,r,t,b,f,a,g)
#        self.colHandler = Collisions.CollisionHandlerRepulsion(self.listOfEntities, l,r,t,b,f,a,g)
        
        # Calculate center of mass. I don't think this is being used currently
        #, but it could be used to keep the camera focused on the center of mass.
        self.com = self.calcPosCoM()

    def pressAndReleaseHandler(self):
        """ Processes all user inputs.
        """        
            # Check for buttons that have been pressed
        def on_key_press(symbol, modifier):

            # '+' and '-' on the key pad speed up or slow down simulation
            if symbol == key.NUM_ADD:
                self.speedMultiplier *= 2
            elif symbol == key.NUM_SUBTRACT:
                self.speedMultiplier *= 0.5

            # Check for buttons that have been released
        def on_key_release(symbol, modifier):
            pass
                
        self.wnd.push_handlers(on_key_press,on_key_release)
                
    def heldKeyHandler(self):
        """ Process held buttons; currently doesn't do much
        """
        pass

    def update(self, dt, *args, **kwargs):
        """ The foundation for running the animation and simulation. Gets called
        by the pyglet application framework.
        """
        self.currentTime += dt
        stepsize = dt*self.speedMultiplier

        self.physics.advance(stepsize)
        self.colHandler.checkCollisionsPybullet()
#        self.colHandler.checkCollisions()

#        while self.colHandler.checkCollisions() is True:
#        if self.colHandler.checkColPybNoPenetration() is True:
            
#            i+=1
#            if i>25: 
#                print('Timeout: Failed to resolve collision')
#                break
#            print('here')
#            self.listOfEntities = cp.deepcopy(listOfEntities)
#            self.physics.listOfEntities = self.listOfEntities
#            self.colHandler.listOfEntities = self.listOfEntities
##            step /= 2
##            self.nextTime -= 0.1 #self.stepsize
##            print(self.nextTime)
#            self.currentTime = self.physics.advance(self.currentTime - self.stepsize*0.9)
#            time.sleep(1)
#        print('passed collision detection')
            
#        tryAgain = self.colHandler.checkCollisionsPybullet()
#        if tryAgain:
#            self.colHandler.search()
                        
#        self.nextTime = self.currentTime + self.stepsize

        
#        print(self.currentTime)
#        self.physics.solver.set_initial_value = 0
#        self.physics.advance(self.dt)

#         Update the display
        
        width, height = (self.wnd.width, self.wnd.height)
        self.ctx.viewport = (0, 0, width, height)
        self.ctx.clear(1.0, 1.0, 1.0)
        self.ctx.enable_only(mgl.DEPTH_TEST | mgl.BLEND)
#        self.ctx.enable(mgl.BLEND) # This enables use of the tint which I 
        # think determines that the background of each square will be white 
        # with an alpha of 0, meaning transparent.
#        self.ctx.enable(mgl.DEPTH_TEST)
#        self.ctx.enable(mgl.CULL_FACE)
#        angle = self.currentTime
#        camera_pos = (self.com[0],self.com[1],self.com[2]+1)
        camera_pos = (0,0,2)

        proj = Matrix44.perspective_projection(45, width / height, .1, 1000.0) # fovy, aspect, near, far
        lookat = Matrix44.look_at(
            camera_pos, # Eye position
            (0,0,0),#self.com, # target position
            (0,1,0), # "up"?
        )

        self.mvp.write((proj * lookat).astype('f4').tobytes())
        
        self.com = self.calcPosCoM()

#        self.floorHeight+=self.com[1]
#        self.ceilingHeight+=self.com[1]
#        self.leftWall+=self.com[0]
#        self.rightWall+=self.com[0]

        self.entityBuffer.write(b''.join(struct.pack('3f9f3f1f1i', 
                ent.body.x.x, ent.body.x.y, ent.body.x.z,
                *ent.body.q.asMatrix().flatten(), 
                *ent.size, ent.imageIndex, ent.entTypeInt) for ent in self.listOfEntities))
        
#        self.mug_texture.use()
        self.vao.render(vertices = len(self.sphereobj.vert)*5, instances=len(self.listOfEntities)) # mgl.TRIANGLE_STRIP

#============================================================================#

class SimEntity(object):
    """ Class for building and maintaining information about generic objects 
    involved in the simulation. The body contains the purely physical info
    while this class contains the extra info used for drawing and collision
    detection.
    
    Parameters
    -----
    body: instance of RigidBody
        Contains information like the mass, moment of inertia, orientation, etc.
    imageIndex: int
        In Level, we specify which image is which with an index.
    entityType: string
        Either "ball" or "box." Determines how collisions are handled.
    size: float
        The radius or half-extents of the object in screen coordinates.
    
    """
    def __init__(self, body, imageIndex, entityType, size):
        
        self.body        = body
        self.imageIndex  = imageIndex
        self.entityType  = entityType
        self.size        = size
        
        if entityType is 'box':
            self.boundRadius = np.linalg.norm(np.array(size))
            self.entTypeInt = 0 # the shaders can't handle strings
        elif entityType is 'ball':
            self.boundRadius = max(size)
            self.entTypeInt = 1
        
#        self.accVector = Vector.Quaternion(0, np.array([0,0,0]))
#        self.rotAccVector = Vector.Quaternion(0, np.array([0,0,0]))

        # Default assumptions
        self.angle    = body.q.Zangle
        if len(size)==2:
            size.append(0)
                    
    @property
    def vertices(self):
        """Verticies of shape with respect to origin.
        The array starts with the bottom left vertex on the front side and 
        wraps around clockwise. Then it moves to the back face in the same
        pattern. 
        """
        rMat = self.body.q.asMatrix()

        x = self.size[0]
        y = self.size[1]
        z = self.size[2]
        
        v=[
        rMat @ np.array([-x,-y, z]).reshape([3,1]),
        rMat @ np.array([-x, y, z]).reshape([3,1]),
        rMat @ np.array([ x, y, z]).reshape([3,1]),
        rMat @ np.array([ x,-y, z]).reshape([3,1]),
        rMat @ np.array([-x,-y,-z]).reshape([3,1]),
        rMat @ np.array([-x, y,-z]).reshape([3,1]),
        rMat @ np.array([ x, y,-z]).reshape([3,1]),
        rMat @ np.array([ x,-y,-z]).reshape([3,1])]
        
        for i in range(8):
            v[i] = v[i].flatten() + self.body.x.v
        
        return v

if __name__=='__main__':

#    try:
    myApp = App()
        
#    except:
#        print("failed")
#        pyg.app.exit()
#        failed = True
        
#    if not failed:
    pyg.clock.schedule_interval(myApp.update, 1/fps)
    pyg.app.run()
    pyg.clock.unschedule(myApp.update)
    pyb.disconnect()
    pyg.app.exit()
    