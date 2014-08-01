import sys, random, os
sys.path.append('C:/Users/JieTan/Documents/MyProjects/AdobeIntern/Mitsuba 0.5.0/python/3.3')
# Ensure that Python will be able to find the Mitsuba core libraries
os.environ['PATH'] = 'C:/Users/JieTan/Documents/MyProjects/AdobeIntern/Mitsuba 0.5.0' + os.pathsep + os.environ['PATH']
import mitsuba, math, time
from math import *
from mitsuba.core import *
from mitsuba.render import *
import multiprocessing
from os import listdir
from os.path import isfile, join

def getShapesAABB(shapes):
    aabb = shapes[0].getAABB()
    for i in range(1,len(shapes)):
        aabb.expandBy(shapes[i].getAABB())

    return aabb

def goRenderTurnTableAnimation(scene):
    scheduler = Scheduler.getInstance()

    for i in range(0,multiprocessing.cpu_count()):
        scheduler.registerWorker(LocalWorker(i,'wrk%i' % i))

    sensor = scene.getSensor()
    worldTransform = sensor.getWorldTransform(0).getMatrix()
    cameraPos = Point(worldTransform[0,3],worldTransform[1,3],worldTransform[2,3])
    stepSize = 10

    scheduler.start()
    queue = RenderQueue()

    shapes = scene.getShapes()
    aabb = getShapesAABB(shapes)
    center = aabb.getCenter()

    for i in range(0,360/stepSize):
        rotationCur = Transform.rotate(Vector(0,1,0),i*stepSize)
        rotationNext = Transform.rotate(Vector(0,1,0),(i+1)*stepSize)
        trafoCur = Transform.lookAt(rotationCur*cameraPos,Point(center),rotationCur*Vector(0,1,0))
        trafoNext = Transform.lookAt(rotationNext*cameraPos,Point(center),rotationNext*Vector(0,1,0))

        atrafo = AnimatedTransform()
        atrafo.appendTransform(0,trafoCur)
        atrafo.appendTransform(1,trafoNext)
        atrafo.sortAndSimplify()
        sensor.setWorldTransform(atrafo)
        scene.setDestinationFile('frame_%03i' % i)
        job = RenderJob('job_%i' % i,scene,queue)

        job.start()
        queue.waitLeft(0)
        queue.join()
    print(Statistics.getInstance().getStats())

def genRandomPosition(distance):
    radiusRange = (0.5,1)
    elevationRange = (40,90)
    randRadius = random.uniform(radiusRange[0]*distance,radiusRange[1]*distance)
    randTheta = random.uniform(0,360*pi/180)
    randPhi = random.uniform(elevationRange[0]*pi/180,elevationRange[1]*pi/180)
    # For 2012 - 2014 models with gravity = -y
    #randPos = [randRadius*sin(randPhi)*cos(randTheta),randRadius*cos(randPhi),randRadius*sin(randPhi)*sin(randTheta)]
    # For 2011 models with gravity = -z
    randPos = [randRadius*sin(randPhi)*cos(randTheta),randRadius*sin(randPhi)*sin(randTheta),randRadius*cos(randPhi)]
    return Point(randPos[0],randPos[1],randPos[2])

def writeLogFile(fname,tMat,frameID):
    f= open(fname,'a')
    f.writelines(frameID[1:]+'\n')
    for i in range(4):
        for j in range(4):
            f.write(tMat[i,j].__str__()+'\t');
        f.write('\n')
    f.close()

def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)

def goRenderRandomCameras(scene,nRenderings,OUT_DIR,scheduler):
    queue = RenderQueue()
    sensor = scene.getSensor()
    aabb = getShapesAABB(scene.getShapes())
    diagLength = aabb.getBSphere().radius
    sensorProperties = sensor.getProperties()
    fov = sensorProperties['fov']
    distance = diagLength/(2*math.tan(fov*0.5*math.pi/180.0))
    distance = max(aabb.max.x,aabb.max.y,aabb.max.z)+distance
    sceneResID = scheduler.registerResource(scene)

    for i in range(nRenderings):
        frameID = '_%05i'%i
        destination = join(OUT_DIR,scene.getID(),'depth',scene.getID()+frameID)
        ensure_dir(destination)
        newScene = Scene(scene)
        pmgr= PluginManager.getInstance()
        newSensor = pmgr.createObject(scene.getSensor().getProperties())
        # Sample random camera position
        randPos = genRandomPosition(distance)
        #newWorldTransform = Transform.lookAt(randPos,aabb.getCenter(),Vector(0,1,0)) #Gravity along x
        newWorldTransform = Transform.lookAt(randPos,aabb.getCenter(),Vector(0,0,1)) # Gravity along z
        newSensor.setWorldTransform(newWorldTransform)
        newFilm = pmgr.createObject(scene.getFilm().getProperties())
        newFilm.configure()
        newSensor.addChild(newFilm)
        newSensor.configure()
        newScene.addSensor(newSensor)
        newScene.setSensor(newSensor)
        newScene.setSampler(scene.getSampler())
        newScene.setDestinationFile(destination)
        logFile = join(OUT_DIR,scene.getID(),scene.getID()+'.log')
        tMat = newWorldTransform.getMatrix()
        writeLogFile(logFile,tMat,frameID)
        job = RenderJob('myRenderJob'+str(i),newScene,queue,sceneResID)
        job.start()
        queue.waitLeft(0)
        queue.join()


# Function to add the range sensor to the scene
def getRangeSensor(scene):
    pmgr = PluginManager.getInstance()
    fov = 58.0 #X axis field of view of the sensor
    # Get shapes and total bounding box
    shapes = scene.getShapes()
    aabb = getShapesAABB(shapes)

    # Setup camera parameters
    center = aabb.getCenter()
    diagLength = aabb.getBSphere().radius;
    extents = aabb.getExtents()
    maxExtentsXY = max(extents.x,extents.y)
    distance = maxExtentsXY/(2*math.tan(fov*0.5*math.pi/180.0))
#    maxExtentsXYZ = max(extents.z, maxExtentsXY)
    farClip = diagLength*5+distance
    nearClip = distance/100
    toWorld = Transform.translate((Vector(center.x,center.y,aabb.min.z-distance)))
    # Setup new scene and copy shapes from old scene
    newScene = Scene()
    for i in range(len(shapes)):
        newScene.addChild(shapes[i])
    # Add perspective sensor and multichannel integrators to new scene
    newScene.addChild(pmgr.create({'type':'perspective',
                                'toWorld':toWorld,
                                'fov': fov,
                                'fovAxis' : 'diagonal',
                                'nearClip' : nearClip,
                                'farclip' : farClip,
                                'sampler' : {
                                    'type' : 'ldsampler',
                                    'sampleCount' : 32
                                    },
                                'film': {
                                    'type':'hdrfilm',
                                    'width' : 640,
                                    'height': 480,
                                    'pixelFormat':'xyz,luminance',
                                    'channelNames':'normal,distance',
                                    'banner':False,
                                    'rfilter' : {'type':'gaussian'}
                                    }
                                }))
    # Add integrators
    multChannels = Properties('multichannel')
    multChannels = pmgr.createObject(multChannels)
    normalsIntegrator = Properties('field')
    normalsIntegrator['field']='shNormal'
    distanceIntegrator = Properties('field')
    distanceIntegrator['field'] = 'distance'
    normalsIntegrator = pmgr.createObject(normalsIntegrator)
    distanceIntegrator = pmgr.createObject(distanceIntegrator)
    normalsIntegrator.configure()
    distanceIntegrator.configure()
    multChannels.addChild('integrators',normalsIntegrator)
    multChannels.addChild('integrators',distanceIntegrator)
    multChannels.configure()
    newScene.addChild(multChannels)
    newScene.configure()
    return newScene

def getModelNames(MODEL_DIR):
    #fnames = [ f[:len(f)-4] for f in listdir(MODEL_DIR) if isfile(join(MODEL_DIR,f)) and (f.endswith(".xml"))]
    #return fnames
    fnames = [f for f in listdir(MODEL_DIR)]
    return fnames

def main():
    MODEL_DIR = 'C:/Users/JieTan/Documents/MyProjects/AdobeIntern/PrincetonModelNet/chair/'
    OUT_DIR = 'C:/Users/JieTan/Documents/MyProjects/AdobeIntern/PrincetonModelNet/chair/'
    nRenderings = 10
    fnames = getModelNames(MODEL_DIR)
    fileResolver = Thread.getThread().getFileResolver()
    
    scheduler = Scheduler.getInstance()
    for i in range(0,multiprocessing.cpu_count()):
        scheduler.registerWorker(LocalWorker(i,'wrk%i' % i))
    scheduler.start()
    for sceneName in fnames:
        fileResolver.appendPath(MODEL_DIR+'/'+sceneName)
        scene = SceneHandler.loadScene(fileResolver.resolve(sceneName+".xml"))
        newScene = getRangeSensor(scene)
        newScene.setID(sceneName)
        #nRenderings = 100;
        #goRenderAnimation(newScene)
        goRenderRandomCameras(newScene,nRenderings,OUT_DIR,scheduler)
        time.sleep(10)

if __name__ == '__main__':
    main()
