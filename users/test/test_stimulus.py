import time
import numpy
try:
    import serial
except:
    pass
import os.path
import os

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import visexpman
from visexpman.engine.vision_experiment.configuration import VisionExperimentConfig
from visexpman.engine.generic import utils,fileop,introspect
from visexpman.engine.vision_experiment import experiment
from visexpman.engine.hardware_interface import daq_instrument
from visexpman.users.test import unittest_aggregator
from visexpman.users.common import stimuli

class TestNTGratingConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestNTGratingExp'
        self._create_parameters_from_locals(locals())

class TestNTGratingExp(experiment.Experiment):
    def run(self):
        for o in range(0,360,45):
            self.show_grating_non_texture(3,100,100,o,2,contrast=1.0,background_color=0.0)
            break

class TestShowGratingPars(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestShowGratingExp'
        self._create_parameters_from_locals(locals())
        
class TestShowGratingExp(experiment.Experiment):
    def run(self):
        self.show_grating(duration=10,velocity=2000,white_bar_width=100)

class LaserBeamStimulusConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.REPEATS =2
        self.POSITIONS = utils.rc(0.8*numpy.array([[0,0], [-0.5,0],[-1,0],[-2,0],[0,0], [0,-0.5],[0,-1],[0,-2]]))
        self.JUMP_TIME = 0.05
        self.HOLD_TIME = 0.5
        self.runnable = 'LaserBeamStimulus'
        self._create_parameters_from_locals(locals())
        
class LaserBeamStimulusConfigOutOfRange(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.REPEATS =2
        self.POSITIONS = utils.rc(100.0*numpy.array([[0,0], [-0.5,0],[-1,0],[-2,0],[0,0], [0,-0.5],[0,-1],[0,-2]]))
        self.JUMP_TIME = 0.05
        self.HOLD_TIME = 0.5
        self.runnable = 'LaserBeamStimulus'
        self._create_parameters_from_locals(locals())

class TestTextureStimConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestTextureStim'
        self._create_parameters_from_locals(locals())
        
class TestTextureStim(experiment.Experiment):
    def run(self):
        nframes = int(self.machine_config.SCREEN_EXPECTED_FRAME_RATE*60*1)
        shape = (1, self.config.SCREEN_RESOLUTION['row'], self.config.SCREEN_RESOLUTION['col'], 3)
        data = numpy.random.random(shape)
#        data = numpy.cast['uint8'](data)
        texture = data[0]
        diagonal = numpy.sqrt(2) * numpy.sqrt(self.config.SCREEN_RESOLUTION['row']**2 + self.config.SCREEN_RESOLUTION['col']**2)
        diagonal =  numpy.sqrt(2) * self.config.SCREEN_RESOLUTION['col']
        alpha =numpy.pi/4
        angles = numpy.array([alpha, numpy.pi - alpha, alpha + numpy.pi, -alpha])
        vertices = 0.5 * diagonal * numpy.array([numpy.cos(angles), numpy.sin(angles)])
        vertices = vertices.transpose()
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(vertices)
        dt = GL_FLOAT
#        dt = GL_UNSIGNED_BYTE
        glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[0], texture.shape[1], 0, GL_RGB, dt, texture)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
        glEnable(GL_TEXTURE_2D)
        glEnableClientState(GL_TEXTURE_COORD_ARRAY)
        texture_coordinates = numpy.array(
                             [
                             [1.0, 1.0],
                             [0.0, 1.0],
                             [0.0, 0.0],
                             [1.0, 0.0],
                             ])
        glTexCoordPointerf(texture_coordinates)
        t0=time.time()
        for frame_i in range(nframes):
#            texture = data[frame_i]
            texture = numpy.random.random(shape)[0]
            texture == texture*0.5+0.5
            glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[0], texture.shape[1], 0, GL_RGB, dt, texture)
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv((1.0,1.0,1.0))
            glDrawArrays(GL_POLYGON,  0, 4)
            self._flip_and_block_trigger(frame_i, frame_i+10, True, False)#Don't want to calculate the overall number of frames
            if self.abort:
                break
        dt = time.time()-t0
#        print dt, nframes, nframes/dt
        glDisable(GL_TEXTURE_2D)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        if nframes/dt < self.machine_config.SCREEN_EXPECTED_FRAME_RATE*0.95:
            raise RuntimeError('Expected frame rate: {0}, measured frame rate {1}'.format(nframes/dt, self.machine_config.SCREEN_EXPECTED_FRAME_RATE))

class TestVideoExportConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestVideoExportExp'
        self._create_parameters_from_locals(locals())

class TestVideoExportExp(experiment.Experiment):
    def run(self):
        for c in numpy.arange(0.0, 1.0, 0.01):
            self.show_fullscreen(color = c)
        self.export2video(os.path.join(self.machine_config.EXPERIMENT_DATA_PATH, 'out.mp4'), img_format='png')
        
class TestCurtainConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestCurtainExp'
        self._create_parameters_from_locals(locals())

class TestCurtainExp(experiment.Experiment):
    def run(self):
        self.moving_curtain(self.machine_config.SCREEN_SIZE_UM['col']*0.5, color = 1.0, direction=45.0, background_color = 0.0, pause = 0.0)
        
class TestNaturalStimConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestNaturalStimExp'
        self._create_parameters_from_locals(locals())

class TestNaturalStimExp(experiment.Experiment):
    def run(self):
        res=[]
        for dc in [True,False]:
            for rep in range(self.machine_config.REPEATS):
                if self.abort:
                    break
                for directions in self.machine_config.DIRECTIONS:
                    if self.abort:
                        break
                    r=self.show_natural_bars(speed = self.machine_config.SPEED, repeats = 1, duration=self.machine_config.DURATION, 
                                minimal_spatial_period = self.machine_config.MSP, spatial_resolution = self.machine_config.SCREEN_PIXEL_TO_UM_SCALE, 
                                intensity_levels = 255, direction = directions, circular=self.machine_config.CIRCULAR,fly_in=not self.machine_config.CIRCULAR,
                                fly_out=not self.machine_config.CIRCULAR,save_frame_info =True, is_block = False,duration_calc_only=dc)
                    res.append(r)
        numpy.testing.assert_almost_equal(sum([i for i in res if i !=None]), self.frame_counter/self.machine_config.SCREEN_EXPECTED_FRAME_RATE,0)
        if utils.safe_istrue(self.machine_config, 'STIM2VIDEO') and hasattr(self.machine_config, 'OUT_PATH') and self.machine_config.OS == 'Linux':
            self.export2video(os.path.join(self.machine_config.OUT_PATH, 'natural_stim.mp4'))
        if utils.safe_istrue(self.machine_config, 'EXPORT_INTENSITY_PROFILE'):
            txtpath = os.path.join(self.machine_config.OUT_PATH, 'intensity_profile.txt')
            if os.path.exists(txtpath):
                os.remove(txtpath)
            numpy.savetxt(txtpath, numpy.round(self.intensity_profile,4))
            from pylab import plot, savefig
            plot(self.intensity_profile)
            savefig(os.path.join(self.machine_config.OUT_PATH, 'intensity_profile.png'))
            
            
class TestMovingShapeConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.REPETITIONS = 1
        self.SHAPE_SIZE = utils.rc((100,200))
        self.DIRECTIONS = [0,45,90]
        self.SPEEDS = [2000,3000]
        self.SHAPE_CONTRAST = 0.5
        self.SHAPE_BACKGROUND = 0.1
        self.SHAPE = 'rect'
        self.PAUSE_BETWEEN_DIRECTIONS = 0.1
        self.runnable = 'MovingShapeExperiment'
        self._create_parameters_from_locals(locals())

class Pointing2NotExistingConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'None'
        self._create_parameters_from_locals(locals())
        
class Pointing2NonExpConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'DummyClass'
        self._create_parameters_from_locals(locals())
        
class DummyClass(object):
    pass

class WhiteNoiseParameters(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.DURATION = 1.0
        self.PIXEL_SIZE = 50.0
        self.FLICKERING_FREQUENCY = 30.0
        self.N_WHITE_PIXELS = None
        self.COLORS = [0.0, 1.0]
        self.runnable = 'WhiteNoiseExperiment'
        self._create_parameters_from_locals(locals())

class WhiteNoiseExperiment(experiment.Experiment):
    def run(self):
        self.show_white_noise(duration = self.experiment_config.DURATION,
            pixel_size = self.experiment_config.PIXEL_SIZE, 
            flickering_frequency = self.experiment_config.FLICKERING_FREQUENCY, 
            colors = self.experiment_config.COLORS,
            n_on_pixels = self.experiment_config.N_WHITE_PIXELS)
        self.show_fullscreen()

class GUITestExperimentConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'Debug'
        self.DURATION = 10.0#Comment
        self.PAR2 = 2.0#Comment2
        self.PAR1 = 1.0#Comment1
        self.editable=True
        self._create_parameters_from_locals(locals())

class TestCommonExperimentConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'MovingShapeExperiment'
        SHAPE_SIZE = 100
        SPEEDS = [1000.0]
        DIRECTIONS = [90]
        REPETITIONS=1
        self._create_parameters_from_locals(locals())
        
class DebugExperimentConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'Debug'
        self.DURATION = 15#Comment
        self.SPOT_SIZE = 400#Comment
#        self.pre_runnable = 'TestPre'
        self._create_parameters_from_locals(locals())

class Debug(experiment.Experiment):
    def prepare(self):
        self.stimulus_duration = self.experiment_config.DURATION
    
    def run(self):
#        self.show_fullscreen(duration=0, color=0.5)
#        self.show_shape(duration=0,size=10, color=1.0, part_of_drawing_sequence=True, flip=False)
#        self.show_shape(duration=0,pos = utils.rc((10, 0)), size=30, color=0.7, part_of_drawing_sequence=True, flip=False)
#        self.show_shape(duration=0,size=100, color=0.4, part_of_drawing_sequence=True, flip=True)
        nsteps = 2
        d=self.stimulus_duration/(4*nsteps+1.)
        self.show_fullscreen(duration=d, color=0.0)
        for i in range(nsteps):
            self.show_shape(duration=d,size=self.experiment_config.SPOT_SIZE, color=0.9,is_block=True)
            self.show_fullscreen(duration=2*d, color=0.0)
        
class TestStimulusBlockParams(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.COLORS = [0.1, 0.6]
        self.T_FULLSCREEN = 1.0
        #TODO: get image rendering run on Linux
        self.IMAGE_FOLDERS = [os.path.join(fileop.visexpman_package_path(), 'data', 'images')] if self.OS != 'Linux' else []#TODO: show_image should work on linux too
        self.IMAGE_STRETCHES = [0.5, 1.4]
        self.T_IMAGE = 1.0
        self.SHAPES = ['rect', 'o']
        self.T_SHAPE = [1.0, 0.0]
        self.POSITIONS = [utils.rc((0,0))]
        self.SIZES = [100.0, utils.rc((100.0,20.0))]
        self.T_GRATING = [1.0]
        self.GRATING_PROFILES = ['sin']
        self.GRATING_WIDTH = [100.0]
        self.GRATING_SPEEDS = [0.0, 800.0]
        self.runnable = 'TestStimulusBlocks'
        self._create_parameters_from_locals(locals())
        
class TestStimulusBlocks(experiment.Experiment):
    #TODO =: separate test for centering
    def run(self):
        import itertools
        flip = [True,False]
        is_block = [True,False]
        count = [True,False]
        frame_trigger = [True,False]
        ct=0
        for flipi, is_blocki, counti, frame_triggeri, color in itertools.product(*[flip, is_block, count, frame_trigger, self.experiment_config.COLORS]):
            if is_blocki and flipi:
                ct+=1
            self.show_fullscreen(duration = self.experiment_config.T_FULLSCREEN,  color = color, flip = flipi, count = counti, is_block = is_blocki, frame_timing_pulse = frame_triggeri)
        params = [self.experiment_config.IMAGE_FOLDERS, self.experiment_config.IMAGE_STRETCHES, is_block]
        for path, stretch, is_block_i in itertools.product(*params):
            if is_block_i:
                ct+=1
            self.show_image(path, duration = self.experiment_config.T_IMAGE,  stretch=stretch, flip=True, is_block = is_block_i)
        params = [self.experiment_config.SHAPES, self.experiment_config.T_SHAPE, self.experiment_config.POSITIONS, 
                            self.experiment_config.COLORS, self.experiment_config.SIZES, flip, is_block]
        for shape, duration, pos, color, size, flip_i, is_block_i in itertools.product(*params):
            self.show_shape(shape = shape,  duration = duration,  
                                pos = pos,  color = color,  background_color = 0.5,  size = size,  flip = flip_i, is_block = is_block_i)
            if is_block_i and flip_i:
                ct+=1
        params = ['T_GRATING', 'GRATING_PROFILES', 'GRATING_WIDTH', 'GRATING_SPEEDS']   
        params = [getattr(self.experiment_config, p) for p in params]
        params.extend([is_block])
        for t, profile, white_bar_width, speed, is_block_i in itertools.product(*params):
            self.show_grating(duration = t,  profile = profile,  white_bar_width =white_bar_width,   
                    velocity = speed, duty_cycle = 1.0, is_block = is_block_i)
            if is_block_i:
                ct+=1
        for b in is_block:
            self.show_natural_bars(speed = 300, repeats = 2, duration=0.5, is_block = b)
            if b:
                ct+=1
        for b in is_block:    
            if b:
                ct+=1
            self.moving_shape(utils.rc((100,300)), [2000.0], [45.0], shape = 'rect', color = 1.0, background_color = 0.5, 
                    moving_range=utils.rc((500.0,500.0)), pause=1.0, repetition = 1, block_trigger = b, shape_starts_from_edge=False)

class TestCheckerboardConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestCheckerboardExp'
        self._create_parameters_from_locals(locals())

class TestCheckerboardExp(experiment.Experiment):
    
    def run(self):
        #White noise
        checker_size = 20#75
        duration = 20*60/100
        self.show_white_noise(duration, checker_size)
        
class TestGratingConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'TestGratingExp'
        self._create_parameters_from_locals(locals())

class TestGratingExp(experiment.Experiment):
        
    def run(self):
        from visexpman.engine.generic import colors
        #glEnable (GL_BLEND)
        #glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        display_area_adjusted=numpy.array([40,40])
        alpha = numpy.arctan(display_area_adjusted[1]/display_area_adjusted[0])
        angles = numpy.array([alpha, numpy.pi - alpha, alpha + numpy.pi, -alpha])
        diagonal = numpy.sqrt((display_area_adjusted **2).sum())
        vertices = 0.5 * diagonal * numpy.array([numpy.cos(angles), numpy.sin(angles)])
        vertices = vertices.transpose()+250
        vertices = numpy.tile(vertices,(2,1))
        vertices[4:]-=100
        print(vertices)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(vertices)
        #self._init_texture(utils.rc((100,100)))
#        c=colors.convert_color([1.0,1.0,1.0], self.config)
#        c.append(1.0)
#        glColor4fv(c)
        for i in range(60*5):
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            texture = numpy.ones((40,40))
            texture[::5]=0.0
#            texture = numpy.rollaxis(numpy.array(4*[numpy.cast['float'](texture)]), 0,3)
#            texture[:,:,1]=0.0
#            texture[:,:,2]=0.0
#            texture[:,:,3]=1.0
            if 1:
#                glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[1], texture.shape[0], 0, GL_RGB, GL_FLOAT, texture)#LUMINANCE
                glTexImage2D(GL_TEXTURE_2D, 0, 1, texture.shape[1], texture.shape[0], 0, GL_LUMINANCE, GL_FLOAT, texture)#LUMINANCE
                #TODO: blending and texture, mask on screen
                
                
                glColor3fv(colors.convert_color([0.0,0.0,0.0], self.config))
                glDrawArrays(GL_POLYGON,  0, 4)

#            glBindTexture(GL_TEXTURE_2D, 0)
#            glDisable(GL_TEXTURE_2D)

#            glColor4fv([0.0,1.0,0.0,0.5])
#            glDrawArrays(GL_POLYGON,  4, 4)
            
#            glEnable(GL_TEXTURE_2D)
#            glBindTexture(GL_TEXTURE_2D, self.grating_texture)


            self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        #self._deinit_texture()
        #glDisable (GL_BLEND)
        
        
        #self.show_grating(duration = 10.0,  white_bar_width =1,velocity = 1.0,  color_contrast = 1.0,  color_offset = 0.5)

class TestStim(experiment.Stimulus):
    def configuration(self):
        self.DURATION=0.2
        
    def calculate_stimulus_duration(self):
        self.duration=self.DURATION
        
    def run(self):
        self.show_fullscreen(color=1.0,duration=self.DURATION)
        
class TestStim1(TestStim):
    def configuration(self):
        self.DURATION=0.3
        
class TestTimeIndexing(experiment.Stimulus):
    def run(self):
        size=100
        speeds=[1000]
        directions=[0,180]
        duration=1
        moving_duration=self.moving_shape_trajectory(size, speeds, directions,1,shape_starts_from_edge=True)[2]
        naturalbar_duration=self.show_natural_bars(speed = speeds[0], duration=duration, 
                                direction = directions[0], duration_calc_only=True)
        nframes=(duration*self.machine_config.SCREEN_EXPECTED_FRAME_RATE)
        from PIL import Image
        import os,shutil
        framesfolder='c:\\Data\\frames'
        if not os.path.exists(framesfolder):
            #shutil.rmtree(framesfolder)
            os.mkdir(framesfolder)
            for i in range(int(nframes)):
                frame=numpy.cast['uint8'](numpy.random.random((100,100,3))*255)
                frame[:,:,1]=frame[:,:,0]
                frame[:,:,2]=frame[:,:,0]
                frame[:,:,:]=100
                Image.fromarray(frame).save(os.path.join(framesfolder, 'f{0:0=4}.png'.format(i)))
        t0=time.time()
        self.moving_shape(size, speeds,directions,shape_starts_from_edge=True)
        t1=time.time()
        self.show_grating(duration,velocity=100, white_bar_width=200, duty_cycle=3)
        t2=time.time()
        self.show_natural_bars(speed = speeds[0], duration=duration, direction = directions[0])
        t3=time.time()
        self.show_white_noise(duration, 100)
        t4=time.time()
        dts=numpy.array([moving_duration-(t1-t0),duration-(t2-t1),duration-(t4-t3),naturalbar_duration-(t3-t2)])
        print(dts*1000)

        if 0:
            self.show_image(framesfolder)
            t5=time.time()

if __name__ == "__main__":
    pass
