import time
import numpy
import serial
import os.path
import os

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import visexpman
from visexpman.engine import visexp_runner
from visexpman.engine.vision_experiment.configuration import VisionExperimentConfig
from visexpman.engine.generic import utils
from visexpman.engine.vision_experiment import experiment
from visexpman.engine.hardware_interface import daq_instrument
from visexpman.users.test import unittest_aggregator
from visexpman.users.common import stimuli

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
        for rep in range(self.machine_config.REPEATS):
            if self.abort:
                break
            for directions in self.machine_config.DIRECTIONS:
                if self.abort:
                    break
                self.show_natural_bars(speed = self.machine_config.SPEED, repeats = 1, duration=self.machine_config.DURATION, minimal_spatial_period = None, spatial_resolution = self.machine_config.SCREEN_PIXEL_TO_UM_SCALE, intensity_levels = 255, direction = directions, save_frame_info =True, block_trigger = False)
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
        self.white_noise(duration = self.experiment_config.DURATION,
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
        self._create_parameters_from_locals(locals())
                                 
        
class DebugExperimentConfig(experiment.ExperimentConfig):
    def _create_parameters(self):
        self.runnable = 'Debug'
        self.DURATION = 10.0#Comment
#        self.pre_runnable = 'TestPre'
        self._create_parameters_from_locals(locals())

class Debug(experiment.Experiment):
    def prepare(self):
        self.experiment_duration = self.experiment_config.DURATION
    
    def run(self):
        self.show_shape(duration=0,size=10, color=1.0, part_of_drawing_sequence=True, flip=False)
        self.show_shape(duration=0,pos = utils.rc((10, 0)), size=30, color=0.7, part_of_drawing_sequence=True, flip=False)
        self.show_shape(duration=0,size=100, color=0.4, part_of_drawing_sequence=True, flip=True)
        time.sleep(self.experiment_duration)
        return
            
if __name__ == "__main__":
    if True:
        v = visexp_runner.VisionExperimentRunner('peter', 'StimulusDevelopment')
#        v.run_experiment(user_experiment_config = 'MovingGratingTuning')
        v.run_experiment(user_experiment_config = 'WhiteNoiseParameters')
    elif not True:
        v = visexp_runner.VisionExperimentRunner(['zoltan', 'chi'], 'SwDebugConfig')
        v.run_experiment(user_experiment_config = 'FullfieldSinewave')
    elif not True:
        v = visexp_runner.VisionExperimentRunner('antonia',  'MEASetup')
        v.run_experiment(user_experiment_config = 'WhiteNoiseParameters')
    elif True:
        v = visexp_runner.VisionExperimentRunner(['zoltan', 'chi'], 'SwDebugConfig')
        if True:
            v.run_loop()
        else:
            v.run_experiment(user_experiment_config = 'IncreasingAnnulusParameters')
    else:
        v = visexp_runner.VisionExperimentRunner('zoltan',  'SwDebugConfig')
        v.run_experiment(user_experiment_config = 'WhiteNoiseParameters')