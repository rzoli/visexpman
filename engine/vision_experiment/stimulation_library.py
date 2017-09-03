import pdb
import os.path
import os
import numpy
import math
import time
from PIL import Image
import inspect
import re
import multiprocessing
try:
    from OpenGL.GL import *
    from OpenGL.GLUT import *
    default_text=GLUT_BITMAP_TIMES_ROMAN_24
except ImportError:
    default_text=None
    print 'opengl not installed'
from contextlib import closing
import tables

import experiment_control
from visexpman.engine.generic import graphics,utils,colors,fileop, signal,geometry,videofile
from visexpman.engine.vision_experiment import screen
try:
    from visexpman.users.test import unittest_aggregator
    test_mode=True
except IOError:
    test_mode=False

import unittest

class Stimulations(experiment_control.StimulationControlHelper):#, screen.ScreenAndKeyboardHandler):
    """
    Contains all the externally callable stimulation patterns:
    1. show_image(self,  path,  duration = 0,  position = (0, 0),  formula = [])
    """
    def __init__(self, machine_config, parameters, queues, application_log):
        self.config=machine_config#TODO: eliminate self.config
        self._init_variables()
        #graphics.Screen constructor intentionally not called, only the very necessary variables for flip control are created.
        if hasattr(self, 'kwargs') and self.kwargs.has_key('screen') and self.kwargs['screen'] !=None:
            self.screen=self.kwargs['screen']
        else:
            self.screen = graphics.Screen(machine_config, init_mode = 'no_screen')
        experiment_control.StimulationControlHelper.__init__(self, machine_config, parameters, queues, application_log)
        if self.config.SCREEN_MODE != 'psychopy':
            self.grating_texture = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, self.grating_texture)
            glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        #Calculate axis factors
        if self.machine_config.VERTICAL_AXIS_POSITIVE_DIRECTION == 'up':
            self.vaf = 1
        else:
            self.vaf = -1
        if self.machine_config.HORIZONTAL_AXIS_POSITIVE_DIRECTION == 'right':
            self.haf = 1
        else:
            self.haf = -1
        self.frame_counter = 0
        self.stimulus_frame_info = []
        self.frame_rates = []
        
    def _init_variables(self):
        self.delayed_frame_counter = 0 #counts how many frames were delayed
        self.log_on_flip_message = ''
        self.text_on_stimulus = []
        #Command buffer for keyboard commands during experiment
        self.keyboard_commands = multiprocessing.Queue()
        
    def _flip(self, frame_timing_pulse, count = True):
        """
        Flips screen buffer. Additional operations are performed here: saving frame and generating trigger
        """
        current_texture_state = glGetBooleanv(GL_TEXTURE_2D)
        if current_texture_state:
            glDisable(GL_TEXTURE_2D)
        self._show_text()
        if current_texture_state:
            glEnable(GL_TEXTURE_2D)
        self.screen.flip()
        if frame_timing_pulse and not self.config.STIMULUS2MEMORY:
            self._frame_timing_pulse()
        if count:
            self.frame_counter += 1
        if not self.config.STIMULUS2MEMORY:
            # If this library is not called by an experiment class which is called form experiment control class, no logging shall take place
            self.frame_rates.append(self.screen.frame_rate)
        self.check_abort()
        
    def _get_frame_index(self):
        if not hasattr(self, 't0'):
            return
        dt=time.time()-self.t0
        return int(round(dt*self.config.SCREEN_EXPECTED_FRAME_RATE))

    def _save_stimulus_frame_info(self, caller_function_info, is_last = False,parameters=None):
        '''
        Saves:
        -frame counter
        -elapsed time
        -stimulus function's name
        -parameters of stimulus
        '''
        args, _, _, values = inspect.getargvalues(caller_function_info)
        caller_name =inspect.getframeinfo(caller_function_info)[2]
        frame_info = {}
        frame_info['counter'] = self.frame_counter
        #Removed: when 1 frame time stimulus is presented, the last and first enrties will have the same counter value and from this the duration cannot be consistently calculated
        #if is_last: #ensures that there are not stimulus_frame_info entries with the same counter value
        #    frame_info['counter']  -= 1
        frame_info['stimulus_type'] = caller_name
        frame_info['is_last'] = is_last
        frame_info['parameters'] = {}
        for arg in args:
            if arg != 'self':
                if values[arg] is None:
                    frame_info['parameters'][arg] = ''
                else:
                    frame_info['parameters'][arg] = values[arg]
        if hasattr(parameters,'has_key'):
            frame_info['parameters'].update(parameters)
        self.stimulus_frame_info.append(frame_info)
        if is_last and self.stimulus_frame_info[-2].has_key('counter') and self.stimulus_frame_info[-1]['counter']<self.stimulus_frame_info[-2]['counter']:
            raise RuntimeError('frame counter value cannot decrease: {0}, {1}'.format(*self.stimulus_frame_info[-2:]))
            
    def _append_to_stimulus_frame_info(self,values):
        self.stimulus_frame_info[-1]['parameters'].update(values)
            
    def trigger_pulse(self, pin, width,polarity=True):
        '''
        Generates trigger pulses
        '''
        if hasattr(self.digital_output,'set_data_bit'):
            self.digital_output.set_data_bit(pin, int(polarity), log = False)
            time.sleep(width)
            self.digital_output.set_data_bit(pin, int(not polarity), log = False)

    def _frame_timing_pulse(self):
        '''
        Generates frame trigger pulses
        '''
        if self.config.FRAME_TRIGGER_PIN!=-1:
            self.trigger_pulse(self.config.FRAME_TRIGGER_PIN, self.config.FRAME_TRIGGER_PULSE_WIDTH)
            
    def block_start(self, block_name = 'stimulus function'):
        if hasattr(self.digital_output,'set_data_bit'):
            self.digital_output.set_data_bit(self.config.BLOCK_TRIGGER_PIN, 1, log = False)
        self.stimulus_frame_info.append({'block_start':self.frame_counter, 'block_name': block_name})
        if self.machine_config.PLATFORM == 'elphys_retinal_ca':
            self.send({'plot': [time.time(), 1]})
        if hasattr(self.log, 'info'):
            self.log.info('{0} block started' .format(block_name), source='stim')
                
    def block_end(self, block_name = 'stimulus function'):
        if hasattr(self.digital_output,'set_data_bit'):
            self.digital_output.set_data_bit(self.config.BLOCK_TRIGGER_PIN, 0, log = False)
        self.stimulus_frame_info.append({'block_end':self.frame_counter, 'block_name': block_name})
        if self.machine_config.PLATFORM == 'elphys_retinal_ca':
            self.send({'plot': [time.time(), 0]})
        if hasattr(self.log, 'info'):
            self.log.info('{0} block ended' .format(block_name), source='stim')
        
    def _add_block_start(self, is_block, frame_i, nframes):
        if frame_i == 0 and is_block:
            self.block_start()
            
    def _add_block_end(self, is_block, frame_i, nframes):
        if frame_i == nframes - 1 and is_block:
            self.block_end()
            
    def draw(self):
        '''
        This method is called after drawing a stimulus, User can overdefine this function with drawing a mask, additional object etc
        '''
        
    def _show_text(self):
        '''
        Overlays on stimulus all the added text configurations
        '''
        if self.config.ENABLE_TEXT:
            for text_config in self.text_on_stimulus:
                if text_config['enable']:
                    self.screen.render_text(text_config['text'], color = text_config['color'], position = text_config['position'],  text_style = text_config['text_style'])
                    
    def _get_shape_string(self, shape):
        if shape == 'circle' or shape == '' or shape == 'o' or shape == 'c' or shape =='spot':
            shape_type = 'circle'
        elif shape == 'rect' or shape == 'rectangle' or shape == 'r' or shape == '||':
            shape_type = 'rectangle'
        elif shape == 'annuli' or shape == 'annulus' or shape == 'a':
            shape_type = 'annulus'
        return shape_type
    
    #== Public, helper functions ==
    def set_background(self,  color):
        '''
        Set background color. Call this when a visual pattern should have a different background color than config.BACKGROUND_COLOR
        '''
        color_to_set = colors.convert_color(color, self.config)
        glClearColor(color_to_set[0], color_to_set[1], color_to_set[2], 0.0)
        
    def add_text(self, text, color = (1.0,  1.0,  1.0), position = utils.rc((0.0, 0.0)),  text_style = default_text):
        '''
        Adds text to text list
        '''
        text_config = {'enable' : True, 'text' : text, 'color' : colors.convert_color(color, self.config), 'position' : position, 'text_style' : text_style}
        self.text_on_stimulus.append(text_config)
        
    def change_text(self, id, enable = None, text = None, color = None, position = None,  text_style = None):
        '''
        Changes the configuration of the text pointed by id. id is the index of the text_configuration in the text_on_stimulus stimulus list
        '''
        text_config = self.text_on_stimulus[id]
        if enable != None:
            text_config['enable'] = enable
        if text != None:
            text_config['text'] = text
        if color != None:
            text_config['color'] = colors.convert_color(color, self.config)
        if position != None:
            text_config['position'] = position
        if text_style != None:
            text_config['text_style'] = text_style        
        self.text_on_stimulus[id]
        
    def disable_text(self, id = None):
        '''
        Disables the configuration of the text pointed by id. id is the index of the text_configuration in the text_on_stimulus stimulus list
        '''
        if id == None:
            index = -1
        else:
            index = id
        self.text_on_stimulus[index]['enable'] = False

    #== Various visual patterns ==
    
    def show_fullscreen(self, duration = 0.0,  color = None, flip = True, count = True, 
                is_block = False, save_frame_info = True, frame_timing_pulse = True):
        '''
        Show a fullscreen simulus where color is the color of the screen. 
            duration: duration of stimulus, 0.0: one frame time, -1.0: forever, 
                    any other value is interpreted in seconds        
        '''
        if color == None:
            color_to_set = self.config.BACKGROUND_COLOR
        else:
            color_to_set = colors.convert_color(color, self.config)
        if count and save_frame_info:
            self.log.info('show_fullscreen(' + str(duration) + ', ' + str(color_to_set) + ')', source='stim')
            self._save_stimulus_frame_info(inspect.currentframe())
        self.screen.clear_screen(color = color_to_set)
        if duration == 0.0:
            if flip:
                self._flip(frame_timing_pulse = frame_timing_pulse, count = count)
        elif duration == -1.0:
            i = 0
            while not self.abort:
                if i == 1:
                    self.screen.clear_screen(color = color_to_set)
                if flip:
                    self._flip(frame_timing_pulse = True, count = count)
                i += 1
        else:
            nframes = int(duration * self.config.SCREEN_EXPECTED_FRAME_RATE)
            for i in range(nframes):
                if i == 1:
                    self.screen.clear_screen(color = color_to_set)
                if flip:
                    self._add_block_start(is_block, i, nframes)
                    self._flip(frame_timing_pulse = frame_timing_pulse, count = count)
                    self._add_block_end(is_block, i, nframes)
                if self.abort:
                    break
        #set background color to the original value
        glClearColor(self.config.BACKGROUND_COLOR[0], self.config.BACKGROUND_COLOR[1], self.config.BACKGROUND_COLOR[2], 0.0)
        if count and save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
                
    def show_image(self,  path,  duration = 0,  position = utils.rc((0, 0)),  stretch=1.0, 
            flip = True, is_block = False):
        '''        
        Three use cases are handled here:
            - showing individual image files
                duration: duration of showing individual image file
                path: path of image file
            - showing the content of a folder
                duration: duration of showing each image file in folder
                path: path of folder containing images
            - path is a hdf5 file containing a 3d numpy array which is loaded frame by frame
        Image is shown for one frame time if duration is 0.
        Further parameters:
            position: position of image on screen in pixels.
            stretch: stretch of image, 1 means no scaling, 0.5 means half size
        Example:
            Show a single image  for 1 second in a centered position:
                self.show_image('c:\\images\\frame.png',  duration = 1.0,  position = (0, 0))
            Play the content of a directory (directory_path) which contains image files. 
            Each image is shown for one frame time:
                show_image('c:\\images',  duration = 0,  position = (0, 0))
        '''
        #Generate log messages
        flips_per_frame = duration/(1.0/self.config.SCREEN_EXPECTED_FRAME_RATE)
        if flips_per_frame != numpy.round(flips_per_frame):
            raise RuntimeError('This duration is not possible, it should be the multiple of 1/SCREEN_EXPECTED_FRAME_RATE')                
        self.log.info('show_image(' + str(path)+ ', ' + str(duration) + ', ' + str(position) + ', ' + str(stretch)  + ', ' + ')', source='stim')
        self._save_stimulus_frame_info(inspect.currentframe())
        if os.path.isdir(path):
            fns = os.listdir(path)
            fns.sort()
            if len([f for f in fns if os.path.splitext(f)[1] not in ['.png', '.bmp', '.jpg']])>0:
                 raise RuntimeError('{0} folder contains non image files, please remove them!'.format(path))
            if is_block:
                self.block_start()
            self.t0=time.time()
            for i in range(len(fns)):
                if self.machine_config.ENABLE_TIME_INDEXING:
                    index=self._get_frame_index()
                else:
                    index=i
                fn=fns[index]
                self._show_image(os.path.join(path,fn),duration,position,stretch,flip,is_block=False)
            self.screen.clear_screen()
            self._flip(frame_timing_pulse = True)
            if is_block:
                self.block_end()
        elif os.path.isfile(path) and os.path.splitext(path)=='.hdf5': # a hdf5 file with stimulus_frames variable having nframes * x * y dimensions
            if self.machine_config.ENABLE_TIME_INDEXING:
                raise NotImplementedError()
            with closing(tables.open_file(path,'r')) as handler:
                full_chunk = 0
                if full_chunk: # read big chunk
                    allframedata = handler.root.stimulus_frames[:5000].astype(float)/255
                if is_block:
                    self.block_start()
                for f1i in range(handler.root.stimulus_frames.shape[0]):
                    mytime = time.time()
                    if full_chunk:
                        framedata = allframedata[f1i]
                    else:
                        framedata = handler.root.stimulus_frames[f1i].astype(float)/255  # put actual image frames into the list of paths
                    if self.config.VERTICAL_AXIS_POSITIVE_DIRECTION == 'down':
                        framedata = framedata[::-1]  # flip row order = flip image TOP to Bottom
                    self._show_image(numpy.rollaxis(numpy.tile(framedata,(3,1,1)),0,3), duration, position, stretch, flip, is_block=False)
                    print(1./(time.time() - mytime))
                    if self.abort:
                        break
                self.screen.clear_screen()
                self._flip(frame_timing_pulse=True)
                if is_block:
                    self.block_end()
        else:
            self._show_image(path,duration,position,stretch,flip,is_block)
        self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
        
    def _show_image(self,path,duration,position,stretch,flip,is_block):
        if duration == 0.0:
            nframes=1
        else:
            nframes = int(duration * self.config.SCREEN_EXPECTED_FRAME_RATE)
        for i in range(nframes):
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            if hasattr(path,'shape'):  # show image already loaded
                self.screen.render_image(path, position=utils.rc_add(position, self.machine_config.SCREEN_CENTER),
                                             stretch=stretch)
            else:  # load image file given its path as a string
                self.screen.render_imagefile(path, position = utils.rc_add(position, self.machine_config.SCREEN_CENTER),stretch=stretch)
            if flip:
                self._add_block_start(is_block, i, nframes)
                self._flip(frame_timing_pulse = True)
                self._add_block_end(is_block, i, nframes)
            if self.abort:
                break

    def show_shape(self, shape = '',  duration = 0.0,  pos = utils.rc((0,  0)),  color = [1.0,  1.0,  1.0],  
                background_color = None,  orientation = 0.0,  size = utils.rc((0,  0)),  ring_size = None, 
                ncorners = None, inner_radius = None, L_shape_config = None, X_shape_angle = None,
                flip = True, is_block = False, save_frame_info = True, enable_centering = True, 
                part_of_drawing_sequence = False,angle=None):
        '''
        Shows simple, individual shapes like rectangle, circle or ring.
            shape: 'spot', 'rectangle', 'annulus', 'triangle', 'star'
            duration: duration in seconds, 0 means 1 frame time
            pos: position(s) of stimulus in row/column recarray format. Its dimension is micormeter on retina. 
                    By default 0,0 is te center of the screen but COORDINATE_SYSTEM 
                    shall be checked in machine config. If multiple values are provided, the object will be 
                    displayed at each position for one frame time, duration parameter is overridden. 
                    This can be used for presenting a moving object.                    
            color: color of a the displayed object. If 2d numpy array is provided, the object is presented 
                    with each color for one frame time. An array of colors can be used to generate 
                    a flickering object
            background_color: background color. If None, the system default background color is used.
            orientation: orienatation of object in degrees
            size: size of stimulus in row/column recarray format in micrometer. If multiple values are provided, 
                    the object will be displayed with each sizes for one frame time, duration parameter is 
                    overridden. Looming stimulus can be generated by providing and array of sizes.
            enable_centering: object position will be shifted to preset screen center
            L_shape_config: dictionary of shorter_side, longer_side lengths, shorter_position: start, middle, end, 
                    angle = 45, 90, 135 and width of shape
        '''
        if save_frame_info:
            self.log.info('show_shape(' + str(shape)+ ', ' + str(duration) + ', ' + str(pos) + ', ' + str(color)  + ', ' + str(background_color)  + ', ' + str(orientation)  + ', ' + str(size)  + ', ' + str(ring_size) + ')', source = 'stim')
            self._save_stimulus_frame_info(inspect.currentframe())
        #Calculate number of frames
        n_frames = int(float(duration) * float(self.config.SCREEN_EXPECTED_FRAME_RATE))
        if n_frames == 0:
            n_frames = 1
        if hasattr(color,  'shape') and len(color.shape) ==2:
            n_frames = color.shape[0]
        #convert um dimension parameters to pixel
        if isinstance(size, float) or isinstance(size, int):        
            size_pixel = utils.rc((size, size))        
        elif isinstance(size, numpy.ndarray):   
            size_pixel = size
        else:
            raise RuntimeError('Parameter size is provided in an unsupported format')
        size_pixel = utils.rc_x_const(size_pixel, self.config.SCREEN_UM_TO_PIXEL_SCALE)
        if hasattr(self, 'screen_center') and enable_centering:
            pos_with_offset = utils.rc_add(pos, self.screen_center)
        else:
            pos_with_offset = pos
        pos_pixel = utils.rc_x_const(pos_with_offset, self.config.SCREEN_UM_TO_PIXEL_SCALE)
        if ring_size is not None:
            ring_size_pixel = ring_size * self.config.SCREEN_UM_TO_PIXEL_SCALE
        #Calculate vertices
        points_per_round = 360
        if shape == 'circle' or shape == '' or shape == 'o' or shape == 'c' or shape =='spot':
            shape_type = 'circle'
            vertices = geometry.circle_vertices([size_pixel['col'],  size_pixel['row']],  resolution = points_per_round / 360.0)#resolution is vertex per degree
        elif shape == 'rect' or shape == 'rectangle' or shape == 'r' or shape == '||':
            vertices = geometry.rectangle_vertices(size_pixel, orientation = orientation)
            shape_type = 'rectangle'
        elif shape == 'annuli' or shape == 'annulus' or shape == 'a':
            vertices_outer_ring = geometry.circle_vertices([size_pixel['col'],  size_pixel['row']],  resolution = points_per_round / 360.0)#resolution is vertex per degree
            vertices_inner_ring = geometry.circle_vertices([size_pixel['col'] - 2*ring_size_pixel,  size_pixel['row'] - 2*ring_size_pixel],  resolution = points_per_round / 360.0)#resolution is vertex per degree
            vertices = numpy.zeros(shape = (vertices_outer_ring.shape[0] * 2, 2))
            vertices[:vertices_outer_ring.shape[0]] = vertices_outer_ring
            vertices[vertices_outer_ring.shape[0]:] = vertices_inner_ring
            shape_type = 'annulus'
        elif shape == 'triangle':
            vertices = geometry.triangle_vertices(size_pixel['row'], orientation)
            shape_type = shape
        elif shape == 'star':
            vertices_ = geometry.star_vertices(size_pixel['row'],ncorners,orientation,inner_radius)
            #make triangles of shape
            vertices = numpy.zeros((ncorners*3,2))
#            import pdb
#            pdb.set_trace()
            vertices[1::3] = vertices_[0::2]
            vertices[0::3] = numpy.roll(vertices_[1::2],1,axis=0)
            vertices[2::3] = vertices_[1::2]
            vertices = numpy.concatenate((vertices, vertices_[1::2]))
            shape_type = shape
        elif shape == 'X':
            shape_type = shape
            vertices = numpy.concatenate([geometry.rectangle_vertices(size_pixel, orientation = orientation),
                    geometry.rectangle_vertices(size_pixel, orientation = orientation+X_shape_angle)])
        elif shape == 'L':
            shape_type = shape
            #create skeleton
            if L_shape_config['shorter_position'] == 'middle':
                p=geometry.point_coordinates(L_shape_config['shorter_side'], numpy.radians(L_shape_config['angle']), numpy.array([0,0]))
                v = numpy.array([[-0.5*L_shape_config['longer_side'], 0], [0,0], [0.5*L_shape_config['longer_side'],0],p])
            elif L_shape_config['shorter_position'] == 'start':
                start_point = numpy.array([-0.5*L_shape_config['longer_side'],0])
                p=geometry.point_coordinates(L_shape_config['shorter_side'], numpy.radians(L_shape_config['angle']), start_point)
                v=numpy.array([[0.5*L_shape_config['longer_side'],0], start_point, p])
            elif L_shape_config['shorter_position'] == 'end':
                start_point = numpy.array([0.5*L_shape_config['longer_side'],0])
                p=geometry.point_coordinates(L_shape_config['shorter_side'], numpy.radians(L_shape_config['angle']), start_point)
                v=numpy.array([[-0.5*L_shape_config['longer_side'],0], start_point, p])
            if L_shape_config['shorter_position'] == 'middle':
                wrist_distance = 0.5* L_shape_config['width']/numpy.sin(numpy.radians(45))
                base_shape = numpy.array([geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(90), v[0]), 
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(-90), v[0]),
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(-90), v[2]),
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(90), v[2])])
                angle = numpy.radians(L_shape_config['angle'])
                endpoints = numpy.array([geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(L_shape_config['angle']+90), v[3]),
                                                        geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(L_shape_config['angle']-90), v[3]),
                                                        ])
                wrist = numpy.array([
                                            geometry.point_coordinates(L_shape_config['shorter_side'], angle-numpy.pi, endpoints[1]),
                                            geometry.point_coordinates(L_shape_config['shorter_side'], angle-numpy.pi, endpoints[0])
                                            ])
                vertices = numpy.concatenate((base_shape, wrist, endpoints))
            elif L_shape_config['shorter_position'] == 'start':
                wrist_distance = 0.5* L_shape_config['width']/numpy.sin(numpy.radians(L_shape_config['angle']*0.5))
                endvertices1 = numpy.array([geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(90), v[0]), 
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(-90), v[0])])
                endvertices2 = numpy.array([geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(L_shape_config['angle']-90), v[2]), 
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(L_shape_config['angle']+90), v[2])])
                angle = numpy.radians((L_shape_config['angle']*0.5))
                wrist = numpy.array([geometry.point_coordinates(wrist_distance, angle - numpy.pi, v[1]),
                                                    geometry.point_coordinates(wrist_distance, angle, v[1])])
                vertices = numpy.concatenate((endvertices1, wrist, wrist, endvertices2))
            elif L_shape_config['shorter_position'] == 'end':
                wrist_distance = 0.5* L_shape_config['width']/numpy.sin(numpy.radians((180-L_shape_config['angle'])*0.5))
                endvertices1 = numpy.array([geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(90), v[0]), 
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(-90), v[0])])
                endvertices2 = numpy.array([geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(L_shape_config['angle']-90), v[2]), 
                                                                geometry.point_coordinates(0.5*L_shape_config['width'], numpy.radians(L_shape_config['angle']+90), v[2])])
                angle = numpy.radians(((180-L_shape_config['angle'])*0.5)+L_shape_config['angle'])
                wrist = numpy.array([geometry.point_coordinates(wrist_distance, angle- numpy.pi, v[1]),
                                                    geometry.point_coordinates(wrist_distance, angle, v[1])])
                vertices = numpy.concatenate((endvertices1, wrist, wrist[::-1], endvertices2))
            vertices = geometry.rotate_point(vertices.T,orientation,numpy.array([0,0])).T
            #Convert to pixels
            vertices *= self.config.SCREEN_UM_TO_PIXEL_SCALE
        n_vertices = vertices.shape[0]
        if len(pos_pixel.shape) == 0:#When does it happen?????????????
            number_of_positions = 1
            vertices = vertices + numpy.array([pos_pixel['col'], pos_pixel['row']])
        elif len(pos_pixel.shape) == 1 and pos_pixel.shape[0] == 1:
            number_of_positions = 1
            vertices = vertices + numpy.array([pos_pixel['col'], pos_pixel['row']]).T
        elif len(pos_pixel.shape) == 1 and pos_pixel.shape[0] > 1:
            if shape_type == 'annulus':
                raise RuntimeError('Moving annulus stimulus is not supported')
            else:
                n_frames = pos_pixel.shape[0]
                number_of_positions = pos_pixel.shape[0]
                packed_vertices = numpy.zeros((number_of_positions * n_vertices, 2), dtype = numpy.float64)
                for i in range(number_of_positions):
                    packed_vertices[i * n_vertices: (i+1) * n_vertices, :] = vertices + numpy.array([pos_pixel['col'][i], pos_pixel['row'][i]])
                vertices = packed_vertices
        #Set color
        if hasattr(color,  'shape') and len(color.shape) ==2:
            glColor3fv(colors.convert_color(color[0], self.config))
        else:
            glColor3fv(colors.convert_color(color, self.config))
        if background_color != None:
            background_color_saved = glGetFloatv(GL_COLOR_CLEAR_VALUE)
            converted_background_color = colors.convert_color(background_color, self.config)
            glClearColor(converted_background_color[0], converted_background_color[1], converted_background_color[2], 0.0)
        else:
            converted_background_color = colors.convert_color(self.config.BACKGROUND_COLOR, self.config)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(vertices)
        frame_i = 0
        self.t0=time.time()
        while True:
            if not part_of_drawing_sequence:
                glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            if shape_type != 'annulus':
                if hasattr(color,  'shape') and len(color.shape) == 2:
                    glColor3fv(colors.convert_color(color[frame_i], self.config))
                if number_of_positions == 1:
                    if shape_type == 'star':#opengl cannot draw concave shapes
                        for i in range(ncorners):
                            glDrawArrays(GL_POLYGON,  i*3, 3)
                        glDrawArrays(GL_POLYGON,  ncorners*3, ncorners)
                    elif shape_type == 'L' or shape_type == 'X':
                        glDrawArrays(GL_POLYGON,  0, n_vertices/2)
                        glDrawArrays(GL_POLYGON,  n_vertices/2, n_vertices/2)
                    else:
                        glDrawArrays(GL_POLYGON,  0, n_vertices)
                else:
                    if shape_type == 'star':
                        raise NotImplementedError('moving star is not implemented')
                    elif shape_type == 'L' or shape_type == 'X':
                        glDrawArrays(GL_POLYGON,  frame_i * n_vertices, n_vertices/2)
                        glDrawArrays(GL_POLYGON,  int((frame_i+0.5) * n_vertices), n_vertices/2)
                    else:
                        glDrawArrays(GL_POLYGON,  frame_i * n_vertices, n_vertices)
            else:
                n = int(n_vertices/2)
                glColor3fv(converted_background_color)
                glDrawArrays(GL_POLYGON,  n, n)
                if hasattr(color,  'shape') and len(color.shape) ==2:
                    glColor3fv(colors.convert_color(color[frame_i], self.config))
                else:
                    glColor3fv(colors.convert_color(color, self.config))
                glDrawArrays(GL_POLYGON,  0, n)
            if flip:
                self._add_block_start(is_block, frame_i, n_frames)
                self._flip(frame_timing_pulse = True)
                self._add_block_end(is_block, frame_i, n_frames)
            if self.abort:
                break
            if self.machine_config.ENABLE_TIME_INDEXING:
                frame_i=self._get_frame_index()
            else:
                frame_i += 1
            if duration != -1 and frame_i == n_frames:
                break
        glDisableClientState(GL_VERTEX_ARRAY)
        #Restore original background color
        if background_color != None:            
            glClearColor(background_color_saved[0], background_color_saved[1], background_color_saved[2], background_color_saved[3])
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)

    def show_checkerboard(self, n_checkers, duration = 0.0, pos = utils.cr((0,  0)), color = [], 
            box_size = utils.cr((0,  0)), background_color = None, flip = True, save_frame_info = True,
            block_trigger=False):
        '''
        Shows checkerboard:
            n_checkers = (x dir (column), y dir (rows))
            pos - position of display area in um
            box_size - size of a box in um
            duration - duration of displaying each pattern in seconds
            color - array of color values. Dimensions:
                            1. Frame
                            2. row
                            3. col
                            4. color channel
        '''
        raise NotImplementedError('block handling and trigger generation is not implemented')
        self.log.info('show_checkerboard(' + str(n_checkers)+ ', ' + str(duration) +', ' + str(box_size) +')',source='stim')
        first_flip = False
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe())
        nframes = color.shape[0]
        nshapes = n_checkers['row']*n_checkers['col']
        if not hasattr(box_size, 'dtype'):
            box_size = utils.rc((box_size, box_size))
        shape_size = numpy.array([numpy.ones(nshapes)*box_size['row'], numpy.ones(nshapes)*box_size['col']]).T
        grid_positions = numpy.array(numpy.meshgrid(numpy.linspace(-(n_checkers['row']-1)/2.0, (n_checkers['row']-1)/2.0, n_checkers['row'])*box_size['row'], numpy.linspace(-(n_checkers['col']-1)/2.0, (n_checkers['col']-1)/2.0, n_checkers['col'])*box_size['col']),dtype=numpy.float).T
        shape_positions = utils.rc(numpy.array([numpy.array(grid_positions[:,:,0].flatten().tolist()), numpy.array(grid_positions[:,:,1].flatten().tolist())]))
        color_adjusted = color[:,::-1,:,:]
        self.show_shapes('rectangle', shape_size, shape_positions, nshapes, 
                    duration = duration, 
                    color = numpy.reshape(color_adjusted.flatten(), (color_adjusted.shape[0], color_adjusted.shape[1]*color_adjusted.shape[2],color_adjusted.shape[3])), 
                    background_color = background_color,
                    block_trigger = block_trigger, colors_per_shape = False, 
                    are_same_shapes_over_frames = True, 
                    save_frame_info = False)
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
                    
    def show_grating(self, duration = 0.0,  profile = 'sqr',  white_bar_width =-1,  
                    display_area = utils.cr((0,  0)),  orientation = 0,  starting_phase = 0.0,  
                    velocity = 0.0,  color_contrast = 1.0,  color_offset = 0.5,  pos = utils.cr((0, 0)),  
                    duty_cycle = 1.0,  noise_intensity = 0, flicker=None, phases=[],
                    part_of_drawing_sequence = False, is_block = False, save_frame_info = True):
        """
        This stimulation shows grating with different color (intensity) profiles.
            - duration: duration of stimulus in seconds
            - profile: shape of grating color (intensity) profile, the followings are possible:
                - 'sqr': square shape, most common grating profile
                - 'tri': triangle profile
                - 'saw': sawtooth profile
                - 'sin': sine
                - 'cos': cosine
                profile parameter can be a list of these keywords. Then different profiles are applied to each 
                color channel
            - white_bar_width: length of one bar in um
            - display area: by default the whole screen but it can be confined to a smaller surface. 
                    In fullscreen mode, the first bar in the grating pattern may not be at the edge of the screen
            - pos: position of display area            
            - orientation: orientation of grating in degrees
            - starting_phase: starting phase of stimulus in degrees
            - velocity: velocity of the movement of grating in um/s
            - color_contrast: color contrast of grating stimulus.
            - color_offset: color (intensity) offset of stimulus.
            - duty_cycle: duty cycle of grating stimulus with sqr profile. Its interpretation is 
                            different from the usual: period = (bar_width * (1.0 + duty_cycle). 
                            For a 50% black and white the duty_cycle value should be 1.0
            - noise_intensity: Maximum contrast of random noise mixed to the stimulus.
            - flicker = {'frequency':,'modulation_size'}: 
                        grating flickering frequency. Grating pattern is modulated with Modulation Size
        
        Usage examples:
        1) Show a simple, fullscreen, grating stimuli for 3 seconds with 45 degree orientation
            show_grating(duration = 3.0, orientation = 45, velocity = 100, white_bar_width = 100)
        2) Show grating with sine profile on a 500x500 area with 10 degree starting phase
            show_grating(duration = 3.0, profile = 'sin', display_area = (500, 500), starting_phase = 10, 
                        velocity = 100, white_bar_width = 200)
        3) Show grating with sawtooth profile on a 500x500 area where the color contrast is 
                light red and the color offset is light blue
            show_grating(duration = 3.0, profile = 'saw', velocity = 100, white_bar_width = 200, 
                    color_contrast = [1.0,0.0,0.0], color_offset = [0.0,0.0,1.0]) 
        """
        if white_bar_width == -1:
            bar_width = self.config.SCREEN_RESOLUTION['col'] * self.config.SCREEN_UM_TO_PIXEL_SCALE
        else:
            bar_width = white_bar_width * self.config.SCREEN_UM_TO_PIXEL_SCALE
        #== Logging ==
        self.log.info('show_grating(' + str(duration)+ ', ' + str(profile) + ', ' + str(white_bar_width) + ', ' + str(display_area)  + ', ' + str(orientation)  + ', ' + str(starting_phase)  + ', ' + str(velocity)  + ', ' + str(color_contrast)  + ', ' + str(color_offset) + ', ' + str(pos)  + ')',source='stim')
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe())
        
        #== Prepare ==
        orientation_rad = orientation * math.pi / 180.0
        if self.config.VERTICAL_AXIS_POSITIVE_DIRECTION == 'down':
            orientation_rad *= -1
        if self.config.HORIZONTAL_AXIS_POSITIVE_DIRECTION == 'down':
            pass
            
        pos_transformed = utils.rc_x_const(pos, self.config.SCREEN_UM_TO_PIXEL_SCALE)
        
        pos_adjusted = []
        pos_adjusted.append(pos_transformed['col'])
        pos_adjusted.append(pos_transformed['row'])
        if display_area['col'] == 0 and display_area['row'] == 0 and self.config.COORDINATE_SYSTEM == 'ulcorner':
            pos_adjusted[0] += 0.5 * self.config.SCREEN_RESOLUTION['col']
            pos_adjusted[1] += 0.5 * self.config.SCREEN_RESOLUTION['row']            
        
        if display_area['col'] == 0:
            screen_width = self.config.SCREEN_RESOLUTION['col']
        else:
            screen_width = display_area['col']
        display_area_adjusted = []
        display_area_adjusted.append(display_area['col'] * self.config.SCREEN_UM_TO_PIXEL_SCALE)
        display_area_adjusted.append(display_area['row'] * self.config.SCREEN_UM_TO_PIXEL_SCALE)
        display_area_adjusted = numpy.array(display_area_adjusted)        
        
        #If grating are to be shown on fullscreen, modify display area so that no ungrated parts are on the screen considering rotation
        if display_area_adjusted[0] == 0:            
            display_area_adjusted[0] = self.config.SCREEN_RESOLUTION['col'] * abs(math.cos(orientation_rad)) + self.config.SCREEN_RESOLUTION['row'] * abs(math.sin(orientation_rad))
            screen_width = self.config.SCREEN_RESOLUTION['col']
            display_area_adjusted[0] = numpy.sqrt(2)*self.config.SCREEN_RESOLUTION['col']
        if display_area_adjusted[1] == 0:
            display_area_adjusted[1] = self.config.SCREEN_RESOLUTION['row'] * abs(math.cos(orientation_rad)) + self.config.SCREEN_RESOLUTION['col'] * abs(math.sin(orientation_rad))        
            display_area_adjusted[1] = numpy.sqrt(2)*self.config.SCREEN_RESOLUTION['col']
        #calculate vertices of display area
        #angles between diagonals
        alpha = numpy.arctan(display_area_adjusted[1]/display_area_adjusted[0])
        angles = numpy.array([alpha, numpy.pi - alpha, alpha + numpy.pi, -alpha])
        angles = angles + orientation_rad
        diagonal = numpy.sqrt((display_area_adjusted **2).sum())
        vertices = 0.5 * diagonal * numpy.array([numpy.cos(angles), numpy.sin(angles)])
        vertices = vertices.transpose()
        vertices = vertices + numpy.array([pos_adjusted])
        #glEnableClientState(GL_VERTEX_ARRAY)
        #glVertexPointerf(vertices)
        #== Generate grating profile
        if isinstance(profile, str):
            profile_adjusted = [profile,  profile,  profile]            
        else:
            profile_adjusted = []
            for profile_i in profile:
                profile_adjusted.append(profile_i)
        #contrast and offset can be provided in rgb or intensity. For both the accepted range is 0...1.0
        if not isinstance(color_contrast, list) and not isinstance(color_contrast, tuple):
            color_contrast_adjusted = [color_contrast,  color_contrast,  color_contrast]
        else:
            color_contrast_adjusted = []
            for color_contrast_i in color_contrast:
                color_contrast_adjusted.append(color_contrast_i)
        if not isinstance(color_offset, list) and not isinstance(color_offset, tuple):
            color_offset_adjusted = [color_offset,  color_offset,  color_offset]
        else:
            color_offset_adjusted = []
            for color_offset_i in color_offset:
                color_offset_adjusted.append(color_offset_i)
        #calculate grating profile period from spatial frequency
        period = int(bar_width * (1.0 + duty_cycle))
        #modify profile length so that the profile will contain integer number of repetitions
        repetitions = numpy.ceil(display_area_adjusted[0]/period)
        profile_length = period * repetitions
        cut_off_ratio = display_area_adjusted[0]/profile_length
        profile_length = int(profile_length)
        waveform_duty_cycle = 1.0 / (1.0 + duty_cycle)#???/cut_off_ratio
        stimulus_profile_r = utils.generate_waveform(profile_adjusted[0], profile_length, period, color_contrast_adjusted[0], color_offset_adjusted[0], starting_phase, waveform_duty_cycle)
        stimulus_profile_g = utils.generate_waveform(profile_adjusted[1], profile_length, period, color_contrast_adjusted[1], color_offset_adjusted[1], starting_phase, waveform_duty_cycle)
        stimulus_profile_b = utils.generate_waveform(profile_adjusted[2], profile_length, period, color_contrast_adjusted[2], color_offset_adjusted[2], starting_phase, waveform_duty_cycle)
        stimulus_profile = numpy.array([[stimulus_profile_r], [stimulus_profile_g], [stimulus_profile_b]])
        stimulus_profile = stimulus_profile.transpose()
        if hasattr(self.config, 'GAMMA_CORRECTION'):
            stimulus_profile = self.config.GAMMA_CORRECTION(stimulus_profile)
        if hasattr(self.config, 'COLOR_MASK'):
            stimulus_profile *= self.config.COLOR_MASK
        if duration == 0.0:
            n_frames = 1
        else:
            n_frames = int(float(duration) * float(self.config.SCREEN_EXPECTED_FRAME_RATE))
        ######### Calculate texture phase shift per frame value ######
        if hasattr(velocity, 'dtype') and len(velocity.shape)>0:
            pixel_velocities = -velocity * self.config.SCREEN_UM_TO_PIXEL_SCALE / float(self.config.SCREEN_EXPECTED_FRAME_RATE) / float(stimulus_profile.shape[0])
        else:
            pixel_velocity = -velocity * self.config.SCREEN_UM_TO_PIXEL_SCALE / float(self.config.SCREEN_EXPECTED_FRAME_RATE) / float(stimulus_profile.shape[0])
            pixel_velocities = numpy.ones(n_frames)*pixel_velocity
        pixel_velocities = numpy.interp(
                                        numpy.arange(n_frames)/float(n_frames), numpy.arange(pixel_velocities.shape[0])/float(pixel_velocities.shape[0]), pixel_velocities)
        #== Generate texture  
        texture = stimulus_profile
        texture=texture.swapaxes(0,1)
        if hasattr(flicker, 'has_key') and flicker.has_key('frequency') and flicker.has_key('modulation_size'):
            modulation_size_p=flicker['modulation_size']*self.config.SCREEN_UM_TO_PIXEL_SCALE
            modulation_pixels=int(display_area_adjusted[1]/modulation_size_p)
            texture=numpy.tile(texture,(modulation_pixels,1,1))
            flicker_state=False
            switch_count=int(self.config.SCREEN_EXPECTED_FRAME_RATE/float(flicker['frequency']))
        if len(phases)>0:
            phases_pixel=numpy.array(phases)*self.config.SCREEN_UM_TO_PIXEL_SCALE / float(stimulus_profile.shape[0])
            n_frames=phases_pixel.shape[0]
        else:
            phases_pixel=None
        texture_coordinates = numpy.array(
                             [
                             [cut_off_ratio, 1.0],
                             [0.0, 1.0],
                             [0.0, 0.0],
                             [cut_off_ratio, 0.0],
                             ])
        self._init_texture(utils.cr(display_area_adjusted),orientation,texture_coordinates)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[1], texture.shape[0], 0, GL_RGB, GL_FLOAT, texture)
        start_time = time.time()
        phase = 0
        self.t0=time.time()
        phases=pixel_velocities.cumsum()
        phases-=phases[0]
        for i in range(n_frames):
            if self.machine_config.ENABLE_TIME_INDEXING:
                index=self._get_frame_index()
            else:
                index=i
            if not hasattr(phases_pixel,'shape'):
                phase = phases[index]
            else:
                phase=phases_pixel[index]
            if hasattr(flicker, 'has_key') and flicker.has_key('frequency') and flicker.has_key('modulation_size'):
                if i%switch_count==0:
                    flicker_state=not flicker_state
                    texture1=numpy.copy(texture)
                    texture1[int(flicker_state)::2]=0.0
                    glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[1], texture.shape[0], 0, GL_RGB, GL_FLOAT, texture1)
            glTexCoordPointerf(texture_coordinates + numpy.array([phase,0.0]))
            if not part_of_drawing_sequence:
                glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv((1.0,1.0,1.0))
            glDrawArrays(GL_POLYGON,  0, 4)
            if not part_of_drawing_sequence:
                self._add_block_start(is_block, i, n_frames)
                self._flip(frame_timing_pulse = True)
                self._add_block_end(is_block, i, n_frames)
            if self.abort:
                break
        glDisable(GL_TEXTURE_2D)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
        
    def show_dots(self,  dot_diameters, dot_positions, ndots, duration = 0.0,  color = (1.0,  1.0,  1.0), 
            block_trigger = False):
        '''
        Maintains backward compatibility with old stimulations using show_dots. Use the show_shapes instead
        '''
        raise NotImplementedError('block handling and trigger generation is not implemented')
        self.show_shapes('o', dot_diameters, dot_positions, ndots, duration = duration,  color = color, block_trigger = block_trigger, colors_per_shape = False)
                    
    def show_shapes(self, shape, shape_size, shape_positions, nshapes, duration = 0.0,  
                            color = (1.0,  1.0,  1.0), background_color = None, block_trigger = False, 
                            are_same_shapes_over_frames = False, colors_per_shape = True, save_frame_info = True):
        '''
        Shows a huge number (up to several hunders) of shapes.
        Parameters:
            shape_size: one dimensional list of shape sizes in um or the size of rectangle, 
                in this case a two dimensional array is also supported
            shape_positions: one dimensional list of shape positions (row, col) in um.
            nshapes: number of shapes per frame
            color: can be a single tuple of the rgb values that apply to each shapes over the whole stimulation. 
                Both list and numpy formats are supported. Optionally a two dimensional list can be provided 
                where the dimensions are organized as above controlling the color of each shape individually
            duration: duration of each frame in s. When 0, frame is shown for one frame time.
            are_same_shapes_over_frames: if True, all frames show the same shapes with different colors

        The shape_sizes and shape_positions are expected to be in a linear list. Based on the nshapes, 
        these will be segmented to frames assuming that on each frame the number of shapes are equal.
        '''
        raise NotImplementedError('block handling and trigger generation is not implemented')
        self.log_on_flip_message_initial = 'show_shapes(' + str(duration)+ ', ' + str(shape_size) +', ' + str(shape_positions) +')'
        self.log_on_flip_message_continous = 'show_shapes'
        first_flip = False
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe())
        shape = self._get_shape_string(shape)
        if shape == 'circle':
            radius = 1.0
            vertices = geometry.circle_vertices([radius,  radius],  1.0/1.0)
        elif shape == 'rectangle':
            vertices = numpy.array([[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]])
        else:
            raise RuntimeError('Unknown shape: {0}'.format(shape))
        if are_same_shapes_over_frames:
            n_frames = color.shape[0]
        else:
            n_frames = len(shape_positions) / nshapes
        self.log_on_flip_message_initial += ' n_frames = ' + str(n_frames)
        n_vertices = len(vertices)        
        if are_same_shapes_over_frames:
            frames_vertices = numpy.zeros(( nshapes * n_vertices,  2))         
        else:
            frames_vertices = numpy.zeros((n_frames * nshapes * n_vertices,  2))         
        index = 0
        for frame_i in range(n_frames):
            for shape_i in range(nshapes):
                shape_index = frame_i * nshapes + shape_i
                shape_size_i = shape_size[shape_index]
                shape_position = numpy.array((shape_positions[shape_index]['col'], shape_positions[shape_index]['row']))
                shape_to_screen =  self.config.SCREEN_UM_TO_PIXEL_SCALE * (vertices * shape_size_i + shape_position)
                frames_vertices[index: index + n_vertices] = shape_to_screen
                index = index + n_vertices
            if are_same_shapes_over_frames:
                break
        if duration == 0:
            n_frames_per_pattern = 1
        else:
            n_frames_per_pattern = int(float(duration) * float(self.config.SCREEN_EXPECTED_FRAME_RATE))
#         if hasattr(color, 'dtype') and hasattr(self.config, 'GAMMA_CORRECTION'):
#             color_corrected = self.config.GAMMA_CORRECTION(color)
#         else:
        color_corrected = color
        if background_color != None:
            background_color_saved = glGetFloatv(GL_COLOR_CLEAR_VALUE)
            converted_background_color = colors.convert_color(background_color, self.config)
            glClearColor(converted_background_color[0], converted_background_color[1], converted_background_color[2], 0.0)
        else:
            converted_background_color = colors.convert_color(self.config.BACKGROUND_COLOR, self.config)        
        glEnableClientState(GL_VERTEX_ARRAY)
        shape_pointer = 0
        for frame_i in range(n_frames):
            start_i = shape_pointer * n_vertices
            end_i = (shape_pointer + nshapes) * n_vertices
            if not are_same_shapes_over_frames:
                shape_pointer = shape_pointer + nshapes
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)            
#            self._show_text()
            glVertexPointerf(frames_vertices[start_i:end_i])
            for i in range(n_frames_per_pattern):
                for shape_i in range(nshapes):
                    if colors_per_shape:
                        glColor3fv(colors.convert_color(color_corrected[shape_i], self.config))
                    elif isinstance(color_corrected[0], list):
                        glColor3fv(colors.convert_color(color_corrected[frame_i][shape_i], self.config))
                    elif isinstance(color_corrected[0], numpy.ndarray):
                        glColor3fv(colors.convert_color(color_corrected[frame_i][shape_i].tolist()))
                    else:
                        glColor3fv(colors.convert_color(color_corrected, self.config))
                    glDrawArrays(GL_POLYGON, shape_i * n_vertices, n_vertices)
                    if self.abort:
                        break
                #Make sure that at the first flip the parameters of the function call are logged
                if not first_flip:
                    self.log_on_flip_message = self.log_on_flip_message_initial
                    first_flip = True
                else:
                    self.log_on_flip_message = self.log_on_flip_message_continous
                self._flip_and_block_trigger(i, n_frames_per_pattern, True, block_trigger)
                if self.abort:
                    break
            if self.abort:
                break
                
        glDisableClientState(GL_VERTEX_ARRAY)
        #Restore original background color
        if background_color != None:            
            glClearColor(background_color_saved[0], background_color_saved[1], background_color_saved[2], background_color_saved[3])
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
            
    def show_natural_bars(self, speed = 300, repeats = 1, duration=20.0, minimal_spatial_period = None, 
                            spatial_resolution = None, intensity_levels = 255, direction = 0, background=None,
                            offset=0.0, scale=1.0, fly_in=False, fly_out=False, circular=False,
                            duration_calc_only=False,save_frame_info =True, is_block = False):
        '''
        Show vertical bars where the distribution of the color of the bar corresponds to the distribution of
        a natural scene which means that the spectra of the colors over spatial freuqency is 1/f
            speed: the speed of the movement of the vertical bars in um/s
            duration: duration of the stimulus. speed*duration detemines the lower end of 
                the spatial frequency range
            minimal_spatial_period: the higher end of the spatial frequency range in um
            spatial_resolution: The spatial frequency resolution. By default it corresponds to one pixel.
            direction: the stimulus and its movement is rotated to this angle
            fly_in: when stimulus starts, the barcode pattern flies on
            fly_out: at the end of the stimulus the barcode pattern flies out
            background: background color of the stimulus when fly_in and fly_out enabled
            circular: The stimulus flashes up with the initial pattern and at the 
                end the very same pattern is shown.
            offset, scale: with this the 1/f profile can be shifted and scaled
            duration_calc_only: the function returns with the precalculated duration of the stimulus. 
                No stimulus is presented.
        '''
        if spatial_resolution is None:
            spatial_resolution = self.machine_config.SCREEN_PIXEL_TO_UM_SCALE
        if minimal_spatial_period is None:
            minimal_spatial_period = 10 * spatial_resolution
        if not duration_calc_only:
            self.log.info('show_natural_bars(' + str(speed)+ ', ' + str(repeats) +', ' + str(duration) +', ' + str(minimal_spatial_period)+', ' + str(spatial_resolution)+ ', ' + str(intensity_levels) +', ' + str(direction)+ ')',source='stim')
        self.intensity_profile = offset+scale*signal.generate_natural_stimulus_intensity_profile(duration, speed, minimal_spatial_period, spatial_resolution, intensity_levels)
        if 0:#For testing only
            self.intensity_profile = numpy.linspace(0,1,self.intensity_profile.shape[0])
            self.intensity_profile[:0.1*self.intensity_profile.shape[0]]=0.0
            self.intensity_profile[-0.1*self.intensity_profile.shape[0]:]=1.0
        self.intensity_profile = numpy.tile(self.intensity_profile, repeats)
        if save_frame_info and not duration_calc_only:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = False)
            self.stimulus_frame_info[-1]['parameters']['intensity_profile']=self.intensity_profile
        if hasattr(self.machine_config, 'GAMMA_CORRECTION'):
            self.intensity_profile = self.machine_config.GAMMA_CORRECTION(self.intensity_profile)
        intensity_profile_length = self.intensity_profile.shape[0]
        if self.intensity_profile.shape[0] < self.config.SCREEN_RESOLUTION['col']:
            self.intensity_profile = numpy.tile(self.intensity_profile, int(numpy.ceil(float(self.config.SCREEN_RESOLUTION['col'])/self.intensity_profile.shape[0])))
        alltexture = numpy.repeat(self.intensity_profile,3).reshape(self.intensity_profile.shape[0],1,3)
        bg=colors.convert_color(self.config.BACKGROUND_COLOR[0] if background is None else background, self.config)
        fly_in_out = bg[0] * numpy.ones((self.config.SCREEN_RESOLUTION['col'],1,3))
        intensity_profile_length += (fly_in+fly_out)*fly_in_out.shape[0]
        if fly_in:
            alltexture=numpy.concatenate((fly_in_out,alltexture))
        if fly_out:
            alltexture=numpy.concatenate((alltexture,fly_in_out))
        ds = float(speed*self.config.SCREEN_UM_TO_PIXEL_SCALE)/self.machine_config.SCREEN_EXPECTED_FRAME_RATE
        if duration_calc_only:
            return (alltexture.shape[0]-(0 if circular else self.config.SCREEN_RESOLUTION['col']))/(ds*float(self.machine_config.SCREEN_EXPECTED_FRAME_RATE))
        texture = alltexture[:self.config.SCREEN_RESOLUTION['col']]
        diagonal = numpy.sqrt(2) * numpy.sqrt(self.config.SCREEN_RESOLUTION['row']**2 + self.config.SCREEN_RESOLUTION['col']**2)
        diagonal =  1*numpy.sqrt(2) * self.config.SCREEN_RESOLUTION['col']
        alpha =numpy.pi/4
        angles = numpy.array([alpha, numpy.pi - alpha, alpha + numpy.pi, -alpha])
        angles = angles + direction*numpy.pi/180.0
        vertices = 0.5 * diagonal * numpy.array([numpy.cos(angles), numpy.sin(angles)])
        vertices = vertices.transpose()
        if self.config.COORDINATE_SYSTEM == 'ulcorner':
            vertices += self.config.SCREEN_UM_TO_PIXEL_SCALE*numpy.array([self.machine_config.SCREEN_CENTER['col'], self.machine_config.SCREEN_CENTER['col']])
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(vertices)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[0], texture.shape[1], 0, GL_RGB, GL_FLOAT, texture)
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
        self.t0=time.time()
        texture_pointer = 0
        frame_counter = 0
        self._add_block_start(is_block, 0, 0)
        while True:
            start_index = int(texture_pointer)
            end_index = int(start_index + self.config.SCREEN_RESOLUTION['col'])
            if end_index > alltexture.shape[0]:
                end_index -= alltexture.shape[0]
            if start_index > alltexture.shape[0]:
                start_index -= alltexture.shape[0]
            if start_index < end_index:
                texture = alltexture[start_index:end_index]
            else:
                if circular:
                    texture = numpy.zeros_like(texture)
                    texture[:-end_index] = alltexture[start_index:]
                    texture[-end_index:] = alltexture[:end_index]
                    if start_index >= intensity_profile_length-1:
                        break
                else:
                    break
            if self.machine_config.ENABLE_TIME_INDEXING:
                frame_counter=self._get_frame_index()
            else:
                frame_counter += 1
            texture_pointer = ds*frame_counter
            glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[0], texture.shape[1], 0, GL_RGB, GL_FLOAT, texture)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv((1.0,1.0,1.0))
            glDrawArrays(GL_POLYGON,  0, 4)
            self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        self._add_block_end(is_block, 0, 1)
        dt=(time.time()-self.t0)
        #print 'frame rate', frame_counter/dt,'dt', dt,'frame counter', frame_counter,'text pointer', texture_pointer,'all texture size', alltexture.shape[0], 'self.intensity_profile', self.intensity_profile.shape, 'ds', ds
        glDisable(GL_TEXTURE_2D)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        if save_frame_info and not duration_calc_only:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
            self.stimulus_frame_info[-1]['parameters']['intensity_profile']=self.intensity_profile
            
    def show_white_noise(self, duration, square_size,save_frame_info=True):
        '''
        Generates white noise stimulus using numpy.random.random
        
        duration: duration of white noise stimulus in seconds
        square_size: size of squares. Number of squares is calculated from screen size but 
        fractional squares are not displayed. The array of squares is centered
        '''
        if save_frame_info:
            self.log.info('show_white_noise(' + str(duration)+ ', ' + str(square_size) + ')', source = 'stim')
            self._save_stimulus_frame_info(inspect.currentframe())
        square_size_pixel = square_size*self.machine_config.SCREEN_UM_TO_PIXEL_SCALE
        nframes = int(self.machine_config.SCREEN_EXPECTED_FRAME_RATE*duration)
        ncheckers = utils.rc_x_const(self.machine_config.SCREEN_SIZE_UM, 1.0/square_size)
        ncheckers = utils.rc((numpy.floor(ncheckers['row']), numpy.floor(ncheckers['col'])))
        numpy.random.seed(0)
        checker_colors = numpy.where(numpy.random.random((nframes,int(ncheckers['row']),int(ncheckers['col'])))<0.5, False,True)
        row_coords = numpy.arange(ncheckers['row'])-0.5*(ncheckers['row'] - 1)
        col_coords = numpy.arange(ncheckers['col'])-0.5*(ncheckers['col'] -1)
        rc, cc = numpy.meshgrid(row_coords, col_coords)
        positions=numpy.rollaxis(numpy.array([rc,cc]),0,3)*square_size
        params = {'colors': checker_colors, 'ncheckers':ncheckers, 'positions': positions}
        if save_frame_info:
            self._append_to_stimulus_frame_info(params)
        size = utils.rc_x_const(ncheckers, square_size_pixel)
        self._init_texture(size)
        self.t0=time.time()
        for frame_i in range(nframes):
            if self.machine_config.ENABLE_TIME_INDEXING:
                index=self._get_frame_index()
            else:
                index=frame_i
            texture = checker_colors[index]
            texture = numpy.rollaxis(numpy.array(3*[numpy.cast['float'](texture)]), 0,3)
            glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[1], texture.shape[0], 0, GL_RGB, GL_FLOAT, texture)
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv((1.0,1.0,1.0))
            glDrawArrays(GL_POLYGON,  0, 4)
            self.draw()
            self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        self._deinit_texture()
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
            self._append_to_stimulus_frame_info(params)
            
    def show_rolling_image(self, filename,pixel_size,speed,shift,yrange,axis='horizontal', save_frame_info=True):
        if save_frame_info:
            self.log.info('show_rolling_image(' + str(filename)+ ', ' + str(pixel_size) + ', ' + str(speed) + ', ' + str(shift)  + ', ' + str(yrange)  + ')', source = 'stim')
            self._save_stimulus_frame_info(inspect.currentframe())
        texture=numpy.flipud(numpy.asarray(Image.open(filename))/255.)
        if len(texture.shape)<3:
            texture=numpy.swapaxes(numpy.array(3*[texture]),0,2)
        if yrange!=None:
            if axis=='horizontal':
                texture=texture[yrange[0]:yrange[1],:,:]
            elif axis=='vertical':
                texture=texture[:, yrange[0]:yrange[1],:]
        shift_pixel=shift/self.config.SCREEN_UM_TO_PIXEL_SCALE
        dpixel=speed*self.config.SCREEN_UM_TO_PIXEL_SCALE/self.config.SCREEN_EXPECTED_FRAME_RATE
        #Image size: texture.shape*pixel_size*screen um2 pixel ratio
        size=utils.rc(numpy.array(texture.shape[:2])*pixel_size/self.config.SCREEN_UM_TO_PIXEL_SCALE)
        texture_coordinates = numpy.array(
                             [
                             [1, 1],
                             [0.0, 1],
                             [0.0, 0.0],
                             [1, 0.0],
                             ])
        self._init_texture(size,0)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[1], texture.shape[0], 0, GL_RGB, GL_FLOAT, texture)
        #Calculate trajectory of image motion
        ndsize=numpy.array([size['row'],size['col']])
        ndres=numpy.array([self.config.SCREEN_RESOLUTION['row'],self.config.SCREEN_RESOLUTION['col']])
        if axis=='horizontal':
            p0=(ndsize/2-ndres/2)[::-1]
            p1=p0*numpy.array([-1,1])
            p1=p1.flatten()
            p0=p0.flatten()
            nshifts=int(size['row']/shift_pixel)-1
            vertical_offsets=numpy.arange(nshifts)*shift_pixel
            vertical_offsets=numpy.repeat(vertical_offsets,3)
            points=numpy.array([p0,p1,p0])
            points=numpy.array(points.tolist()*nshifts)
            points[:,1]-=vertical_offsets
        elif axis=='vertical':
            p0=(ndsize/2-ndres/2)[::-1]
            p1=p0*numpy.array([1,-1])
            p1=p1.flatten()
            p0=p0.flatten()
            nshifts=int(size['col']/shift_pixel)-1
            horizontal_offsets=numpy.arange(nshifts)*shift_pixel
            horizontal_offsets=numpy.repeat(horizontal_offsets,3)
            points=numpy.array([p0,p1,p0])
            points=numpy.array(points.tolist()*nshifts)
            points[:,0]-=horizontal_offsets
        #Interpolate between points
        offset=numpy.empty((0,2))
        for i in range(points.shape[0]-1):
            start=points[i]
            end=points[i+1]
            nsteps=int(abs(((end-start)/dpixel)).max())
            increment_vector=(end-start)/abs((end-start)).max()*dpixel
            steps=numpy.repeat(numpy.arange(nsteps),2).reshape(nsteps,2)*increment_vector+start
            offset=numpy.concatenate((offset,steps))
        import time
        t0=time.time()
        for i in range(offset.shape[0]):
            now=time.time()
            index=int((now-t0)*self.config.SCREEN_EXPECTED_FRAME_RATE)
            if index>=offset.shape[0]:
                break
            vertices = geometry.rectangle_vertices(size, orientation = 0)-offset[index]
            glEnableClientState(GL_VERTEX_ARRAY)
            glVertexPointerf(vertices)
            glTexCoordPointerf(texture_coordinates)
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv((1.0,1.0,1.0))
            glDrawArrays(GL_POLYGON,  0, 4)
            self._flip(False)
            if self.abort:
                break
        print i/(time.time()-t0)
        self._deinit_texture()
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
            
    def show_approach_stimulus(self, motion, bar_width, speed, color=1.0, initial_wait=2.0, mask_size=400.,save_frame_info=True):
        if save_frame_info:
            self.log.info('show_approach_stimulus(' + str(motion)+ ', ' + str(bar_width) + ', ' + str(speed) + ', ' + str(color)  + ', ' + str(initial_wait)  + ', ' + str(mask_size)  + ')', source = 'stim')
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = False)
        bar_width_pixel=bar_width*self.config.SCREEN_UM_TO_PIXEL_SCALE
        mask_size_pixel=mask_size*self.config.SCREEN_UM_TO_PIXEL_SCALE
        dpix=speed*self.config.SCREEN_UM_TO_PIXEL_SCALE/self.config.SCREEN_EXPECTED_FRAME_RATE
        circle_vertices=geometry.circle_vertices([mask_size_pixel]*2,  resolution = 0.5)+0*numpy.array([[100,0]])
        #Convert circle to its complementer shape being composed of rectangles
        rect_width=self.config.SCREEN_RESOLUTION['col']/2+circle_vertices[:,0].min()
        x1=-self.config.SCREEN_RESOLUTION['col']/2+rect_width/2
        x2=self.config.SCREEN_RESOLUTION['col']/2-rect_width/2
        mask_vertices1=geometry.rectangle_vertices(utils.cr((rect_width,self.config.SCREEN_RESOLUTION['row'])))+numpy.array([[x1,0]])
        mask_vertices2=geometry.rectangle_vertices(utils.cr((rect_width,self.config.SCREEN_RESOLUTION['row'])))+numpy.array([[x2,0]])
        mask_vertices=mask_vertices1
        mask_vertices=numpy.append(mask_vertices,mask_vertices2,axis=0)
        for pi in range(circle_vertices.shape[0]):
            p1=circle_vertices[pi]
            if pi==circle_vertices.shape[0]-1:
                p2=circle_vertices[0]
            else:
                p2=circle_vertices[pi+1]
            if numpy.sign(p1[1])!=numpy.sign(p2[1]):
                continue
            if p1[1]>0:
                coo=self.config.SCREEN_RESOLUTION['row']/2
            else:
                coo=-self.config.SCREEN_RESOLUTION['row']/2
            rect=numpy.array([p1,p2, [p2[0],coo], [p1[0],coo]])
            mask_vertices=numpy.append(mask_vertices,rect,axis=0)
        intial_wait_frames=int(self.config.SCREEN_EXPECTED_FRAME_RATE*initial_wait)
        #precalculate edge positions
        start_pos=utils.rc((0,0))
        start_size=bar_width_pixel
        if motion=='expand':
            end_pos=utils.rc((0,0))
            end_size=mask_size_pixel
        elif motion=='shrink':
            end_pos=utils.rc((0,0))
            end_size=0
        elif motion=='left':
            end_pos=utils.rc((0,-mask_size_pixel/2))
            end_size=bar_width_pixel
        elif motion=='right':
            end_pos=utils.rc((0,mask_size_pixel/2))
            end_size=bar_width_pixel
        #interpolate positions and sizes
        if start_pos['col']==end_pos['col']:
            if start_size<end_size:
                d=dpix
            else:
                d=-dpix
            size=numpy.arange(start_size,end_size+dpix, d)
            x=numpy.ones(size.shape[0])*start_pos['col']
        elif start_size==end_size:
            if start_pos['col']<end_pos['col']:
                d=dpix
            else:
                d=-dpix
            x=numpy.arange(start_pos['col'],end_pos['col']+dpix, d)
            size=numpy.ones_like(x)*start_size
        else:
            raise NotImplementedError('')
        #x=numpy.concatenate((numpy.ones(intial_wait_frames)*x[0],x))
        #size=numpy.concatenate((numpy.ones(intial_wait_frames)*size[0],size))
        y=numpy.ones_like(x)*start_pos['row']
        #generate rectangle vertices
        vertices=mask_vertices
        for i in range(size.shape[0]):
            vertices=numpy.append(vertices,geometry.rectangle_vertices(utils.rc((mask_size_pixel,size[i])))+numpy.array([[x[i],y[i]]]),axis=0)
        #texture_coordinates=self._init_texture(self.config.SCREEN_RESOLUTION)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(vertices)
        n_vertices=4
        for i in range(size.shape[0]):
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT)
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE)
            glEnable( GL_STENCIL_TEST )
            glStencilFunc( GL_ALWAYS, 1, 1 )
            glStencilOp( GL_REPLACE, GL_REPLACE, GL_REPLACE )
            for shi in range(mask_vertices.shape[0]/4):
                glDrawArrays(GL_POLYGON, shi*4, 4)
            
            glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE )
            glStencilFunc( GL_EQUAL, 1, 1 )
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP)
            glColor3fv(colors.convert_color(color, self.config))
            glDrawArrays(GL_POLYGON,  i * n_vertices+mask_vertices.shape[0], n_vertices)
            glDisable( GL_STENCIL_TEST )
            if i==1:
                for w in range(intial_wait_frames-1):
                    self._flip(frame_timing_pulse = True)
            else:
                self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        glDisableClientState(GL_VERTEX_ARRAY)
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
            
    def show_moving_plaid(self,duration, direction, relative_angle, velocity,line_width, duty_cycle, mask_size=None, contrast=1.0, background_color=0.0,  sinusoid=False, save_frame_info=True):
        if save_frame_info:
            params=map(str, [duration, direction, relative_angle, velocity,line_width, duty_cycle, mask_size, contrast, background_color,  sinusoid]            )
            self.log.info('show_moving_plaid({0})'.format(', '.join(params)), source = 'stim')
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = False)
        #Generate texture:
        line_width_p=int(line_width*self.config.SCREEN_UM_TO_PIXEL_SCALE)
        line_spacing_p=int(line_width_p*duty_cycle)
        #Generate single tile:
        tile_height=abs(int(line_spacing_p/numpy.tan(0.5*numpy.radians(relative_angle))))
        tile=Image.new('L', (line_spacing_p, tile_height))
        from PIL import ImageDraw
        draw = ImageDraw.Draw(tile)
        draw.line((0,0, line_spacing_p, tile_height), fill=255, width =line_width_p)
        draw.line((line_spacing_p, 0,0, tile_height), fill=255, width =line_width_p)
        tile=numpy.cast['float'](numpy.asarray(tile))/255*contrast
        tilea=numpy.where(tile==0, background_color, tile)
        if mask_size ==None:
            texture_size=numpy.sqrt(self.config.SCREEN_RESOLUTION['col'] **2+self.config.SCREEN_RESOLUTION['row'] **2)
        else:
            texture_size=mask_size*self.config.SCREEN_UM_TO_PIXEL_SCALE
        texture_size=int(line_spacing_p*numpy.ceil(texture_size/line_spacing_p))
        #Repeat tile and make a texture of it
        nrepeats=numpy.cast['int'](numpy.ceil(numpy.array(2*[texture_size], dtype=numpy.float)/numpy.array(tilea.shape)))
        if sinusoid:#Extend texture
            nrepeats+=2
        texture=numpy.zeros(nrepeats*numpy.array(tilea.shape))
        for row in range(nrepeats[0]):
            for col in range(nrepeats[1]):
                texture[row*tilea.shape[0]:(row+1)*tilea.shape[0],col*tilea.shape[1]:(col+1)*tilea.shape[1]]=tilea
        if sinusoid:
            #transform texture into distance from edges
            texture=numpy.where(texture==background_color,0,1)
            texture=signal.shape2distance(texture, line_width_p/2)
            #Apply sinus on distances
            contrast_step=contrast-background_color
            texture=numpy.sin(texture/float(texture.max())*numpy.pi/2)*contrast_step+background_color
            #Cut off extensions
            texture=texture[tilea.shape[0]:-tilea.shape[0], tilea.shape[1]:-tilea.shape[1]]
            
        texture=numpy.rot90(texture)
        texture_coordinates=self._init_texture(utils.rc((texture.shape[0], texture.shape[1])),direction,set_vertices=(mask_size == None))
        texture=numpy.rollaxis(numpy.array(3*[texture]),0,3)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, texture.shape[1], texture.shape[0], 0, GL_RGB, GL_FLOAT, texture)
        dpixel=-velocity*self.config.SCREEN_UM_TO_PIXEL_SCALE/self.config.SCREEN_EXPECTED_FRAME_RATE/texture.shape[1]
        if mask_size != None:
            mask_resolution=0.1
            mask=self._generate_mask_vertices(mask_size*self.config.SCREEN_UM_TO_PIXEL_SCALE, resolution=mask_resolution)
            vertices = geometry.rectangle_vertices(utils.rc((texture.shape[0], texture.shape[1])),direction)
            vertices=numpy.append(vertices, mask,axis=0)
            glEnableClientState(GL_VERTEX_ARRAY)
            glVertexPointerf(vertices)
        phase=0
        t0=time.time()
        for i in range(int(self.config.SCREEN_EXPECTED_FRAME_RATE*duration)):
            phase+=dpixel
            if mask_size == None:
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT)
                glTexCoordPointerf(texture_coordinates+numpy.array([phase,0.0]))
                glColor3fv((1.0,1.0,1.0))
                glDrawArrays(GL_POLYGON, 0, 4)
            else:
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT)
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE)
                glEnable(GL_STENCIL_TEST)
                glStencilFunc(GL_ALWAYS, 1, 1)
                glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE)
                for shi in range(mask.shape[0]/4):
                    glDrawArrays(GL_POLYGON, (shi+1)*4, 4)
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)
                glStencilFunc(GL_EQUAL, 1, 1)
                glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP)
                glTexCoordPointerf(texture_coordinates+numpy.array([phase,0.0]))
                glColor3fv((1.0,1.0,1.0))
                glDrawArrays(GL_POLYGON, 0, 4)
                glDisable(GL_STENCIL_TEST)
            self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        print time.time()-t0
        self._deinit_texture()
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
            
class StimulationHelpers(Stimulations):
    def _init_texture(self,size,orientation=0,texture_coordinates=None, set_vertices=True):
        from visexpman.engine.generic import geometry
        vertices = geometry.rectangle_vertices(size, orientation = orientation)
        if set_vertices:
            glEnableClientState(GL_VERTEX_ARRAY)
            glVertexPointerf(vertices)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
        glEnable(GL_TEXTURE_2D)
        glEnableClientState(GL_TEXTURE_COORD_ARRAY)
        if texture_coordinates is None:
            texture_coordinates = numpy.array(
                             [
                             [1.0, 1.0],
                             [0.0, 1.0],
                             [0.0, 0.0],
                             [1.0, 0.0],
                             ])
        glTexCoordPointerf(texture_coordinates)
        return texture_coordinates
        
    def _deinit_texture(self):
        glDisable(GL_TEXTURE_2D)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        
    def _generate_mask_vertices(self, mask_size_pixel, resolution=0.5):
        circle_vertices=geometry.circle_vertices([mask_size_pixel]*2,  resolution = resolution)+0*numpy.array([[100,0]])
        #Convert circle to its complementer shape being composed of rectangles
        rect_width=self.config.SCREEN_RESOLUTION['col']/2+circle_vertices[:,0].min()
        x1=-self.config.SCREEN_RESOLUTION['col']/2+rect_width/2
        x2=self.config.SCREEN_RESOLUTION['col']/2-rect_width/2
        mask_vertices1=geometry.rectangle_vertices(utils.cr((rect_width,self.config.SCREEN_RESOLUTION['row'])))+numpy.array([[x1,0]])
        mask_vertices2=geometry.rectangle_vertices(utils.cr((rect_width,self.config.SCREEN_RESOLUTION['row'])))+numpy.array([[x2,0]])
        mask_vertices=mask_vertices1
        mask_vertices=numpy.append(mask_vertices,mask_vertices2,axis=0)
        for pi in range(circle_vertices.shape[0]-1):
            p1=circle_vertices[pi]
            if pi==circle_vertices.shape[0]-1:
                p2=circle_vertices[0]
            else:
                p2=circle_vertices[pi+1]
            if numpy.sign(numpy.round(p1[1],6))!=numpy.sign(numpy.round(p2[1])) and numpy.round(p2[1])!=0 and numpy.round(p1[1])!=0:
                continue
            if numpy.round(p1[1],6)==0:
                if p2[1]>0:
                    coo=self.config.SCREEN_RESOLUTION['row']/2
                else:
                    coo=-self.config.SCREEN_RESOLUTION['row']/2
            elif p1[1]>0:
                coo=self.config.SCREEN_RESOLUTION['row']/2
            else:
                coo=-self.config.SCREEN_RESOLUTION['row']/2
            rect=numpy.array([p1,p2, [p2[0],coo], [p1[0],coo]])
            mask_vertices=numpy.append(mask_vertices,rect,axis=0)
        return mask_vertices
    
    def _merge_identical_frames(self):
        self.merged_bitmaps = [[self.screen.stimulus_bitmaps[0], 1]]
        for frame_i in range(1, len(self.screen.stimulus_bitmaps)):
            if abs(self.merged_bitmaps[-1][0] - self.screen.stimulus_bitmaps[frame_i]).sum()==0:
                self.merged_bitmaps[-1][1] += 1
            else:
                self.merged_bitmaps.append([self.screen.stimulus_bitmaps[frame_i], 1])
        
    def stimulusbitmap2uled(self):
        expected_configs = ['ULED_SERIAL_PORT', 'STIMULUS2MEMORY']
        if all([hasattr(self.machine_config, expected_config) for expected_config in expected_configs]) and self.machine_config.STIMULUS2MEMORY:
            self.config.STIMULUS2MEMORY = False
            from visexpman.engine.hardware_interface import microled
            self.microledarray = microled.MicroLEDArray(self.machine_config)
            self._merge_identical_frames()
            for frame_i in range(len(self.merged_bitmaps)):
                t0=time.time()
                self.microledarray.reset()
                self._frame_timing_pulse()
                if utils.is_abort_experiment_in_queue(self.queues['gui']['in'], False) or self.check_abort_pressed():
                    self.abort=True
                    break
                pixels = numpy.where(self.merged_bitmaps[frame_i][0].mean(axis=2) == 0, False, True)
                self.microledarray.display_pixels(pixels, self.merged_bitmaps[frame_i][1]/self.machine_config.SCREEN_EXPECTED_FRAME_RATE-(time.time()-t0), clear=False)
            self.microledarray.release_instrument()
        else:
            raise RuntimeError('Micro LED array stimulation is not configured properly, make sure that {0} parameters have correct values'.format(expected_configs))
        
    def export2video(self, filename, img_format='png'):
        if self.machine_config.ENABLE_FRAME_CAPTURE:
            videofile.images2mpeg4(os.path.join(self.machine_config.CAPTURE_PATH,  'captured_%5d.{0}'.format(img_format)), filename, int(self.machine_config.SCREEN_EXPECTED_FRAME_RATE))

    def projector_calibration(self, intensity_range = [0.0, 1.0], npoints = 128, time_per_point = 1.0, repeats = 3, sync_flash = False):
        self._save_stimulus_frame_info(inspect.currentframe())
        step = (intensity_range[1]-intensity_range[0])/(npoints)
        intensities = numpy.concatenate((numpy.arange(intensity_range[0], intensity_range[1]+step, step), numpy.arange(intensity_range[1], intensity_range[0]-step, -step)))
        if sync_flash:
            self.show_fullscreen(duration = 1.0, color = intensity_range[1])
            self.show_fullscreen(duration = 3.0, color = intensity_range[0])
        else:
            self.show_fullscreen(duration = 4.0, color = intensity_range[0])
        for r in range(repeats):
            for c in intensities:
                self.show_fullscreen(duration = time_per_point, color = c)
                self.measure_light_power(c)
                if self.check_abort_pressed():
                    break

    def measure_light_power(self, reference_intensity):
        '''
        Placeholder for light power measurement. This shall be implemented in the experiment class
        '''
        pass

class AdvancedStimulation(StimulationHelpers):
    '''
    Stimulation sequences, helpers
    ''' 
    
    def moving_comb(self, speed, orientation, bar_width, tooth_size, tooth_type, contrast, background,pos = utils.rc((0,0))):
        #tooth_type: square, sawtooth
        self._save_stimulus_frame_info(inspect.currentframe(), is_last = False)
        bar_width_pix = self.config.SCREEN_UM_TO_PIXEL_SCALE*bar_width
        tooth_size_pix = self.config.SCREEN_UM_TO_PIXEL_SCALE*tooth_size
        pos_pix = utils.rc_multiply_with_constant(pos, self.config.SCREEN_UM_TO_PIXEL_SCALE)
        converted_background_color = colors.convert_color(self.config.BACKGROUND_COLOR, self.config)
        combv,nshapes=self._draw_comb(orientation, bar_width_pix, tooth_size_pix, tooth_type)
        combv = geometry.rotate_point(utils.cr((combv[:,0],combv[:,1])),orientation,utils.cr((0,0)))
        combv = numpy.array([combv['col'], combv['row']]).T
        trajectories, trajectory_directions, duration = self.moving_shape_trajectory(bar_width, [speed], [orientation],1,0,shape_starts_from_edge=True)
        glEnableClientState(GL_VERTEX_ARRAY)
        nvertice = (combv.shape[0]-4)/(nshapes-1)
        for frame_i in range(trajectories[0].shape[0]):
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv(colors.convert_color(contrast, self.config))
            v=combv + numpy.array([trajectories[0][frame_i]['col']*self.config.SCREEN_UM_TO_PIXEL_SCALE+pos_pix['col'],trajectories[0][frame_i]['row']*self.config.SCREEN_UM_TO_PIXEL_SCALE+pos_pix['row']])
            glVertexPointerf(v)
            for shi in range(nshapes):
                if shi == 0:
                    glDrawArrays(GL_POLYGON,  0, 4)
                else:
                    glDrawArrays(GL_POLYGON,  4+(shi-1)*nvertice, nvertice)
            self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        
        glDisableClientState(GL_VERTEX_ARRAY)
        self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
    
    def _draw_comb(self, orientation, bar_width, tooth_size, tooth_type):
        bar_height = numpy.sqrt(self.machine_config.SCREEN_RESOLUTION['row']**2+self.machine_config.SCREEN_RESOLUTION['col']**2)
        vertices = geometry.rectangle_vertices(utils.cr((bar_width,bar_height)))
        if tooth_type == 'sawtooth':
            tooth_v = geometry.triangle_vertices(tooth_size)
            offset = numpy.array([0,0])
            tooth_spacing = tooth_size
            ntooth = int(bar_height/tooth_size)
        elif tooth_type == 'square':
            tooth_v = geometry.rectangle_vertices(utils.rc((tooth_size,tooth_size)))
            offset = numpy.array([0.5*tooth_size,0])
            tooth_spacing = 2*tooth_size
            ntooth = int(bar_height/tooth_size/2)
        for toothi in range(ntooth):
            rel_position = numpy.array([0.5*(bar_width-0*tooth_size), (toothi+0.5)*tooth_spacing-0.5*bar_height])
            vertices = numpy.concatenate((vertices, tooth_v+offset+rel_position))
        nshapes = ntooth + 1
        return vertices,nshapes
        
    def moving_cross(self, speeds, sizes, position, movement_directions):
        self._save_stimulus_frame_info(inspect.currentframe())
        bar_height = 3*numpy.sqrt(self.machine_config.SCREEN_SIZE_UM['row']**2+self.machine_config.SCREEN_SIZE_UM['col']**2)
        ds = numpy.array(speeds)/self.machine_config.SCREEN_EXPECTED_FRAME_RATE
        movement = float(max(self.machine_config.SCREEN_SIZE_UM['row'], self.machine_config.SCREEN_SIZE_UM['col']))+max(sizes)
        trajectories = []
        for i in range(len(movement_directions)):
            startp = geometry.point_coordinates(0.5*movement, numpy.radians(movement_directions[i]-180), self.config.SCREEN_CENTER)
            if i>0:
                startp = utils.rc_add(startp, position)
            endp = geometry.point_coordinates(0.5*movement, numpy.radians(movement_directions[i]), self.config.SCREEN_CENTER)
            if ds[i] == 0:
                startp = position
                trajectories.append(numpy.repeat(startp,trajectories[0].shape[0]))
            else:
                trajectories.append(numpy.tile(utils.calculate_trajectory(startp, endp, ds[i]),int(numpy.ceil(ds[i]/ds[0]))))
            if i>0:
                if trajectories[0].shape[0]<trajectories[i].shape[0]:
                    trajectories[i] = trajectories[i][:trajectories[0].shape[0]]
        nframes = trajectories[0].shape[0]
        trajectories = numpy.array([numpy.array([t['col'], t['row']]) for t in trajectories])
        trajectories = numpy.concatenate([trajectories[:,:,i] for i in range(trajectories.shape[2])])
        base_vertices = numpy.concatenate([geometry.rectangle_vertices(utils.rc((bar_height, sizes[i])), movement_directions[i]) for i in range(len(sizes))]).T
        vertices = numpy.tile(base_vertices,trajectories.shape[0]/2).T
        vertices += numpy.repeat(trajectories,4,axis=0)
        vertices *= self.config.SCREEN_UM_TO_PIXEL_SCALE
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(vertices)
        for frame_i in range(nframes):
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glColor3fv(colors.convert_color(1.0, self.config))
            glDrawArrays(GL_POLYGON, frame_i*8, 4)
            glDrawArrays(GL_POLYGON, frame_i*8+4, 4)
            self._flip(frame_timing_pulse = True)
            if self.abort:
                break
        glDisableClientState(GL_VERTEX_ARRAY)
        self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
    
    def flash_stimulus(self, shape, timing, colors, sizes = utils.rc((0, 0)), position = utils.rc((0, 0)), background_color = 0.0, repeats = 1, block_trigger = True, save_frame_info = True,  ring_sizes = None):
        '''
        Use cases:
        shape: like show_shape, ff = fullfield
        timing: 1. [ON TIME, OFF_TIME] for each size and/or each color
                    2. OFF TIME 1, ON_TIME flash1, OFF_TIME 2, ON_TIME flash2
        colors: formats:
                1. single color in intensity or rgb format
                2. 2d numpy array: duration is ignored and the first dimension will be the number of intensities displayed
        sizes: 1. single size
                  2. 2d numpy array: series of different sizes
        '''
        raise NotImplementedError('block handling and trigger generation is not implemented')
        if save_frame_info:
            self.log.info('flash_stimulus(' + str(shape)+ ', ' + str(timing) +', ' + str(colors) +', ' + str(sizes)  +', ' + str(position)  + ', ' + str(background_color) + ', ' + str(repeats) + ', ' + str(block_trigger) + ')', source='stim')
            self._save_stimulus_frame_info(inspect.currentframe())
        if isinstance(timing, list) and len(timing) == 2 or hasattr(timing, 'dtype') and timing.shape[0] == 2:
            #find out number of flashes
            if isinstance(sizes, list):
                n_flashes = len(sizes)
            elif hasattr(sizes, 'dtype'):
                if len(sizes.shape) == 0:
                    n_flashes = 0
                else:
                    n_flashes = sizes.shape[0]
            elif isinstance(ring_sizes, list):
                n_flashes = len(ring_sizes)
            elif hasattr(ring_sizes, 'dtype'):
                if len(ring_sizes.shape) == 0:
                    n_flashes = 0
                else:
                    n_flashes = ring_sizes.shape[0]
            else:
                raise RuntimeError('sizes or ring_sizes parameter shall be list or numpy.array.')
            if n_flashes == 0:
                if isinstance(colors, list):
                    n_flashes = len(colors)
                elif hasattr(colors, 'dtype'):
                    n_flashes = colors.shape[0]
            timing = [timing[0], timing[1]] * n_flashes
            timing.insert(0, timing[1])
        for r in range(repeats):
            state = False
            for i in range(len(timing)):
                if state:
                    if hasattr(colors, '__iter__'):
                        color = colors[(i-1)/2]
                    else:
                        color = colors
                    if shape == 'ff':
                        self.show_fullscreen(color = color, duration = timing[i], save_frame_info = False, block_trigger = block_trigger)
                    else:
                        if hasattr(sizes, '__iter__') and len(sizes.shape) > 0:
                            if len(sizes.dtype) == 2 : #row, col format
                                size = utils.rc((sizes[(i-1)/2]['row'], sizes[(i-1)/2]['col']))
                            else:
                                size = utils.rc((sizes[(i-1)/2],sizes[(i-1)/2]))
                        else:
                            size = sizes
                        if hasattr(ring_sizes, '__iter__') and len(ring_sizes.shape) > 0:
                            ring_size = ring_sizes[(i-1)/2]
                        else:
                            ring_size = ring_sizes
                        self.show_shape(shape = shape,  duration = timing[i],  pos = position,  color = color,  background_color = background_color,  size = size,  block_trigger = block_trigger, save_frame_info = False, ring_size = ring_size)
                else:
                    self.show_fullscreen(color = background_color, duration = timing[i], save_frame_info = False, block_trigger = False)
                state = not state
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)

    def increasing_spot(self, spot_sizes, on_time, off_time, color = 1.0, 
                        background_color = 0.0, pos = utils.rc((0,  0)), block_trigger = True):
        '''
        Presents increasing spot stimulus.
            spot_sizes: list of spot sizes in um
            on_time: duration of flashing the spot
            off_time: duration of pause between spots
            color: color of spot
            background_color: background color of screen
            pos: position of spots in um,
        '''
        raise NotImplementedError('block handling and trigger generation is not implemented')
        self.log.info('increasing_spot(' + str(spot_sizes)+ ', ' + str(on_time) +', ' + str(off_time) +', ' + str(color) +', ' + str(background_color) +', ' + str(pos) + ', ' + str(block_trigger) + ')', source='stim')
        self._save_stimulus_frame_info(inspect.currentframe())
        self.flash_stimulus('o', [on_time, off_time], color, sizes = numpy.array(spot_sizes), position = pos, background_color = background_color, repeats = 1, block_trigger = block_trigger, save_frame_info = False)
        self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
        
    def angle2screen_pos(self,angle,axis=None):
        if self.machine_config.SCREEN_DISTANCE_FROM_MOUSE_EYE==0:
            raise RuntimeError('SCREEN_DISTANCE_FROM_MOUSE_EYE parameter has invalid (0) value')
        distance_from_center = self.machine_config.SCREEN_DISTANCE_FROM_MOUSE_EYE*numpy.tan(numpy.radians(angle))
        if axis!=None:
            distance_from_center+=self.machine_config.SCREEN_CENTER[axis]
        self.machine_config.SCREEN_PIXEL_WIDTH#mm
        return (distance_from_center/self.machine_config.SCREEN_PIXEL_WIDTH)/self.machine_config.SCREEN_UM_TO_PIXEL_SCALE#um coordinate
        
    def angle2size(self,size_deg, pos_deg):
        '''
        based on angular position and size the um based position and size is calculated
        '''
        minangle=utils.rc_add(pos_deg,utils.rc_multiply_with_constant(size_deg,0.5),'-')
        maxangle=utils.rc_add(pos_deg,utils.rc_multiply_with_constant(size_deg,0.5),'+')
        size_row=self.angle2screen_pos(maxangle['row'],'row')-self.angle2screen_pos(minangle['row'],'row')
        size_col=self.angle2screen_pos(maxangle['col'],'col')-self.angle2screen_pos(minangle['col'],'col')
        return utils.rc((size_row,size_col))

    def receptive_field_explore(self,shape_size, on_time, off_time, nrows = None, ncolumns=None, 
                                display_size = None, flash_repeat = 1, sequence_repeat = 1, 
                                background_color = None,shape_colors = [1.0], random_order = False):
        '''
        The screen is divided into a meshgrid and rectangles are presented in each position 
                to map the recpeive field.
        display_size: row,col format in um, by default the whole screen
        shape_size: size of the rectangle in row/col format in um. If None, the shape size is calculated from the
                display size and the nrows and ncolumns
        nrows,ncolumns: number of rows and columns. If not provided calculated from display_size and shape_size
        flash_repeat: number of flashes for each position and color
        sequence_repeat: number of repeat for the whole sequence of rectangles flashed at 
                different positions and colors
        shape_colors: rectangles are shown at each position in these colors.
        background_color: background color of screen
        random_order: order of positions are shuffled. It tries to avoid adjacent positions shown subsequently
        
        if self.experiment_config.SIZE_DIMENSION=='angle':
            DISPLAY_CENTER shows the offset from the screen center in angles
        '''
        shape_size, nrows, ncolumns, display_size, shape_colors, background_color = \
                self._parse_receptive_field_parameters(shape_size, nrows, ncolumns, display_size, shape_colors, background_color)
        duration, positions = self.receptive_field_explore_durations_and_positions(shape_size=shape_size, 
                                                                            nrows = nrows,
                                                                            ncolumns = ncolumns,
                                                                            shape_colors = shape_colors,
                                                                            flash_repeat = flash_repeat,
                                                                            sequence_repeat = sequence_repeat,
                                                                            on_time = on_time,
                                                                            off_time = off_time)
        self.display_size=display_size
        import random,itertools
        positions_and_colors=[[c,p] for c,p in itertools.product(shape_colors,positions)]
        if random_order:
            ct=0
            while True:
                positions_and_colors,success = utils.shuffle_positions_avoid_adjacent(positions_and_colors,shape_size)
                if success:
                    break
                else:
                    ct+=1
                if ct>=10:
                    raise RuntimeError('Could not generate non adjacent random order of squares')
                    
            #random.shuffle(positions_and_colors)
        if hasattr(self.experiment_config, 'SIZE_DIMENSION') and self.experiment_config.SIZE_DIMENSION=='angle':
            positions_and_colors_angle=positions_and_colors
            positions=numpy.array([[position['row'], position['col']] for position in positions])
            corners=numpy.repeat(positions,2,axis=0)
            size=numpy.array([shape_size['row'], shape_size['col']])
            corners[0::2]+=size*0.5
            corners[1::2]-=size*0.5
            offset_angle_v=-self.experiment_config.DISPLAY_CENTER['row']-corners[:,0].min()
            offset_angle_h=-self.experiment_config.DISPLAY_CENTER['col']-corners[:,1].min()
            corners[:,0]+=offset_angle_v
            corners[:,1]+=offset_angle_h
            #Convert to distance from closest point
            d=numpy.tan(numpy.radians(corners))*self.machine_config.SCREEN_DISTANCE_FROM_MOUSE_EYE
            #Calculate closest point's coordinates
            posv=numpy.tan(numpy.radians(self.experiment_config.DISPLAY_CENTER['row']))*self.machine_config.SCREEN_DISTANCE_FROM_MOUSE_EYE
            posh=numpy.tan(numpy.radians(self.experiment_config.DISPLAY_CENTER['col']))*self.machine_config.SCREEN_DISTANCE_FROM_MOUSE_EYE
            screen_v=self.machine_config.SCREEN_PIXEL_WIDTH*self.machine_config.SCREEN_RESOLUTION['row']
            screen_h=self.machine_config.SCREEN_PIXEL_WIDTH*self.machine_config.SCREEN_RESOLUTION['col']
            #Check if the range covered by corners does not exceed the screen's physical size
            if any(signal.coo_range(d)>numpy.array([screen_v, screen_h])):
                raise RuntimeError('angle coordinates exceed screen size')
            posvcoo=-(screen_v/2-posv)
            poshcoo=-(screen_h/2-posh)
            offset=numpy.array([posvcoo,poshcoo])#Offset (closest point's coordinates) in mm space / on screen
            d+=offset
            #distance from screen center needs to be transformed to um space
            scale=numpy.array([self.machine_config.SCREEN_SIZE_UM['row'],self.machine_config.SCREEN_SIZE_UM['col']])/numpy.array([screen_v,screen_h])
            corners_um=d*scale
            #Calculate square centers and sizes from corners
            for pi in range(corners_um.shape[0]/2):
                c1=corners_um[2*pi]
                c2=corners_um[2*pi+1]
                size=c1-c2
                position=c1-0.5*size
                color, angle=positions_and_colors[pi]
                positions_and_colors[pi]=[angle, utils.rc(size), color, utils.rc(position)]
            pass
            #Consider positions in degree units and convert them to real screen positions
            if 0:
                #correct for screen center
                screen_center_um=self.machine_config.SCREEN_CENTER
                positions_and_colors = [[c,utils.rc((p['row']-screen_center_um['row'], p['col']-screen_center_um['col']))] for c,p in positions_and_colors]
                #Correct for display center
                center_angle_correction=utils.rc_add(utils.rc_multiply_with_constant(display_size,0.5),self.experiment_config.DISPLAY_CENTER,'-')
                positions_and_colors = [[c,utils.rc((p['row']-center_angle_correction['row'], p['col']-center_angle_correction['col']))] for c,p in positions_and_colors]
                #Convert angles to positions
                positions_and_colors = [[p,self.angle2size(shape_size, p),c,utils.rc((self.angle2screen_pos(p['row'],'row'),self.angle2screen_pos(p['col'],'col')))] for c, p in positions_and_colors]
                pos=numpy.array([p for a,d,c,p in positions_and_colors])
                offset=utils.cr(((pos['col'].max()+pos['col'].min())/2,(pos['row'].max()+pos['row'].min())/2))
                #offset=utils.rc_add(offset,screen_center_um,'+')
                positions_and_colors = [[a,d,c,utils.rc_add(p,offset,'-')] for a,d,c, p in positions_and_colors]
                #Convert to ulcorner
            if self.machine_config.COORDINATE_SYSTEM=='ulcorner':
                positions_and_colors = [[a,d,c,utils.rc((-p['row']+0.5*self.machine_config.SCREEN_SIZE_UM['row'],p['col']+0.5*self.machine_config.SCREEN_SIZE_UM['col']))] for a,d,c, p in positions_and_colors]
        else:
            positions_and_colors= [[0,shape_size,c,p] for c,p in positions_and_colors]
        self.nrows=nrows
        self.ncolumns=ncolumns
        self.shape_size=shape_size
        if 1:
            print corners_um[:,0].min(), corners_um[:,0].max(), self.machine_config.SCREEN_SIZE_UM['row']
            print corners_um[:,1].min(), corners_um[:,1].max(), self.machine_config.SCREEN_SIZE_UM['col']
        self.show_fullscreen(color = background_color, duration = off_time)
        for r1 in range(sequence_repeat):
            for angle,shape_size_i, color,p in positions_and_colors:
                    print angle
                    #print shape_size_i['row']
            #for p in positions:
             #   for color in shape_colors:
                    for r2 in range(flash_repeat):
                        if self.abort:
                            break
                        if hasattr(self, 'block_start'):
                            self.block_start(block_name = 'position')
                        self.show_fullscreen(color = background_color, duration = off_time*0.5)
                        self.show_shape(shape = 'rect',
                                    size = shape_size_i,
                                    color = color,
                                    background_color = background_color,
                                    duration = on_time,
                                    pos = p,
                                    angle=angle)
                        self.show_fullscreen(color = background_color, duration = off_time*0.5)
                        if hasattr(self, 'block_end'):
                            self.block_end(block_name = 'position')
        
    def _parse_receptive_field_parameters(self, shape_size, nrows, ncolumns, display_size, shape_colors, background_color):
        if background_color is None:
            background_color = self.experiment_config.BACKGROUND_COLOR
        if not isinstance(shape_colors, list):
            shape_colors = [shape_colors]
        if display_size is None:
            display_size = self.machine_config.SCREEN_SIZE_UM
        if shape_size is None:
            shape_size = utils.rc((display_size['row']/float(nrows), display_size['col']/float(ncolumns)))
        elif not hasattr(shape_size, 'dtype'):
            shape_size = utils.rc((shape_size, shape_size))
        if nrows is None and ncolumns is None:
            nrows = int(numpy.floor(display_size['row']/float(shape_size['row'])))
            ncolumns = int(numpy.floor(display_size['col']/float(shape_size['row'])))
        return shape_size, nrows, ncolumns, display_size, shape_colors, background_color
        
    def _receptive_field_explore_positions(self,shape_size, nrows, ncolumns):
        if shape_size.shape == (1, ):
            shape_size = shape_size[0]
        y_dir = 1 if self.machine_config.VERTICAL_AXIS_POSITIVE_DIRECTION == 'up' else -1
        first_position = utils.rc_add(self.machine_config.SCREEN_CENTER, utils.rc((shape_size['row']*(0.5*nrows-0.5)*y_dir, shape_size['col']*(0.5*ncolumns-0.5))), '-')
        positions = []
        for r in range(nrows):
            for c in range(ncolumns):
                p=utils.rc_add(first_position, utils.rc((y_dir*r*shape_size['row'], c*shape_size['col'])))
                positions.append(p)
        return positions
        
    def receptive_field_explore_durations_and_positions(self, **kwargs):
        positions = self._receptive_field_explore_positions(kwargs['shape_size'], kwargs['nrows'], kwargs['ncolumns'])
        offtime=kwargs['off_time'] if kwargs['off_time']>0 else 2.0/self.machine_config.SCREEN_EXPECTED_FRAME_RATE
        return len(positions)*len(kwargs['shape_colors'])*kwargs['flash_repeat']*kwargs['sequence_repeat']*(kwargs['on_time']+offtime)+offtime, positions
        
    def moving_shape_trajectory(self, size, speeds, directions,repetition,center=utils.rc((0,0)), 
                pause=0.0,moving_range=None, shape_starts_from_edge=False):
        '''
        Calculates moving shape trajectory and total duration of stimulus
        '''
        if not (isinstance(speeds, list) or hasattr(speeds,'dtype')):
            speeds = [speeds]
        if hasattr(size, 'dtype'):
            shape_size = max(size['row'], size['col'])
        else:
            shape_size = size
        if shape_starts_from_edge:
            self.movement = numpy.sqrt(2)*max(self.machine_config.SCREEN_SIZE_UM['row'], self.machine_config.SCREEN_SIZE_UM['col']) + shape_size
        else:
            self.movement = min(self.machine_config.SCREEN_SIZE_UM['row'], self.machine_config.SCREEN_SIZE_UM['col']) - shape_size # ref to machine conf which was started
        if moving_range is not None:
            self.movement = moving_range+ shape_size
#        print self.movement,directions,speeds
        trajectory_directions = []
        trajectories = []
        nframes = 0
        for spd in speeds:
            for direction in directions:
                end_point = utils.rc_add(utils.cr((0.5 * self.movement *  numpy.cos(numpy.radians(self.vaf*direction)), 0.5 * self.movement * numpy.sin(numpy.radians(self.vaf*direction)))), self.machine_config.SCREEN_CENTER, operation = '+')
                start_point = utils.rc_add(utils.cr((0.5 * self.movement * numpy.cos(numpy.radians(self.vaf*direction - 180.0)), 0.5 * self.movement * numpy.sin(numpy.radians(self.vaf*direction - 180.0)))), self.machine_config.SCREEN_CENTER, operation = '+')
                if spd == 0:
                    raise RuntimeError('Zero speed is not supported')
                spatial_resolution = spd/self.machine_config.SCREEN_EXPECTED_FRAME_RATE
                t=utils.calculate_trajectory(start_point,  end_point,  spatial_resolution)
                t['row'] +=center['row']
                t['col'] +=center['col']
                for rep in range(repetition):
                    trajectories.append(t)
                    nframes += trajectories[-1].shape[0]
                    trajectory_directions.append(direction)
        duration = float(nframes)/self.machine_config.SCREEN_EXPECTED_FRAME_RATE  + (len(speeds)*len(directions)*repetition+1)*pause
        return trajectories, trajectory_directions, duration
        
    def moving_shape(self, size, speeds, directions, shape = 'rect', color = 1.0, background_color = 0.0, 
                        moving_range=None, pause=0.0, repetition = 1, center = utils.rc((0,0)), 
                        block_trigger = False, shape_starts_from_edge=False,save_frame_info =True):
        '''
        Present a moving simulus in different directions:
            shape: shape of moving object, see show_shapes()
            size: size of stimulus
            speeds: list of speeds in um/s that are used for moving the shape
            directions: list of motion directions in degree 
            color: color of shape
            center: center of movement
            pause: pause between each sweep in seconds
            moving_range: range of movement in um
            shape_starts_from_edge: if True, moving shape starts from the edge of the screen
                        such that shape is not visible
        '''
        
        #TODO:
#        if hasattr(self, 'screen_center'):
#            pos_with_offset = utils.rc_add(pos, self.screen_center)
#        else:
#            pos_with_offset = pos
        self.log.info('moving_shape(' + str(size)+ ', ' + str(speeds) +', ' + str(directions) +', ' + str(shape) +', ' + str(color) +', ' + str(background_color) +', ' + str(moving_range) + ', '+ str(pause) + ', ' + str(block_trigger) + ')', source='stim')
        trajectories, trajectory_directions, duration = self.moving_shape_trajectory(size, speeds, directions,repetition,center,pause,moving_range,shape_starts_from_edge)
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(),parameters={'trajectories':trajectories})
        self.show_fullscreen(duration = 0, color = background_color, save_frame_info = False, frame_timing_pulse = False)
        for block in range(len(trajectories)):
            self.show_shape(shape = shape,  pos = trajectories[block], 
                            color = color,  background_color = background_color, 
                            orientation =self.vaf*trajectory_directions[block] , size = size,  
                            is_block = block_trigger, save_frame_info = True,  #save_frame_info = True might confuse block/repeat detection
                            enable_centering = False)
            if pause > 0:
                self.show_fullscreen(duration = pause, color = background_color, save_frame_info = True, frame_timing_pulse = True)
            if self.abort:
                break
        self.show_fullscreen(duration = 0, color = background_color, save_frame_info = True, frame_timing_pulse = True)
        if save_frame_info:
            self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
        return duration

    def sine_wave_shape(self):
        pass
        
    def moving_curtain(self,speed, color = 1.0, direction=0.0, background_color = 0.0, pause = 0.0,block_trigger = False):
        self.log.info('moving_curtain(' + str(color)+ ', ' + str(background_color) +', ' + str(speed) +', ' + str(direction) +', ' + str(pause) + ', ' + str(block_trigger) +')',source='stim')
        self._save_stimulus_frame_info(inspect.currentframe())
        movement = numpy.sqrt(self.machine_config.SCREEN_SIZE_UM['col']**2+self.machine_config.SCREEN_SIZE_UM['row']**2)
        size = utils.rc((movement, movement))
        end_point = self.config.SCREEN_CENTER
        start_point = utils.rc_add(utils.cr((0.5 * 2 * movement * numpy.cos(numpy.radians(self.vaf*direction - 180.0)), 0.5 * 2 * movement * numpy.sin(numpy.radians(self.vaf*direction - 180.0)))), self.config.SCREEN_CENTER, operation = '+')
        pos = utils.calculate_trajectory(start_point, end_point, speed/self.machine_config.SCREEN_EXPECTED_FRAME_RATE)
        if pause > 0:
            self.show_fullscreen(duration = pause, color = background_color, save_frame_info = False, frame_timing_pulse = False)
        self.show_shape(shape = 'rect',  pos = pos,  
                            color = color,  background_color = background_color,  orientation =self.vaf*direction , size = size,  is_block = block_trigger, 
                            save_frame_info = False, enable_centering = False)
        if pause > 0:
            self.show_fullscreen(duration = pause, color = color, save_frame_info = False, frame_timing_pulse = False)
        self._save_stimulus_frame_info(inspect.currentframe(), is_last = True)
        
    def point_laser_beam(self, positions, jump_time, hold_time):
        '''
        positions is an row,col array, mm dimension
        jump_time: time of jump detween positions
        hold_time: duration of holding the leaser beam in one position
        scanner voltage = atan(position/distance)/angle2voltage_factor
        '''
        if self.config.OS == 'Linux':#Not supported
            return
        #Check expected machine config parameters
        if not hasattr(self.machine_config, 'LASER_BEAM_CONTROL'):
            raise MachineConfigError('LASER_BEAM_CONTROL parameter is missing from machine config')
        channels = self.machine_config.LASER_BEAM_CONTROL['CHANNELS']
        sample_rate = self.machine_config.LASER_BEAM_CONTROL['SAMPLE_RATE']
        mirror_screen_distance = self.machine_config.LASER_BEAM_CONTROL['MIRROR_SCREEN_DISTANCE']
        angle2voltage_factor = self.machine_config.LASER_BEAM_CONTROL['ANGLE2VOLTAGE_FACTOR']
        
        #convert positions to voltage
        
        voltages = numpy.arctan(numpy.array([positions['row'], positions['col']])/mirror_screen_distance)/angle2voltage_factor
        voltages = utils.rc(numpy.concatenate((numpy.zeros((2,1)),voltages,numpy.zeros((2,1))),axis=1))
            
        from visexpman.engine.hardware_interface import daq_instrument
        waveform = utils.rc(numpy.zeros((2,0)))
        for pos_i in range(voltages.shape[0]-1):
            chunk = []
            for axis in ['row', 'col']:
                transient = numpy.linspace(voltages[pos_i][axis],voltages[pos_i+1][axis], int(jump_time*sample_rate))
                hold = numpy.ones(int(hold_time*sample_rate))*voltages[pos_i+1][axis]
                chunk.append(numpy.concatenate((transient,hold)))
            waveform = numpy.append(waveform, utils.rc(numpy.array(chunk)))
        waveform = utils.nd(waveform).T
        if abs(waveform).max()>self.machine_config.LASER_BEAM_CONTROL['MAX_SCANNER_VOLTAGE']:
            from visexpman.engine.hardware_interface.scanner_control import ScannerError
            raise ScannerError('Position(s) are beyond the scanner\'s operational range')
        daq_instrument.set_waveform(channels,waveform,sample_rate = sample_rate)
if test_mode:
    class TestStimulationPatterns(unittest.TestCase):
        
        @unittest.skip('')
        def test_01_curtain(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'GUITestConfig', 'TestCurtainConfig', ENABLE_FRAME_CAPTURE = not True)
    
        @unittest.skip('')
        def test_02_moving_shape(self):
            from visexpman.engine.visexp_app import stimulation_tester
            from visexpman.users.test.test_stimulus import TestMovingShapeConfig
            context = stimulation_tester('test', 'GUITestConfig', 'TestMovingShapeConfig', ENABLE_FRAME_CAPTURE = True)
            ec = TestMovingShapeConfig(context['machine_config'])
            bgcolor = colors.convert_color(ec.SHAPE_BACKGROUND, context['machine_config'])[0]*255
            calculated_duration = float(fileop.read_text_file(context['logger'].filename).split('\n')[0].split(' ')[-1])
            captured_files = map(os.path.join, len(os.listdir(context['machine_config'].CAPTURE_PATH))*[context['machine_config'].CAPTURE_PATH],os.listdir(context['machine_config'].CAPTURE_PATH))
            captured_files.sort()
            captured_files=captured_files[1:]#Drop first frame which is some garbage from the video buffer
            #remove menu frames (red)
            stim_frames = [captured_file for captured_file in captured_files if not (numpy.asarray(Image.open(captured_file))[:,:,0].sum() > 0 and numpy.asarray(Image.open(captured_file))[:,:,1:].sum() == 0)]
            #Check pause durations
            overall_intensity = [numpy.asarray(Image.open(f)).sum() for f in stim_frames]
            mean_intensity_per_frame = overall_intensity/(context['machine_config'].SCREEN_RESOLUTION['row']*context['machine_config'].SCREEN_RESOLUTION['col']*3)
            edges = signal.trigger_indexes(mean_intensity_per_frame)
            pauses = numpy.diff(edges[1:-1])[::2]/float(context['machine_config'].SCREEN_EXPECTED_FRAME_RATE)
            self.assertGreaterEqual(pauses.min(), ec.PAUSE_BETWEEN_DIRECTIONS)
            #TODO: check captured files: shape size, speed
            numpy.testing.assert_almost_equal((len(stim_frames)-2)/float(context['machine_config'].SCREEN_EXPECTED_FRAME_RATE), calculated_duration, int(-numpy.log10(3.0/context['machine_config'].SCREEN_EXPECTED_FRAME_RATE))-1)
    
        @unittest.skip('')
        def test_03_natural_stim_spectrum(self):
            from visexpman.engine.visexp_app import stimulation_tester
            from PIL import Image
            from visexpman.engine.generic import fileop
            spd = 800
            duration = 12
            repeats = 1
            context = stimulation_tester('test', 'NaturalStimulusTestMachineConfig', 'TestNaturalStimConfig', ENABLE_FRAME_CAPTURE = not True,
                    DURATION = duration, REPEATS = repeats, DIRECTIONS = [0], SPEED=spd,MSP=120,CIRCULAR= True)
            intensities = []
            fns = fileop.listdir_fullpath(context['machine_config'].CAPTURE_PATH)
            #Check if number of frames generated corresponds to duration, repeat and frame rate
            self.assertAlmostEqual(len(fns), duration*repeats*context['machine_config'].SCREEN_EXPECTED_FRAME_RATE,delta=5)
            fns.sort()
            for f in fns[1:]:#First frame might be some garbage in the frame buffer
                im = numpy.asarray(Image.open(f))
                first_column = im[:,0]
                self.assertEqual(first_column.std(),0)#Check if columns have the same color
                intensities.append(first_column.mean())
            intensities = numpy.array(intensities)
            spectrum = abs(numpy.fft.fft(intensities))/2/intensities.shape[0]
            spectrum = spectrum[:spectrum.shape[0]/2]
            #TODO: test for checking periodicity
            #TODO: test for checking 1/x spectrum
    
        @unittest.skip('')
        def test_04_natural_export(self):
            export = True
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'NaturalStimulusTestMachineConfig', 'TestNaturalStimConfig', ENABLE_FRAME_CAPTURE = export,
                    STIM2VIDEO = export, OUT_PATH = '/mnt/rzws/dataslow/natural_stimulus',
                    EXPORT_INTENSITY_PROFILE = export,
                    DURATION = 3.0, REPEATS = 2, DIRECTIONS = range(0, 360, 180), SPEED=300,SCREEN_PIXEL_TO_UM_SCALE = 1.0, SCREEN_UM_TO_PIXEL_SCALE = 1.0)
    
    #    @unittest.skipIf(unittest_aggregator.TEST_os != 'Linux',  'Supported only on Linux')    
        @unittest.skip('')
        def test_05_export2video(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'GUITestConfig', 'TestVideoExportConfig', ENABLE_FRAME_CAPTURE = True)
            videofile = os.path.join(context['machine_config'].EXPERIMENT_DATA_PATH, 'out.mp4')
            self.assertTrue(os.path.exists(videofile))
            self.assertGreater(os.path.getsize(videofile), 30e3)
            os.remove(videofile)
        
        @unittest.skip('Funtion is not ready')
        def test_06_texture(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'TextureTestMachineConfig', 'TestTextureStimConfig', ENABLE_FRAME_CAPTURE = False)
            
        @unittest.skipIf(not unittest_aggregator.TEST_daq,  'Daq tests disabled')
        def test_07_point_laser_beam(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'LaserBeamTestMachineConfig', 'LaserBeamStimulusConfig')
    
        @unittest.skipIf(not unittest_aggregator.TEST_daq,  'Daq tests disabled')
        def test_08_point_laser_beam_out_of_range(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'LaserBeamTestMachineConfig', 'LaserBeamStimulusConfigOutOfRange')
            self.assertIn('ScannerError', fileop.read_text_file(context['logger'].filename))
            
        @unittest.skip('')
        def test_09_show_grating_non_texture(self):
            from visexpman.engine.visexp_app import stimulation_tester
            from visexpman.users.test.test_stimulus import TestMovingShapeConfig
            context = stimulation_tester('test', 'GUITestConfigPix', 'TestNTGratingConfig', ENABLE_FRAME_CAPTURE = False)
        
        @unittest.skip('')
        def test_10_block_trigger(self):
            from visexpman.engine.visexp_app import stimulation_tester
            import hdf5io
            from visexpman.users.test.test_stimulus import TestStimulusBlockParams
            context = stimulation_tester('test', 'GUITestConfig', 'TestStimulusBlockParams')
            ec = TestStimulusBlockParams(context['machine_config'])
            stim_context = hdf5io.read_item(fileop.get_context_filename(context['machine_config']), 'context', filelocking=False)
            sfi=utils.array2object(stim_context)['last_experiment_stimulus_frame_info']
            expected_number_of_blocks = len(ec.COLORS)*2**2+len(ec.IMAGE_FOLDERS)*len(ec.IMAGE_STRETCHES)+len(ec.SHAPES)*len(ec.T_SHAPE)*len(ec.POSITIONS)*len(ec.SIZES)*len(ec.COLORS)
            expected_number_of_blocks += numpy.prod(map(len, [getattr(ec, p) for p in ['T_GRATING', 'GRATING_PROFILES', 'GRATING_WIDTH', 'GRATING_SPEEDS']]))
            expected_number_of_blocks += 2
            if 0:
                for s in sfi:
                    if s.has_key('block_start') or s.has_key('block_end'):
                        print s.keys()
                    else:
                        print s['stimulus_type']
            self.assertEqual(len([s for s in sfi if 'block_start' in s]), expected_number_of_blocks)
            self.assertEqual(len([s for s in sfi if 'block_end' in s]), expected_number_of_blocks)
            #block start and block end entries must be adjacent, no stimulus info should be in between
            self.assertEqual((numpy.array([i for i in range(len(sfi)) if 'block_end' in sfi[i]])-numpy.array([i for i in range(len(sfi)) if 'block_start' in sfi[i]])).sum(), expected_number_of_blocks)
            #Check if block indexes are increasing:
            bidiff = numpy.diff(numpy.array([s['block_start' if s.has_key('block_start') else 'block_end'] for s in sfi if 'block_start' in s or 'block_end' in s]))
            self.assertGreaterEqual(bidiff.min(),0)
        
        @unittest.skip('')    
        def test_11_checkerboard(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'GUITestConfig', 'TestCheckerboardConfig', ENABLE_FRAME_CAPTURE = False)
        
        @unittest.skip('')
        def test_12_movinggrating(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'GUITestConfig', 'TestGratingConfig', ENABLE_FRAME_CAPTURE = False)
        
        @unittest.skip('')
        def test_13_receptive_field(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'GUITestConfig', 'ReceptiveFieldExploreNewAngle', ENABLE_FRAME_CAPTURE = False)
            
        def test_14_time_indexing(self):
            from visexpman.engine.visexp_app import stimulation_tester
            context = stimulation_tester('test', 'GUITestConfig', 'TestTimeIndexing', ENABLE_TIME_INDEXING = False)
            context = stimulation_tester('test', 'GUITestConfig', 'TestTimeIndexing', ENABLE_TIME_INDEXING = True)
    

if __name__ == "__main__":
    unittest.main()
