import pygame
import copy
import socket
import time

from visexpman.engine.generic import utils
from visexpman.engine.generic import colors
from visexpman.engine.generic import graphics
from visexpman.engine.generic import fileop
try:
    import Image
except ImportError:
    from PIL import Image

from OpenGL.GL import *#TODO: perhaps this is not necessary
from OpenGL.GLUT import *

def experiment_choices(experiment_list):
    '''
    Lists and displays stimulus files that can be found in the default stimulus file folder
    '''
    return '\n'.join([str(i)+' '+experiment_list[i][1].__name__ for i in range(len(experiment_list))])
    
class CaImagingScreen(graphics.Screen):
    '''
    Graphical screen of ca-imaging application
    '''
    def __init__(self):
        if self.config.FULLSCREEN:
            screen_resolution = graphics.get_screen_size()
        else:
            screen_resolution = utils.cr((1000, 700))
        graphics.Screen.__init__(self, self.config, screen_resolution = screen_resolution, graphics_mode = 'external')
        self.clear_screen()
        import numpy
        im = numpy.ones((200,100,3),dtype=numpy.float)
        im *=0.4
        im[0,0,:]=1.0
        im[0,5:6,1]=1.0
        im[0,5:6,0]=0.0
        im[0,5:6,2]=0.0
        im[5:6,0,2]=1.0
        im[5:6,0,:2]=0.0
        im[:2,:,0]=1.0
        im[-2:,:,0]=1.0
        im[:,:2,0]=1.0
        im[:,-2:,0]=1.0
        for i in range(10):
            im[:,10+i,1] = numpy.arange(im[:,10,1].shape[0],dtype=numpy.float)/im[:,10,1].shape[0]
        
        self.im = im
#        import Image
#        self.im = numpy.cast['float'](numpy.asarray(Image.open('/home/rz/rzws/codes/visexpman/data/images/default.bmp')))
#        self.im.flags.writeable = True
#        self.im/=255.0
#        Image.fromarray(numpy.cast['uint8'](self.im*255)).show()
        
    def refresh(self):
        self.clear_screen(color = colors.convert_color(0.0))
        self.render_image(self.im)
#        self.render_imagefile('/home/rz/rzws/codes/visexpman/data/images/default.bmp')
        self.flip()
    
    
    
class StimulationScreen(graphics.Screen):
    '''
    graphics.Screen is amended with vision experiment specific features: menu&message displaying
    '''    
    def __init__(self):
        graphics.Screen.__init__(self, self.config, graphics_mode = 'external')
        self.clear_screen()
        #== Initialize displaying text ==
        self.text_style = GLUT_BITMAP_8_BY_13
        self.max_lines = int(self.config.SCREEN_RESOLUTION['row']/13.0)
        self.max_chars =  int(self.config.SCREEN_RESOLUTION['col']/(8+13.0))
        self.menu_text = 'cursors - adjust screen center, '
        commands = self.config.KEYS.keys()
        commands.sort()
        for k in commands:
            self.menu_text+= '{0} - {1}, '.format(self.config.KEYS[k], k)
        self.menu_text = self.menu_text[:-2]
        #Split menu text to lines
        parts = [[]]
        self.menu_lines = 0
        char_count = 0
        for item in self.menu_text.split(','):
            char_count += len(item)
            if char_count > self.max_chars:
                char_count = 0
                self.menu_lines += 1
                parts.append([])
            parts[self.menu_lines].append(item)
        self.menu_text = '\n'
        for part in parts:
            self.menu_text = self.menu_text + ', '.join(part) + '\n'
        self.max_print_lines = self.max_lines-self.menu_lines-6
        self.screen_text = ''
        self.show_text = True
        self.show_bullseye = False
        self.bullseye_size = None
        self.text_color = colors.convert_color(self.config.TEXT_COLOR, self.config)
        self.text_position = copy.deepcopy(self.config.UPPER_LEFT_CORNER)
        self.text_position['row'] -= 13
        self.text_position['col'] += 13
        self.refresh_non_experiment_screen()
        
    def clear_screen_to_background(self):
        self.clear_screen(color = colors.convert_color(self.stim_context['background_color'], self.config))
        
    def _display_bullseye(self):
        if self.show_bullseye:
            if self.bullseye_size is None:
                bullseye_path = os.path.join(self.config.PACKAGE_PATH, 'data', 'images', 'bullseye.bmp')
            else:
                self.bullseye_size_in_pixel = int(float(self.bullseye_size) * self.config.SCREEN_UM_TO_PIXEL_SCALE)
                bullseye_path = fileop.get_tmp_file('bmp')
                im = Image.open(os.path.join(self.config.PACKAGE_PATH, 'data', 'images', 'bullseye.bmp'))
                im = im.resize((self.bullseye_size_in_pixel, self.bullseye_size_in_pixel))
                im.save(bullseye_path)
            self.render_imagefile(bullseye_path, position = utils.rc_x_const(self.stim_context['screen_center'], self.config.SCREEN_UM_TO_PIXEL_SCALE))
            
    def refresh_non_experiment_screen(self, flip = True):
        '''
        Render menu and message texts to screen
        '''
        self.clear_screen_to_background()
        self._display_bullseye()
        if self.show_text and self.config.ENABLE_TEXT:
            self.render_text(self.menu_text +'\n\n\n' + self.screen_text, color = self.text_color, position = self.text_position, text_style = self.text_style)
        #TODO: call  prerun method of pre experiment if exists
        self.flip()

    
class ScreenAndKeyboardHandler(StimulationScreen):
    '''
    VisexpmanScreen is amended with keyboard handling
    '''
    def __init__(self):
        StimulationScreen.__init__(self)
        self.command_domain = 'keyboard'
        import Queue
        self.keyboard_command_queue = Queue.Queue()
        self.load_keyboard_commands()
            
    def load_keyboard_commands(self):        
        self.keyboard_commands = copy.deepcopy(self.config.COMMANDS)
        self.separator = '@'
        if hasattr(self, 'experiment_config_list'):
            self.experiment_config_shortcuts = ['{0}'.format(i) for i in range(len(self.experiment_config_list))]#stimulus_file_shortcut
            for shortcut in self.experiment_config_shortcuts:
                self.keyboard_commands['select_experiment' + self.separator + shortcut] = {'key': shortcut, 'domain' : [self.command_domain]}
        else:
            self.experiment_config_shortcuts = []
        
    def _parse_keyboard_command(self, key_pressed, domain):
        '''
        If pressed key valid, generate command string.
        '''
        command = None
        parameter = None
        for k, v in self.keyboard_commands.items():
            if v['key'] == key_pressed and utils.is_in_list(v['domain'], domain) :
                command_and_parameters = k.split(self.separator)
                command = command_and_parameters[0]
                if len(command_and_parameters) == 2:
                    parameter = command_and_parameters[1]
                break
        if command != None:
            command = 'SOC' + command + 'EOC'
            if parameter != None:
                command += parameter + 'EOP'
        return command
        
    def keyboard_handler(self, domain):
        '''
        Registers pressed key and generates command string for command handler.
        '''
        return self._parse_keyboard_command(check_keyboard(),  domain)
        

    def user_interface_handler(self):
        '''
        Updates menu and message on screen, takes care of handling the keyboard
        '''
        self.refresh_non_experiment_screen()
        command = self.keyboard_handler(self.command_domain)
        #Send command to queue
        if command != None:
            self.keyboard_command_queue.put(command)

def check_keyboard():
    '''
    Get pressed key
    '''        
    keys_pressed = []
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            key_pressed = pygame.key.name(event.key)                
            keys_pressed.append(key_pressed)
    return keys_pressed
    
def is_key_pressed(key):
    return key in check_keyboard()
    
if __name__ == "__main__":
    pass
