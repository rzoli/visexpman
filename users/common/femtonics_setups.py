import os
import os.path
import numpy,copy,hdf5io
from visexpman.engine.generic import utils
from visexpman.engine.vision_experiment.configuration import AoCorticalCaImagingConfig

class AOSetup(AoCorticalCaImagingConfig):
    def _set_user_parameters(self):
        AoCorticalCaImagingConfig._set_user_parameters(self)
        # Files
        self.EXPERIMENT_FILE_FORMAT = 'hdf5'
        self.LOG_PATH = 'v:\\log'
        self.EXPERIMENT_DATA_PATH = 'v:\\experiment_data_ao'
        self.CONTEXT_PATH='v:\\context'
        #Stimulus screen
        self.SCREEN_DISTANCE_FROM_MOUSE_EYE = 190.0
        self.SCREEN_RESOLUTION = utils.cr([1280, 720])
        self.SCREEN_PIXEL_WIDTH = 477.0/self.SCREEN_RESOLUTION ['col']
        self.SCREEN_EXPECTED_FRAME_RATE = 60.0
        self.SCREEN_MAX_FRAME_RATE = 60.0
        self.FULLSCREEN=True
        self.COORDINATE_SYSTEM='ulcorner'
        self.ENABLE_FRAME_CAPTURE = False
        self.GUI['SIZE'] =  utils.cr((1024,768)) 
        #Network
        stim_computer_ip = '172.27.26.69'
        self.CONNECTIONS['stim']['ip']['stim'] = stim_computer_ip
        self.CONNECTIONS['stim']['ip']['main_ui'] = stim_computer_ip
        #Command relay server is used for conencting to MES because no zmq is supported and mes works only in client mode
        self.COMMAND_RELAY_SERVER={}
        self.COMMAND_RELAY_SERVER['RELAY_SERVER_IP'] = stim_computer_ip
        self.COMMAND_RELAY_SERVER['TIMEOUT'] = 5
        self.COMMAND_RELAY_SERVER['CLIENTS_ENABLE'] = True
        self.COMMAND_RELAY_SERVER['ENABLE'] = True
        self.COMMAND_RELAY_SERVER['RELAY_SERVER_IP_FROM_TABLE'] = True
        self.COMMAND_RELAY_SERVER['CONNECTION_MATRIX'] = \
            {
            'STIM_MES'  : {'STIM' : {'IP': stim_computer_ip, 'LOCAL_IP': '', 'PORT': 10002}, 'MES' : {'IP': '', 'LOCAL_IP': '', 'PORT': 10003}}, 
            }
        self.COMMAND_RELAY_SERVER['SERVER_IP'] = {\
                     'GUI_MES': ['',''],
                     'STIM_MES': ['',''],
                     'GUI_STIM': ['', stim_computer_ip],
                     'GUI_ANALYSIS'  : ['', stim_computer_ip],
        }   
        #Sync signal
        self.SYNC_RECORDER_CHANNELS='Dev1/ai0:3'#0: ao, 1: frame sync, 2: block, 3: ao
        self.SYNC_RECORDER_SAMPLE_RATE=40000#mes sync pulses are very short
        self.SYNC_RECORDING_BUFFER_TIME=5.0
        self.TIMG_SYNC_INDEX=3
        self.TSTIM_SYNC_INDEX=2
        self.DIGITAL_IO_PORT='COM4'
        self.BLOCK_TRIGGER_PIN = 1
        self.FRAME_TRIGGER_PIN = 0
        
        self.MES_RECORD_OVERHEAD=12
        self.MES_RECORD_START_WAITTIME=6
        self.SYNC_RECORD_OVERHEAD=5
        self.GAMMA_CORRECTION = copy.deepcopy(hdf5io.read_item(os.path.join(self.CONTEXT_PATH, 'gamma_ao_cortical_monitor.hdf5'), 'gamma_correction'))
        
        
        
