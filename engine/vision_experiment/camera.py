import os,time, numpy, hdf5io, traceback, multiprocessing, serial

import PyQt5.Qt as Qt
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore

from visexpman.engine.generic import gui,fileop, signal, utils, introspect
from visexpman.engine.hardware_interface import daq,digital_io,openephys, camera, camera_interface
from visexpman.engine.vision_experiment import gui_engine, main_ui,experiment_data
from visexpman.engine.analysis import behavioral_data

NEW_FUSI_CONTOL=True

class Camera(gui.VisexpmanMainWindow):
    def __init__(self, context):        
        if QtCore.QCoreApplication.instance() is None:
            qt_app = Qt.QApplication([])
        gui.VisexpmanMainWindow.__init__(self, context)
        self.setWindowIcon(gui.get_icon('behav'))
        self._init_variables()
        self.resize(self.machine_config.GUI_WIDTH, self.machine_config.GUI_HEIGHT)
        if hasattr(self.machine_config, 'CAMERA_GUI_POS_X'):
            self.move(self.machine_config.CAMERA_GUI_POS_X, self.machine_config.CAMERA_GUI_POS_Y)
        self._set_window_title()
        toolbar_buttons = ['record', 'stop', 'convert_folder', 'exit']
            
        self.toolbar = gui.ToolBar(self, toolbar_buttons)
        self.addToolBar(self.toolbar)
        self.statusbar=self.statusBar()
        self.statusbar.recording_status=QtGui.QLabel('', self)
        self.statusbar.addPermanentWidget(self.statusbar.recording_status)
        self.statusbar.recording_status.setStyleSheet('background:gray;')
        
        self.statusbar.trigger_status=QtGui.QLabel('', self)
        self.statusbar.addPermanentWidget(self.statusbar.trigger_status)
        self.statusbar.recording_status.setStyleSheet('background:gray;')

        self.debug = gui.Debug(self)
        self._add_dockable_widget('Debug', QtCore.Qt.BottomDockWidgetArea, QtCore.Qt.BottomDockWidgetArea, self.debug)        
        self.image = gui.Image(self)
        self._add_dockable_widget('Camera 1', QtCore.Qt.RightDockWidgetArea, QtCore.Qt.RightDockWidgetArea, self.image)
        if hasattr(self.machine_config,'ENABLE_USER_FOLDER') and self.machine_config.ENABLE_USER_FOLDER:
            self.filebrowserroot= os.path.join(self.machine_config.EXPERIMENT_DATA_PATH,self.machine_config.user)
        else:
            self.filebrowserroot= self.machine_config.EXPERIMENT_DATA_PATH
        self.params = gui.ParameterTable(self, self.params_config)
        self.params.params.sigTreeStateChanged.connect(self.parameter_changed)
        self.main_tab = QtGui.QTabWidget(self)
        if self.machine_config.PLATFORM=='behav':
            self.datafilebrowser = main_ui.DataFileBrowser(self, self.filebrowserroot, ['behav*.hdf5', 'stim*.hdf5', 'eye*.hdf5',   'data*.hdf5', 'data*.mat','*.mp4'])
            self.main_tab.addTab(self.datafilebrowser, 'Data Files')        
        self.main_tab.addTab(self.params, 'Settings')
        self.camera2image=gui.Image(self)
        self.main_tab.addTab(self.camera2image, 'Camera 2')
        self.main_tab.setCurrentIndex(0)
        self.main_tab.setTabPosition(self.main_tab.South)
        self._add_dockable_widget('Main', QtCore.Qt.LeftDockWidgetArea, QtCore.Qt.LeftDockWidgetArea, self.main_tab)
        self.show()
        
        self.update_image_timer=QtCore.QTimer()
        self.update_image_timer.timeout.connect(self.update_image)
        self.update_image_timer.start(1000/self.machine_config.DEFAULT_CAMERA_FRAME_RATE/3)#ms
            
        self.context_filename = fileop.get_context_filename(self.machine_config,'npy')
        if os.path.exists(self.context_filename):
            context_stream = numpy.load(self.context_filename)
            self.parameters=utils.array2object(context_stream)
        else:
            self.parameter_changed()
        self.load_all_parameters()
        self.camera_api=self.machine_config.CAMERA_API if hasattr(self.machine_config, 'CAMERA_API') else ''
        self.camera1_io_port= self.machine_config.CAMERA1_IO_PORT if hasattr(self.machine_config, 'CAMERA1_IO_PORT') else None
        self.camera2_io_port= self.machine_config.CAMERA2_IO_PORT if hasattr(self.machine_config, 'CAMERA2_IO_PORT') else None
        if self.camera_api=='tisgrabber':
            self.camera1handler=camera.ISCamera(self.machine_config.CAMERA1_ID, self.logger.filename.replace('.txt', '_cam1.txt'), self.camera1_io_port, frame_rate=self.parameters['params/Frame Rate'], exposure=self.parameters['params/Exposure time']*1e-3)
        elif self.camera_api=='blackfly':
            self.camera1handler=camera.BlackFlySCamera(self.logger.filename.replace('.txt', '_cam1.txt'),None, frame_rate=self.parameters['params/Frame Rate'], auto_exposure_time = False, exposure_time_us=self.parameters['params/Exposure time']*1000, filename=None, show=False, FrameStart_out_enabled = True)
            self.camera1handler.ttl_disable()
        else:
            self.camera1handler=camera_interface.ImagingSourceCameraHandler(self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3, self.camera1_io_port)
        self.camera1handler.start()
        
        if self.machine_config.CAMERA2_ENABLE:
            if self.camera_api=='tisgrabber':
                self.camera2handler=camera.ISCamera(self.machine_config.CAMERA2_ID, self.logger.filename.replace('.txt', '_cam2.txt'),  self.camera2_io_port,  frame_rate=self.parameters['params/Frame Rate'], exposure=self.parameters['params/Exposure time']*1e-3)
            else:
                self.camera2handler=camera.WebCamera(self.machine_config.EYECAM_ID,os.path.join(self.machine_config.LOG_PATH, 'log_eyecam.txt'),None,filename=None)
            self.camera2handler.start()
        
        self.trigger_detector_enabled=False
        self.triggered_recording=False
#        if hasattr(self.machine_config,  'TRIGGER_DETECTOR_PORT'):
#            self.ioboard=serial.Serial(self.machine_config.TRIGGER_DETECTOR_PORT, 1000000, timeout=0.001)
#            self.trigger_detector_enabled=False
#            self.start_trigger=False
#            self.stop_trigger=False
#        time.sleep(2)
#        if self.machine_config.ENABLE_SYNC=='camera':
#            self.daqqueues = {'command': multiprocessing.Queue(), 
#                                'response': multiprocessing.Queue(), 
#                                'data': multiprocessing.Queue()}
#            limits = {}
#            limits['min_ao_voltage'] = -5.0
#            limits['max_ao_voltage'] = 5.0
#            limits['min_ai_voltage'] = -5.0
#            limits['max_ai_voltage'] = 5.0
#            limits['timeout'] = self.machine_config.DAQ_TIMEOUT
#            self.ai=daq_instrument.AnalogIOProcess('daq', self.daqqueues, None, ai_channels=self.machine_config.SYNC_RECORDER_CHANNELS,limits=limits)
#            self.ai.start()
        
        self.debug.setFixedHeight(self.machine_config.GUI_HEIGHT*0.4)
        if QtCore.QCoreApplication.instance() is not None:
            QtCore.QCoreApplication.instance().exec_()

    def _init_variables(self):
        self.recording=False
        self.track=[]
        self.trigger_state='off'
        if self.machine_config.PLATFORM in ['2p', 'resonant', 'generic']:
            trigger_value = 'network' 
            params=[]
        elif self.machine_config.PLATFORM in ['behav']:
            trigger_value='TTL pulses'
            params=[
                {'name': 'Show track', 'type': 'bool', 'value': True}, 
                {'name': 'Threshold', 'type': 'int', 'value': 140},
                {'name': 'ROI size', 'type': 'int', 'value': 20},
                {'name': 'Show color LEDs', 'type': 'bool', 'value': False}, 
                ]
        else:
            trigger_value='manual'
            params=[]
        self.params_config = [
                {'name': 'Experiment Name', 'type': 'str', 'value': ''},
		{'name': 'Enable live view', 'type': 'bool', 'value': True},
                {'name': 'Trigger', 'type': 'list', 'values': ['file','manual', 'network', 'TTL pulses'], 'value': trigger_value},
                {'name': 'Enable trigger', 'type': 'bool', 'value': False,   'readonly': not self.machine_config.ENABLE_TRIGGERING}, 
                {'name': 'Frame Rate', 'type': 'float', 'value': 50, 'siPrefix': True, 'suffix': 'Hz'},
                {'name': 'Exposure time', 'type': 'float', 'value': 39, 'siPrefix': True, 'suffix': 'ms'},
                {'name': 'Enable ROI cut', 'type': 'bool', 'value': False},
                {'name': 'ROI x1', 'type': 'int', 'value': 200},
                {'name': 'ROI y1', 'type': 'int', 'value': 200},
                {'name': 'ROI x2', 'type': 'int', 'value': 400},
                {'name': 'ROI y2', 'type': 'int', 'value': 400},
                {'name': 'Recording timeout', 'type': 'float', 'value': 60, 'suffix': 's', 'siPrefix': False, 'decimals':6},
                {'name': 'fUSI enable', 'type': 'bool', 'value': False},
                {'name': 'Neuropixel SMA port TRIG', 'type': 'bool', 'value': False},
                    ]
        if not NEW_FUSI_CONTOL:
                self.params_config.extend([
                {'name': 'fUSI sampling rate', 'type': 'float', 'value': 5, 'suffix': 'Hz', 'siPrefix': True},
                {'name': 'fUSI Nimag', 'type': 'int', 'value': 4000},
                ])
        self.params_config.extend(params)
        self.openephys_running=False
        
    def save_context(self):
        context_stream=utils.object2array(self.parameters)
        numpy.save(self.context_filename,context_stream)
        
    def restart_camera(self):
        if hasattr(self, 'camera1handler'):
            self.printc('Restart camera')
            self.camera1handler.stop()
            if self.camera_api=='tisgrabber':
                self.camera1handler=camera.ISCamera(self.machine_config.CAMERA1_ID, self.logger.filename.replace('.txt', '_cam1.txt'), self.camera1_io_port, self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3)
            elif self.camera_api=='blackfly':
                self.camera1handler=camera.BlackFlySCamera(self.logger.filename.replace('.txt', '_cam1.txt'),None, frame_rate=self.parameters['params/Frame Rate'], auto_exposure_time = False, exposure_time_us=self.parameters['params/Exposure time']*1000, filename=None, show=False, FrameStart_out_enabled = True)
            else:
                self.camera1handler=camera_interface.ImagingSourceCameraHandler(self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3,  None)
            self.camera1handler.start()
            if self.machine_config.CAMERA2_ENABLE:
                self.camera2handler.stop()
                self.camera2handler=camera.ISCamera(self.machine_config.CAMERA2_ID, self.logger.filename.replace('.txt', '_cam2.txt'), self.camera2_io_port, self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3)
                self.camera2handler.start()
    
    def start_recording(self,  experiment_parameters=None):
        self.setFocus()
        try:
            if self.recording:
                return
            if self.machine_config.ENABLE_OPENEPHYS_TRIGGER and self.machine_config.OPENEPHYS_COMPUTER_IP=='127.0.0.1' and not introspect.is_process_running('open-ephys'):
                QtGui.QMessageBox.question(None,'Warning', 'Open-ephys GUI is not running', QtGui.QMessageBox.Ok)
                return
            if 1000/self.parameters['params/Frame Rate']<self.parameters['params/Exposure time']:
                msg='Exposure time is too long for this frame rate!'
                self.printc(msg)
                QtGui.QMessageBox.question(self, 'Warning', msg, QtGui.QMessageBox.Ok)
                return
            self.recording=True
            self.experiment_name_tag=self.parameters['params/Experiment Name']
            if ',' in self.experiment_name_tag or ':' in self.experiment_name_tag:
                raise ValueError('Please do not use these characters in experiment name: \',:,;')
            self.printc('Start video recording')
            self.statusbar.recording_status.setStyleSheet('background:yellow;')
            self.statusbar.recording_status.setText('Preparing')
#            self.camera1handler.stop()
#            if self.machine_config.CAMERA2_ENABLE:
#                self.camera2handler.stop()
            time.sleep(self.machine_config.CAMERA_START_DELAY)
            if self.machine_config.ENABLE_SYNC=='camera':
                self.aifix=True
                if self.aifix:
                    import PyDAQmx
                    PyDAQmx.DAQmxResetDevice('Dev1')#Does it solve daq memory error showing up after several recordings?
                    daq.set_digital_line(self.machine_config.MC_STOP_TRIGGER,0)
                    self.ai=daq.AnalogRead(self.machine_config.SYNC_RECORDER_CHANNELS,
                                                self.machine_config.SYNC_MAX_RECORDING_TIME, 
                                                self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
                    time.sleep(1)
                    self.t0=time.time()
                else:
                    self.ai=daq.AnalogRecorder(self.machine_config.SYNC_RECORDER_CHANNELS, 
                            self.machine_config.SYNC_RECORDER_SAMPLE_RATE, 
                            differential=self.machine_config.SYNC_DIFF_ENABLE, 
                            save_mode='queue', 
                            buffertime=30)
                    self.ai.start()
                    self.sync=numpy.empty([0, self.ai.number_of_ai_channels])
                    self.t0=time.time()
                    while True:
                        if not self.ai.responseq.empty():
                            self.printc(self.ai.responseq.get())
                            break
                        if time.time()-self.t0>20:
                            break
                        time.sleep(0.5)
                        QtCore.QCoreApplication.instance().processEvents()
                self.printc('Recording timing signals started.')
#                d=self.ai.read_ai()#Empty ai buffer
#                self.ai.start_daq(ai_sample_rate = self.machine_config.SYNC_RECORDER_SAMPLE_RATE,
#                                ai_record_time=self.machine_config.SYNC_RECORDING_BUFFER_TIME, timeout = 10) 
                msg='Camera&Sync recording'
            else:
                self.aifix=False
                msg='Camera recording'
            if hasattr(experiment_parameters, 'keys') and 'eyecamfilename' in experiment_parameters:
                self.cam1fn=experiment_parameters['eyecamfilename']
                if self.machine_config.CAMERA2_ENABLE:
                    raise NotImplementedError('Fix this!')
            else:
                if not self.machine_config.ENABLE_USER_FOLDER:
                    outfolder=os.path.join(self.machine_config.EXPERIMENT_DATA_PATH, utils.timestamp2ymd(time.time(), separator=''))
                else:
                    outfolder=os.path.join(self.machine_config.EXPERIMENT_DATA_PATH, self.machine_config.user, utils.timestamp2ymd(time.time(), separator=''))
                if not os.path.exists(outfolder):
                    os.makedirs(outfolder)
                id=experiment_data.get_id()
                self.cam1fn=experiment_data.get_recording_path(self.machine_config, {'outfolder': outfolder,  'id': id, 'tag': self.experiment_name_tag},postfix = self.machine_config.CAM1FILENAME_TAG,extension='.mp4')
                if self.machine_config.CAMERA2_ENABLE:
                    self.cam2fn=experiment_data.get_recording_path(self.machine_config, {'outfolder': outfolder,  'id': id, 'tag': self.experiment_name_tag},postfix = self.machine_config.CAM2FILENAME_TAG,extension='.mp4')
                self.metadatafn=fileop.replace_extension(self.cam1fn, '.mat').replace(self.machine_config.CAM1FILENAME_TAG, '_sync_info')
            if self.machine_config.ENABLE_OPENEPHYS_TRIGGER and self.parameters['params/Neuropixel SMA port TRIG'] :
                tag=f'{id}_{self.experiment_name_tag}'
                try:
                    if not openephys.start_recording(self.machine_config.OPENEPHYS_COMPUTER_IP, tag=tag):
                        self.printc('Openephys cannot be triggered, is it started?')
                    else:
                        self.printc('Start Openephys')
                        self.openephys_running=True
                except:
                    self.recording=False
                    self.printc('NP cannot be triggered')
                    return
                time.sleep(self.machine_config.OPENEPHYS_PRETRIGGER_TIME)
                if 0 and self.machine_config.OPENEPHYS_DATA_PATH is not None:
                    #Check if openephys related file has appeared
                    latest_file=fileop.find_latest(self.machine_config.OPENEPHYS_DATA_PATH)
                    self.openephys_datafolder=os.path.join(self.machine_config.OPENEPHYS_DATA_PATH,latest_file.replace(self.machine_config.OPENEPHYS_DATA_PATH,'').split(os.sep)[1])
                    latest_file_age=fileop.file_age(latest_file)
                    if latest_file_age>self.machine_config.OPENEPHYS_PRETRIGGER_TIME:
                        openephys.stop_recording()
                        self.recording=False
                        msg='Openephys GUI does not save file to {0}, recording terminated'.format(self.machine_config.OPENEPHYS_DATA_PATH)
                        self.printc(msg)
                        QtGui.QMessageBox.question(self, "Warning", msg, QtGui.QMessageBox.Ok)
                        self.statusbar.recording_status.setStyleSheet('background:gray;')
                        self.statusbar.recording_status.setText('Ready')
                    
            if self.camera_api=='tisgrabber':
                self.camera1handler.save(self.cam1fn)
                if self.machine_config.CAMERA2_ENABLE:
                    self.camera2handler.save(self.cam2fn)
                #self.camera1handler=camera.ISCamera(self.machine_config.CAMERA1_ID, self.logger.filename.replace('.txt', '_cam1.txt'), self.camera1_io_port, self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3, filename=self.cam1fn)
            elif self.camera_api=='blackfly':
                #self.camera1handler.ttl_enable()
                self.camera1handler.save(self.cam1fn)
            else:
                #OBSOLETE
                self.camera1handler=camera_interface.ImagingSourceCameraHandler(self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3,  self.machine_config.CAMERA_IO_PORT,  filename=self.cam1fn, watermark=True)
            #self.camera1handler.start()
#            if self.machine_config.CAMERA2_ENABLE:
#                raise NotImplementedError('')
#                self.camera2handler=camera.WebCamera(self.machine_config.EYECAM_ID,os.path.join(self.machine_config.LOG_PATH, 'log_eyecam.txt'),self.machine_config.EYECAMERA_IO_PORT,filename=self.eyefn)
#                self.camera2handler.start()
            if self.parameters['params/Neuropixel SMA port TRIG']:
                self.printc('ZMQ trigger to NP/openephys')
            if self.parameters['params/Neuropixel SMA port TRIG'] or 1:
                time.sleep(2)
                if hasattr(self.machine_config, 'MC_STOP_TRIGGER'):
                    daq.set_digital_line(self.machine_config.MC_STOP_TRIGGER,1)
            if self.machine_config.ENABLE_STIM_UDP_TRIGGER:
                fusi_enable='fUSI' if  self.parameters['params/fUSI enable'] else ''
                utils.send_udp(self.machine_config.STIM_COMPUTER_IP,self.machine_config.STIM_TRIGGER_PORT,f'start,{id}_{self.experiment_name_tag},{fusi_enable}')
                self.printc('Sent trigger to Psychotoolbox')
            import psutil
            p = psutil.Process(self.camera1handler.pid)
            p.nice(psutil.HIGH_PRIORITY_CLASS)
            if self.machine_config.CAMERA2_ENABLE:
                p = psutil.Process(self.camera2handler.pid)
                p.nice(psutil.HIGH_PRIORITY_CLASS)
            self.tstart=time.time()
            self.statusbar.recording_status.setStyleSheet('background:red;')
            self.statusbar.recording_status.setText(msg)
            self.track=[]
        except:
            self.printc(traceback.format_exc())
            if hasattr(self,  'send'):
                self.send({'trigger': 'cam error'})
            
    def stop_recording(self, manual=True):
        try:
            if not self.recording:
                return
#            self.printc(('sync lenght before stop',self.sync.shape))
            t0=time.time()
            self.printc('Stop video recording, please wait...')
            self.camera1handler.stop_saving()
            if self.machine_config.CAMERA2_ENABLE:
                self.camera2handler.stop_saving()
            if self.parameters['params/Neuropixel SMA port TRIG']:
                self.printc('Stop Neuropixel')
            if self.parameters['params/Neuropixel SMA port TRIG'] or 1:
                if hasattr(self.machine_config, 'MC_STOP_TRIGGER'):
                    daq.set_digital_line(self.machine_config.MC_STOP_TRIGGER,0)
                    time.sleep(1)
            else:
                if hasattr(self.machine_config, 'MC_STOP_TRIGGER'):
                    self.printc('Stop MC recording')
                    daq.digital_pulse(self.machine_config.MC_STOP_TRIGGER,1)
                    time.sleep(0.5)
#            self.printc(f'Recording time: {time.time()-self.t0}')
#            self.printc(f'Available sync length: {self.sync.shape[0]/self.machine_config.SYNC_RECORDER_SAMPLE_RATE}')
            if self.parameters['params/fUSI enable'] and not manual:
                if not NEW_FUSI_CONTOL:
                    fusi_duration=self.parameters['params/fUSI Nimag']/self.parameters['params/fUSI sampling rate']
                else:
                    fusi_duration=0
                dt=time.time()-self.t0
                wait_left=utils.roundint(fusi_duration-dt)+self.machine_config.FUSI_RECORDING_OVERHEAD
                if wait_left>0 and not NEW_FUSI_CONTOL:#This should be disabled
                    self.printc(f'Wait {wait_left} s until fUSI finishes')
                    QtCore.QCoreApplication.instance().processEvents()
                    time.sleep(wait_left)
            else:
                wait_left=0
            self.statusbar.recording_status.setStyleSheet('background:yellow;')
            self.statusbar.recording_status.setText('Busy')
            QtCore.QCoreApplication.instance().processEvents()
            if hasattr(self.machine_config, 'MINISCOPE_DATA_PATH'): #OBSOLETE
                miniscope_datafolder=self.find_miniscope_data()
                if os.path.exists(miniscope_datafolder):
                    import copy
                    parameters=copy.deepcopy(self.parameters)
                    parameters['miniscope data']=miniscope_datafolder
                    hdf5io.save_item(hdf5_out, 'parameters', parameters)
            else:
                self.matdata={'parameters': self.parameters}
            import skvideo.io
            videometadata=skvideo.io.ffprobe(self.cam1fn)
            if 'video' in videometadata:
                fc=int(videometadata['video']['@nb_frames'])
                self.nrecorded_frames=fc
                self.printc('Recorded {0} frames'.format(fc))
            else:
                fc=None                
            if hasattr(self,  'ai'):
                self.printc('Terminate timing signal recording, please wait...')
                if self.aifix:
                    self.sync=self.ai.read().T
                else:
                    self.sync=numpy.concatenate((self.sync, self.ai.stop()))
#                self.sync=self.ai.stop()
                self.matdata['sync']=self.sync
                self.matdata['machine_config']=self.machine_config.todict()
                self.fps_values, fpsmean,  fpsstd=signal.calculate_frame_rate(self.sync[:, self.machine_config.TCAM1_SYNC_INDEX], self.machine_config.SYNC_RECORDER_SAMPLE_RATE, threshold=2.5)
                self.printc('Measured frame rate is {0:.2f} Hz, std: {1:.2f} ms, recorded {2} frames'.format(fpsmean, 1000/fpsstd,  self.fps_values.shape[0]+1))
                if self.machine_config.CAMERA2_ENABLE:
                    self.fps_values, fpsmean,  fpsstd=signal.calculate_frame_rate(self.sync[:, self.machine_config.TCAM2_SYNC_INDEX], self.machine_config.SYNC_RECORDER_SAMPLE_RATE, threshold=2.5)
                    self.printc('Camera2: Measured frame rate is {0:.2f} Hz, std: {1:.2f} ms, recorded {2} frames'.format(fpsmean, 1000/fpsstd,  self.fps_values.shape[0]+1))
                try:
                    self.check_timing_signal()
                except:
                    e=traceback.format_exc()
                    self.printc(e)
                if self.trigger_state=='stopped':#check if nvista camera was also recording
                    self.check_nvista_camera_timing()
            # else:
                # self.printc('mean: {0} Hz,  std: {1} ms'.format(1/numpy.mean(numpy.diff(self.ts)), 1000*numpy.std(numpy.diff(self.ts))))
            self.printc('Saved to {0}'.format(self.cam1fn))
#            self.printc(('sync lenght after  camera',self.sync.shape))
#            if self.camera_api=='tisgrabber':
#                self.camera1handler=camera.ISCamera(self.machine_config.CAMERA1_ID, self.logger.filename, self.camera1_io_port, self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3)
#            else:
#                self.camera1handler=camera_interface.ImagingSourceCameraHandler(self.parameters['params/Frame Rate'], self.parameters['params/Exposure time']*1e-3,  None)
#            self.camera1handler.start()
#            if self.machine_config.CAMERA2_ENABLE:
#                raise NotImplementedError()
#                self.camera2handler=camera.WebCamera(self.machine_config.EYECAM_ID,os.path.join(self.machine_config.LOG_PATH, 'log_eyecam.txt'),None,filename=None)
#                self.camera2handler.start()
            self.statusbar.recording_status.setStyleSheet('background:gray;')
            self.statusbar.recording_status.setText('Ready')
            self.recording=False
            self.printc('Save time {0} s'.format(int(time.time()-t0-wait_left)))
            if hasattr(self, 'fps_values') and self.fps_values.shape[0]<10:
                self.printc('Recording too short, file removed')
            if self.machine_config.ENABLE_OPENEPHYS_TRIGGER and self.openephys_running and self.parameters['params/Neuropixel SMA port TRIG'] :
                self.openephys_running=False
                time.sleep(self.machine_config.OPENEPHYS_PRETRIGGER_TIME)
                self.printc('Stop Openephys')
                openephys.stop_recording(self.machine_config.OPENEPHYS_COMPUTER_IP)
                if 0 and self.machine_config.OPENEPHYS_DATA_PATH is not None:
                    try:
                        zipfile=os.path.join(os.path.dirname(self.metadatafn), fileop.replace_extension(os.path.basename(self.metadatafn), '.zip'))
                        self.printc('Check recorded data,please wait...')
                        QtCore.QCoreApplication.instance().processEvents()
                        self.frequency, self.frequency_std, self.video_frame_indexes, self.sync_data=openephys.check_data(self.openephys_datafolder)
                        self.matdata['video_frame_indexes']= self.video_frame_indexes
                        self.printc('Move openephys data to {0}'.format(zipfile))
                        fileop.move2zip(self.openephys_datafolder,zipfile)
                    except:
                        self.printc(traceback.format_exc())
            import scipy.io
            self.matdata['software']=experiment_data.pack_software_environment()
            if self.machine_config.ENABLE_SYNC=='camera':
                scipy.io.savemat(self.metadatafn, self.matdata, long_field_names=True)
                self.printc(f'Metadata saved to {self.metadatafn}')
            if 0 and self.machine_config.ENABLE_OPENEPHYS_TRIGGER:
                self.printc(f"Expected frame rate: {self.parameters['params/Frame Rate']} Hz,  measured: {self.frequency} Hz, std: {self.frequency_std}")
                if self.video_frame_indexes[0]<10e3*self.machine_config.OPENEPHYS_PRETRIGGER_TIME:
                    QtGui.QMessageBox.question(None,'Warning', 'Beginning of sync signal is corrupt', QtGui.QMessageBox.Ok)
                if fc!=None and len(self.video_frame_indexes)!=fc:
                    raise ValueError(f'Recorded number of frames ({fc}) and timestamps ({len(self.video_frame_indexes)}) do not match')
            
            if self.triggered_recording:
                mcd_wait_time=2
                t0=time.time()
                time.sleep(2)
                now=time.time()
                #If mcd file is at least mcd_wait_time old but not older than beginning of recording. This ensures that the right mcd file is assigned to this recording
                if now-os.path.getmtime(self.triggered_files[-1])>mcd_wait_time and os.path.getmtime(self.triggered_files[-1])>self.start_trigger_time-mcd_wait_time:
                    self.printc(time.time()-os.path.getmtime(self.triggered_files[-1]))
                    self.printc(self.triggered_files[-1])
                    dst=fileop.replace_extension(self.metadatafn, '.mcd')
                    import shutil
                    shutil.copy(self.triggered_files[-1], dst)
                    self.printc(f'Copied {self.triggered_files[-1]} to {dst}')
            
            self.triggered_recording=False
            #TMP:
            if self.machine_config.ENABLE_SYNC=='camera' :
                self.plotw(self.sync, self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
        except:
            e=traceback.format_exc()
            self.printc(e)
            if hasattr(self,  'send'):
                self.send({'trigger': 'cam error'})
            
    def check_timing_signal(self):
        if self.sync.shape[0]==0:
            msg='Sync recording failed'
            self.printc(msg)
            QtGui.QMessageBox.question(self, 'Warning', msg, QtGui.QMessageBox.Ok)
            return
        self.camera1_timestamps=signal.trigger_indexes(self.sync[:,self.machine_config.TCAM1_SYNC_INDEX])[::2]/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
        if hasattr(self, 'matdata'):
            self.matdata['camera1_timestamps']=self.camera1_timestamps
        self.sync_length=self.sync.shape[0]/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
        two_frame_time=self.parameters['params/Exposure time']*1e-3*2
        msg=''
        if self.nrecorded_frames!=self.camera1_timestamps.shape[0]:
            msg+f'Number of recorded video frames and camera timing pulses do not match, number of pulses {self.camera1_timestamps.shape[0]} '
        if self.camera1_timestamps[0]<two_frame_time or self.camera1_timestamps[-1]>self.sync_length-two_frame_time:
            msg+='Beginning or end of camera timing signal may not be recorder properly! '
        if hasattr(self.machine_config, 'MC_STOP_TRIGGER') and self.triggered_recording:
            #Check if MC stop trigger took place
            self.mc_timestamps=signal.trigger_indexes(self.sync[:,self.machine_config.TMCSTOP_SYNC_INDEX])/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
            if self.mc_timestamps.shape[0]!=1 and self.mc_timestamps[0]<self.sync_length*0.5:
                msg+='Multi Channel Stop signal is corrupt!'
            if hasattr(self, 'matdata'):
                self.matdata['mc_timestamps']=self.mc_timestamps
        if self.machine_config.ENABLE_STIM_UDP_TRIGGER:
            #Check stim timestamps
            self.stim_timestamps=signal.trigger_indexes(self.sync[:,self.machine_config.TSTIM_SYNC_INDEX])/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
            if self.stim_timestamps.shape[0]<=1:
                msg+='Stimulus timing signal was not generated! '
            if self.stim_timestamps.shape[0]<1:
                msg+='Beginning of stimulus was not recorded! '
            if self.sync_length-self.stim_timestamps[-1]<1:
                msg+='End of stimulus was not recorded! '
            if hasattr(self, 'matdata'):
                self.matdata['stim_timestamps']=self.stim_timestamps
        if self.parameters['params/fUSI enable']:
            self.fusi_timestamps=signal.trigger_indexes(self.sync[:,self.machine_config.TFUSI_SYNC_INDEX])/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
            self.matdata['fusi_timestamps']=self.fusi_timestamps
            if self.fusi_timestamps.shape[0]<2:
                msg+='End of fUSI timing is not recorded! '
                numpy.diff(self.fusi_timestamps)
            #Emulate fUSI pulses
            if not NEW_FUSI_CONTOL:
                fusi_overhead=4.8*0
                duration=numpy.diff(self.fusi_timestamps)[0]-fusi_overhead#Measured ~4.8 second
                if duration<0:
                    duration=0
                emulated_pulses=numpy.zeros(int(duration*self.machine_config.SYNC_RECORDER_SAMPLE_RATE))
                period=int(duration/self.parameters['params/fUSI Nimag']*self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
                if period>1:
                    emulated_pulses[::period]=3.3
                o=int(self.fusi_timestamps[0]*self.machine_config.SYNC_RECORDER_SAMPLE_RATE+fusi_overhead)
                self.sync[o:o+emulated_pulses.shape[0], self.machine_config.TFUSI_SYNC_INDEX]+=emulated_pulses
                self.matdata['sync']=self.sync
                self.matdata['emulated_fusi_timestamps']=signal.trigger_indexes(self.sync[:,self.machine_config.TFUSI_SYNC_INDEX])/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
                self.matdata['emulated_pulses']=emulated_pulses
                self.matdata['emulated_offset']=o
        if len(msg)>0:
            self.printc(msg)
            QtGui.QMessageBox.question(self, 'Warning', msg, QtGui.QMessageBox.Ok)
        else:
            self.printc('All timing signals OK')
            
    def check_nvista_camera_timing(self):
        timestamps=signal.trigger_indexes(self.sync[:,self.machine_config.TNVISTA_SYNC_INDEX])/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
        fps=1/numpy.diff(timestamps[::2])
        self.printc('nVista camera frame rate: {0:.1f} Hz, std: {1:.1f} Hz'.format(fps.mean(), fps.std()))
        if fps.mean()>60 or fps.mean()<4:
            self.printc('Invalid nVIsta camera frame rate: {0}'.format(fps.mean()))
            
    def check_video(self):#OBSOLETE
        import hdf5io
        self.fframes=hdf5io.read_item(self.fn,  'frames')
        self.ct=[]
        for f in self.fframes:
            ct=f[0, 0, 0]*256+f[0, 1, 0]
            self.ct.append(ct)
        if (numpy.diff(self.ct)!=1).any():
            raise RuntimeError(numpy.diff(self.ct))
            
        
    def parameter_changed(self):
        newparams=self.params.get_parameter_tree(return_dict=True)
        if hasattr(self,  'parameters') and (newparams['params/Frame Rate']!=self.parameters['params/Frame Rate'] or newparams['params/Exposure time']!=self.parameters['params/Exposure time']):
            self.restart_camera()
        self.parameters=newparams
        
    def record_action(self):
        self.start_recording()
        
    def stop_action(self):
        self.stop_recording()
            
    def convert_folder_action(self):
        #This is handled by main GUI process, delegating it to gui engine would make progress bar handling more complicated        
        try:
            if self.recording: return
            foldername = str(QtGui.QFileDialog.getExistingDirectory(self, 'Select hdf5 video file folder', self.filebrowserroot))
            if foldername=='': return
            if os.name=='nt':
                foldername=foldername.replace('/','\\')
            files=fileop.listdir(foldername)
            p=gui.Progressbar(100, 'Conversion progress',  autoclose=True)
            p.show()
            self.printc('Conversion started')
            self.statusbar.recording_status.setStyleSheet('background:yellow;')
            self.statusbar.recording_status.setText('Processing')
            QtCore.QCoreApplication.instance().processEvents()
            dst=os.path.join(self.machine_config.BACKUP_PATH, os.path.basename(foldername))
            if not os.path.exists(dst):
                os.mkdir(dst)
            time.sleep(0.5)
            for f in files:
                if self.machine_config.ENABLE_OPENEPHYS_TRIGGER:
                    condition=not os.path.isdir(f) and os.path.splitext(f)[1]=='.mp4'
                else:
                    condition=not os.path.isdir(f) and os.path.splitext(f)[1]=='.hdf5' and not os.path.exists(fileop.replace_extension(experiment_data.add_mat_tag(f), '.mat'))
                if condition:
                    if os.path.exists(fileop.replace_extension(f, '.mp4')):
                        fconvert=fileop.replace_extension(f, '.mp4')
                    else:
                        fconvert=f
                    print(f)
                    try:
                        self.convert_file(fconvert)
                        if self.machine_config.ENABLE_VIDEO_RESAMPLING:
                            print('Resampling video')
                            vfn1, vfn2=behavioral_data.align_videos_wrapper(f)
                            import shutil
                            shutil.copy(vfn1,dst)
                            shutil.copy(vfn2,dst)
                        if not self.machine_config.ENABLE_OPENEPHYS_TRIGGER:
                            #Copy files
                            import shutil
                            shutil.copy(f,dst)
                            shutil.copy(fileop.replace_extension(experiment_data.add_mat_tag(f), '.mat'),dst)
                            self.printc('Copied to {0}'.format(dst))
                    except:
                        self.printc(traceback.format_exc(), popup_error=False)
                    prog=int((files.index(f)+1)/float(len(files))*100)
                    p.update(prog)
                    QtCore.QCoreApplication.instance().processEvents()
                    time.sleep(100e-3)
            self.printc('{0} folder complete'.format(foldername))
        except:
            self.printc(traceback.format_exc())
        self.statusbar.recording_status.setStyleSheet('background:gray;')
        self.statusbar.recording_status.setText('Ready')
    
    
    def convert_file(self, filename):
        if os.path.splitext(filename)[1]=='.hdf5':
            h=hdf5io.Hdf5io(filename)
            h.load('machine_config')
            h.load('parameters')
            h.load('sync')
            #Make parameters and machine config compatible with matlab's hdf5io
            p={}
            for k, v in h.parameters.items():
                if isinstance(v, bool):
                    v=int(v)
                p[k.replace(' ', '_').lower()]=v
            for k, v in h.machine_config.items():
                if isinstance(v, bool):
                    h.machine_config[k]=int(v)
            h.parameters=p
            h.head_direction=[]
            h.led_positions=[]
            h.head_position=[]
            ct=0
            h.frame_indexes=[]
            h.tnvista=signal.trigger_indexes(h.sync[:,self.machine_config.TNVISTA_SYNC_INDEX])[::2]/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
            h.tbehav=signal.trigger_indexes(h.sync[:,self.machine_config.TBEHAV_SYNC_INDEX])[::2]/float(self.machine_config.SYNC_RECORDER_SAMPLE_RATE)
            nframes=h.h5f.root.frames.shape[0]
            chunksize=1000
            nchunks=nframes/chunksize
            if nchunks==0:
                nchunks=1
            p=gui.Progressbar(100, 'Processing status',  autoclose=True)
            p.move(200, 100)
            p.show()
            for chunki in range(nchunks):
                frames=h.h5f.root.frames.read(chunki*chunksize,  (chunki+1)*chunksize)
                self.printc((frames[0, 0, :2, 0]*numpy.array([256,1])).sum())#Just to see if frames are loaded in the right order
                prog=100*chunki/nchunks
                p.update(prog)
                QtCore.QCoreApplication.instance().processEvents()
                for f in frames:
                    try:
                        result, position, self.red_angle, red, green, blue, debug=behavioral_data.mouse_head_direction(f, roi_size=self.parameters['params/ROI size'], threshold=self.parameters['params/Threshold'],  saturation_threshold=0.6, value_threshold=0.4)
                        print((result, position, self.red_angle, red, green, blue))
                        if result:
                            h.head_direction.append(self.red_angle)
                            h.led_positions.append([red, green, blue])
                            h.head_position.append(position)
                            h.frame_indexes.append(ct)
                        ct+=1
                    except:
                        self.printc(traceback.format_exc(), popup_error=False)
                        ct+=1
                        continue
            h.save(['head_direction',  'led_positions',  'head_position',  'frame_indexes',  'tnvista', 'tbehav', 'machine_config',  'parameters'])
            h.close()
            experiment_data.hdf52mat(filename,  scale_sync=True, exclude=['frames'])
        elif os.path.splitext(filename)[1]=='.mp4':
            import skvideo.io
            videometadata=skvideo.io.ffprobe(filename)
            fc=int(videometadata['video']['@nb_frames'])
            videogen = skvideo.io.vreader(filename)
            h=hdf5io.Hdf5io(fileop.replace_extension(filename, '.hdf5'))
            h.head_direction=[]
            h.led_positions=[]
            h.head_position=[]
            ct=0
            h.frame_indexes=[]
            p=gui.Progressbar(100, f'{os.path.basename(filename)} processing status',  autoclose=True)
            p.move(200, 100)
            p.show()
            for f in videogen:
                try:
                    result, position, self.red_angle, red, green, blue, debug=behavioral_data.mouse_head_direction(numpy.copy(f), roi_size=self.parameters['params/ROI size'], threshold=self.parameters['params/Threshold'],  saturation_threshold=0.6, value_threshold=0.4)
                    print((result, position, self.red_angle, red, green, blue))
                    if result:
                        h.head_direction.append(self.red_angle)
                        h.led_positions.append([red, green, blue])
                        h.head_position.append(position)
                        h.frame_indexes.append(ct)
                    ct+=1
                except:
                    self.printc(traceback.format_exc(), popup_error=False)
                    ct+=1
                    continue
                prog=100*ct/fc
                p.update(prog)
                QtCore.QCoreApplication.instance().processEvents()
#            h.save(['head_direction',  'led_positions',  'head_position',  'frame_indexes' ])
            for vn in ['head_direction',  'led_positions',  'head_position',  'frame_indexes' ]:
                try:
                    print(vn)
                    print(getattr(h, vn))
                    if len(getattr(h, vn))>0:
                        h.save(vn)
                except:
                    pass

            h.close()
            experiment_data.hdf52mat(fileop.replace_extension(filename, '.hdf5'))
    
    def update_image(self):
        try:
            if hasattr(self.camera1handler, 'log') and not self.camera1handler.log.empty():
                self.printc(self.camera1handler.log.get())
            if self.machine_config.CAMERA2_ENABLE and hasattr(self.camera2handler, 'log') and not self.camera2handler.log.empty():
                self.printc(self.camera2handler.log.get())
            if hasattr(self, 'ai') and not self.aifix:
                s=self.sync.shape[0]
                self.sync=numpy.concatenate((self.sync, self.ai.read()))
#                if s!=self.sync.shape[0]:
#                    self.printc(self.sync.shape[0])
            if hasattr(self.camera1handler, 'display_frame'):
                imgqueue=self.camera1handler.display_frame
            elif hasattr(self.camera1handler, 'queues'):
                imgqueue=self.camera1handler.queues['data']
            if not imgqueue.empty() and self.parameters['params/Enable live view']:
                frame=imgqueue.get()
                self.frame=frame
                if self.parameters['params/Enable ROI cut']:
                    frame=frame[self.parameters['ROI x1']:self.parameters['ROI x2'],self.parameters['ROI y1']:self.parameters['ROI y2']]
                f=numpy.copy(frame)
                self.f=f
                if hasattr(self.machine_config,'TRACK_ANIMAL') and self.machine_config.TRACK_ANIMAL:
                    if self.recording or self.parameters.get('params/Show color LEDs', False):
                        try:
                            result, self.position, self.red_angle, self.red, self.green, self.blue, debug=behavioral_data.mouse_head_direction(f, roi_size=self.parameters['params/ROI size'], threshold=self.parameters['params/Threshold'],  saturation_threshold=0.6, value_threshold=0.4)
                        except:
                            self.printc('Tracking problem')
                            numpy.save('c:\\Data\\log\\{0}.npy'.format(time.time()),  f)
                            self.logger.info(traceback.format_exc())
                            
                        if self.recording:
                            self.track.append(self.position)
                        if self.parameters.get('params/Show color LEDs', False):
                            f[int(self.red[0]), int(self.red[1])]=numpy.array([255, 255,0],dtype=f.dtype)
                            f[int(self.green[0]), int(self.green[1])]=numpy.array([255,255,0],dtype=f.dtype)
                            f[int(self.blue[0]), int(self.blue[1])]=numpy.array([255,255, 0],dtype=f.dtype)
                            #f[int(self.position[0]), int(self.position[1])]=numpy.array([255,255, 255],dtype=f.dtype)
                    if self.parameters.get('params/Show track', False):
                        for p in self.track:
                            try:
                                if numpy.isnan(p[0]):
                                    continue  
                                f[int(p[0]), int(p[1])]=numpy.array([255,255,255],dtype=f.dtype)
                            except:
                                self.printc('Track display problem')
                                numpy.save('c:\\Data\\log\\{0}.npy'.format(time.time()),  numpy.array(p))
                                self.logger.info(traceback.format_exc())
                    
                try:
                    self.frame1=numpy.rot90(numpy.flipud(f))
                    self.image.set_image(self.frame1,color_channel='all')
                except:
                    print(traceback.format_exc())
                if self.recording and hasattr(self,  'tstart'):
                    dt=time.time()-self.tstart
                    title='{0} s'.format(int(dt))
                    if hasattr(self,  'red_angle'):
                        title+='/head direction: {0:0.1f}'.format(self.red_angle)
                    self.image.plot.setTitle(title)
                
                self.trigger_handler()
                self.socket_handler()
            if self.machine_config.CAMERA2_ENABLE:
                frame=self.camera2handler.read()
                if frame is not None:
                    self.frame2=numpy.rot90(numpy.flipud(frame))
                    self.camera2image.set_image(self.frame2,color_channel='all')
        except:
            self.printc(traceback.format_exc())
        
    def exit_action(self):
        self.save_context()
        self.camera1handler.stop()
        if self.machine_config.CAMERA2_ENABLE:
            self.camera2handler.stop()
#        if hasattr(self,  'ioboard'):
#            self.ioboard.close()
#        if hasattr(self, 'ai'):
#            self.ai.queues['command'].put('terminate')
#            self.ai.join()
        self.close()
        
    def socket_handler(self):
        if not self.socket_queues['cam']['fromsocket'].empty():
            command=self.socket_queues['cam']['fromsocket'].get()
            try:
                if 'function' in command:
                    getattr(self,  command['function'])(*command['args'])
            except:
                self.socket_queues['cam']['tosocket'].put({'trigger': 'cam error'})
        if self.machine_config.ENABLE_STIM_UDP_TRIGGER:
            #calculate the recording timeout from fusi frame rate and number of images
            if hasattr(self, 'tstart') and self.recording:
                dt=time.time()-self.tstart
                if not NEW_FUSI_CONTOL:
                    fusitimeout=240+self.parameters['params/fUSI Nimag']/self.parameters['params/fUSI sampling rate']
                    to=fusitimeout if self.parameters['params/fUSI enable'] else self.parameters['params/Recording timeout']
                else:
                    to=self.parameters['params/Recording timeout']
                if dt>to:
                    self.printc(f'Stimulus timeout: {to} seconds')
                    self.stop_recording(manual=False)
            res=utils.recv_udp(self.machine_config.CAM_COMPUTER_IP, self.machine_config.STIM_TRIGGER_PORT, 0.1)
            if len(res)>0:
                self.printc(f'UDP message received: {res}')
                if 'stop' in res:
                    self.stop_recording(manual=False)
        
    def trigger_handler(self):
        now=time.time()
        if self.trigger_state=='off':
            if self.parameters['params/Enable trigger']:
                #Enable trigger
                if self.parameters['params/Trigger']=='TTL pulses':
                    raise NotImplementedError()
                elif self.parameters['params/Trigger']=='network':
                    raise NotImplementedError()
                elif self.parameters['params/Trigger']=='file':
                    self.latest_mcd_file=fileop.find_latest(self.machine_config.EXPERIMENT_DATA_PATH,'.mcd')
                    self.triggered_files=[]
                self.trigger_state='waiting'
        elif self.trigger_state=='waiting':
            if not self.parameters['params/Enable trigger']:
                self.trigger_state='off'
            if self.parameters['params/Trigger']=='file':
                new_latest_mcd_file=fileop.find_latest(self.machine_config.EXPERIMENT_DATA_PATH,extension='.mcd', subfolders=False)
#                print(new_latest_mcd_file, self.latest_mcd_file ,self.triggered_files  , now-os.path.getmtime(new_latest_mcd_file))
#                print(new_latest_mcd_file!=self.latest_mcd_file , new_latest_mcd_file not in self.triggered_files , now-os.path.getmtime(new_latest_mcd_file)>self.machine_config.FILE_TIMEOUT)
                if new_latest_mcd_file!=self.latest_mcd_file and new_latest_mcd_file not in self.triggered_files and now-os.path.getmtime(new_latest_mcd_file)<self.machine_config.FILE_TIMEOUT:
                    self.triggered_files.append(self.latest_mcd_file)
                    self.latest_mcd_file=new_latest_mcd_file
                    self.triggered_files.append(new_latest_mcd_file)
                    self.trigger_state='started'
                    self.printc(f'File trigger received: {new_latest_mcd_file}')
        elif self.trigger_state=='started':
            if not self.parameters['params/Enable trigger']:
                self.trigger_state='off'
            if self.parameters['params/Trigger']=='file':
                self.start_recording()
                self.start_trigger_time=time.time()
                self.triggered_recording=True
                self.trigger_state='waiting'
        if self.trigger_state=='off':
            color='grey'
        elif self.trigger_state=='waiting':
            color='yellow'
        elif self.trigger_state=='started':
            color='red'
        elif self.trigger_state=='stopped':
            color='orange'
        self.statusbar.trigger_status.setStyleSheet('background:{0};'.format(color))
        self.statusbar.trigger_status.setText('trigger status: {0}'.format(self.trigger_state))
    
    def enable_trigger(self):#OBSOLETE
        if not self.trigger_detector_enabled:
            self.trigger_detector=digital_io.TriggerDetector(self.machine_config.TRIGGER_DETECTOR_PORT,self.machine_config.TRIGGER_TIMEOUT)
#            self.ioboard.write('wait_trigger,1\r\n')
#            self.printc(self.ioboard.read(100))
            self.trigger_detector_enabled=True
        
    def disable_trigger(self):#OBSOLETE
        if self.trigger_detector_enabled:
#            self.ioboard.write('wait_trigger,0\r\n')
#            self.printc(self.ioboard.read(100))    
            self.trigger_detector.close()
            self.trigger_detector_enabled=False

    def find_miniscope_data(self):
        time_struct = time.localtime(time.time())
        d='{2:0=2}_{1:0=2}_{0:0=4}'.format(time_struct.tm_year, time_struct.tm_mon, time_struct.tm_mday)
        folder=os.path.join(self.machine_config.MINISCOPE_DATA_PATH, d)
        #Latest folder:
        fns = [fn for fn in fileop.listdir_fullpath(folder) if os.path.isdir(fn)]
        if len(fns) == 0:
            return
        fns_dates = map(os.path.getmtime, fns)
        latest_file_index = fns_dates.index(max(fns_dates))
        return fns[latest_file_index]
        
