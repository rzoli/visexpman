from visexpman.engine.generic import gui
import os,unittest,multiprocessing,time,pdb,numpy, cv2, ctypes, queue
try:
    from thorlabs_tsi_sdk.tl_camera import TLCameraSDK
except:
    print('No thorlabs camera driver installed')

try:
    import tisgrabber
except:
    print('No Imaging source camera driver installed')
try:
    import PyDAQmx
    import PyDAQmx.DAQmxConstants as DAQmxConstants
    import PyDAQmx.DAQmxTypes as DAQmxTypes
except ImportError:
    print('PyDAQmx not installed')
from visexpman.engine.hardware_interface import instrument
from visexpman.engine.hardware_interface import daq
from visexpman import fileop
import copy
import time
import numpy

try:
    import PySpin
except:
    print('No BlackFly S camera driver installed')

def start_widefield(config):#Chuchlab WidefieldImager compatible
    if not hasattr(config,'WIDEFIELD_START_PINS'):
        return
    for port in config.WIDEFIELD_START_PINS:
        daq.digital_pulse(port,100e-3)
        time.sleep(0.3)
        
def stop_widefield(config):
    if not hasattr(config,'WIDEFIELD_STOP_PINS'):
        return
    for port in config.WIDEFIELD_STOP_PINS:
        daq.digital_pulse(port,100e-3)
        time.sleep(0.3)

class ThorlabsCamera(object):
    def __init__(self, dll_path, nbit=8):
        os.environ['PATH'] = dll_path + os.pathsep + os.environ['PATH']
        self.sdk=TLCameraSDK()
        self.camera=self.sdk.open_camera(self.sdk.discover_available_cameras()[0])
        self.nbit=nbit
        self.roi_orig=copy.deepcopy(self.camera.roi)
        self.exposure_time_us_orig=copy.deepcopy(self.camera.exposure_time_us)
        self.gain_orig=copy.deepcopy(self.camera.gain)
        
    def set(self, exposure=None, roi=None, gain=None):
        if exposure !=None:
            self.camera.exposure_time_us = exposure
        if roi != None:
            self.camera.roi = roi
        if gain != None:
            self.camera.gain=gain
            
    def start_(self):
        self.camera.arm(2)
        self.camera.issue_software_trigger()
        
    def get_frame(self):
        frame = self.camera.get_pending_frame_or_null()
        if frame == None:
            return None
        if self.nbit==8:
            frame8bit=frame.image_buffer >> (self.camera.bit_depth-8)
            return frame8bit
        elif self.nbit==16:
            return frame.image_buffer
        
    def stop(self):
        self.camera.disarm()
        
    def close(self):
        self.camera.roi=self.roi_orig
        self.camera.exposure_time_us=self.exposure_time_us_orig
        self.camera.gain=self.gain_orig
#        self.camera.roi=(0, 0, 4096, 2160)
        self.camera.dispose()
        self.sdk.dispose()
        print ("sdk dispose")
        
class ThorlabsCameraProcess(ThorlabsCamera, instrument.InstrumentProcess):
    def __init__(self,dll_path,logfile,roi=None):
        self.roi=roi
        self.dll_path=dll_path
        self.queues={'command': multiprocessing.Queue(), 'response': multiprocessing.Queue(), 'data': multiprocessing.Queue()}
        instrument.InstrumentProcess.__init__(self,self.queues,logfile)
        self.control=multiprocessing.Queue()
        self.data=multiprocessing.Queue()
        self.downsample=13
        self.fov_roi=roi
        
    def start_(self):
        self.queues['command'].put('start')
        
    def stop(self):
        self.queues['command'].put('stop')
        
    def set(self, **kwargs):
        self.queues['command'].put(('set',list(kwargs.keys())[0],list(kwargs.values())[0]))
        
    def set_fov(self, ds, roi):
        self.queues['command'].put((['downsample',ds, roi]))
        
    def read(self):
        if not self.queues['data'].empty():
            return self.queues['data'].get()
        
    def run(self):
        self.setup_logger()
        self.printl(f'pid: {os.getpid()}')
        self.running=False
        ThorlabsCamera.__init__(self, dll_path=self.dll_path,nbit=16)
        if self.roi is not None:
            self.printl(self.roi)
            ThorlabsCamera.set(self,roi=self.roi)
        while True:
            try:
                if not self.queues['command'].empty():
                    cmd=self.queues['command'].get()
                    self.printl(cmd)
                    if cmd=='start':
                        ThorlabsCamera.start_(self)
                        self.running=True
                    elif cmd=='stop':
                        ThorlabsCamera.stop(self)
                        self.running=False
                    elif cmd=='terminate':
                        if self.running:
                            ThorlabsCamera.stop(self)
                        self.close()
                        break
                    elif cmd[0]=='set':
                        kwarg={cmd[1]:cmd[2]}
                        ThorlabsCamera.set(self,**kwarg)
                        self.printl(f'gain: {self.camera.gain}, exp: {self.camera.exposure_time_us}')
                    elif cmd[0]=='downsample':
                        self.downsample=cmd[1]
                        self.fov_roi=cmd[2]
                        self.printl(f'Downsample: {self.downsample}, fov: {self.fov_roi}')
                if self.running:
                    frame=self.get_frame()
                    if self.queues['data'].empty() and frame is not None:#Send frame when queue empty (previous frame was taken
                        if frame.shape[0]>504 or 1:
                            frame=frame[self.fov_roi[0]:self.fov_roi[2], self.fov_roi[1]:self.fov_roi[3]][::self.downsample, ::self.downsample]
                        self.queues['data'].put(frame)
                time.sleep(50e-3)
            except:
                import traceback
                self.printl(traceback.format_exc())
    
    
class CameraCallbackUserdata(ctypes.Structure):
    def __init__(self):
        self.camera = None # Reference to the camera object
        self.frame_queue = None
  
class ISCamera(instrument.InstrumentProcess):
    def __init__(self,camera_id,logfile,digital_line, frame_rate=60, exposure=1/65, filename=None, show=False):
        self.filename=filename
        self.camera_id=camera_id
        self.frame_rate=frame_rate
        self.digital_line=digital_line
        self.exposure=exposure
        self.show=show
        self.queues={'command': multiprocessing.Queue(), 'response': multiprocessing.Queue(), 'data': multiprocessing.Queue()}
        instrument.InstrumentProcess.__init__(self,self.queues,logfile)
        self.control=multiprocessing.Queue()
        self.data=multiprocessing.Queue()
        print(self.logfile)
        self.Userdata = CameraCallbackUserdata()
        
    def stop(self):
        self.queues['command'].put('stop')
        
    def read(self):
        if not self.queues['data'].empty():
            return self.queues['data'].get()
            
    def save(self, fn):
        self.queues['command'].put(['save', fn])

    def stop_saving(self):
        self.queues['command'].put('stop_saving')
        t0=time.time()
        while True:
            if not self.queues['response'].empty():
                msg=self.queues['response'].get()
                if msg=='save done':
                    break
                if time.time()-t0>60:
                    break
                time.sleep(0.5)
                    
                
    def FrameReadyCallback(self, hGrabber, pBuffer, framenumber, pData):
        BildDaten = pData.camera.GetImageDescription()[:4]
        lWidth=BildDaten[0]
        lHeight= BildDaten[1]
        iBitsPerPixel=BildDaten[2]//8

        buffer_size = lWidth*lHeight*iBitsPerPixel*ctypes.sizeof(ctypes.c_uint8)            
                
        Bild = ctypes.cast(pBuffer, ctypes.POINTER(ctypes.c_ubyte * buffer_size))
        frame = numpy.ndarray(buffer = Bild.contents, dtype = numpy.uint8, shape = (lHeight, lWidth, iBitsPerPixel))
         
        frame = frame[:,:,0]
        pData.frame_queue.put(frame)
            
    def run(self):
        try:
            self.setup_logger()
            self.printl(f'pid: {os.getpid()}')
            self.running=False
            Camera = tisgrabber.TIS_CAM()
            camera_name=[camidi for camidi in Camera.GetDevices() if camidi.decode()==self.camera_id]
            if len(camera_name)==0:
                raise ValueError(f'Unkown camera: {camera_name}, {Camera.GetDevices() }')

            Camera.open(camera_name[0].decode())
            Camera.SetPropertySwitch("Strobe","Enable",0)
            Camera.SetPropertySwitch("Strobe","Polarity",1)
            Camera.SetPropertyMapString("Strobe","Mode","exposure")  # exposure,constant or fixed duration
            Camera.SetPropertyValue("Strobe","Delay",0)
            
            
            Camera.SetPropertySwitch("Trigger","Enable",0)
            Camera.SetVideoFormat("Y800 (320x240)")
        
            Camera.SetFrameRate(self.frame_rate)
            Camera.SetPropertyAbsoluteValue("Exposure","Value", self.exposure)   #50fps 640x480
            Camera.SetContinuousMode(0)
            
            if self.digital_line is not None:
                digital_output = PyDAQmx.Task()
                digital_output.CreateDOChan(self.digital_line,'do', DAQmxConstants.DAQmx_Val_ChanPerLine)
            fps='30'
            import skvideo.io
            if self.filename!=None:
                self.video_writer=skvideo.io.FFmpegWriter(self.filename, inputdict={'-r':fps}, outputdict={'-r':fps})
            
            frame_queue = queue.Queue()
            
            self.Userdata.camera = Camera
            self.Userdata.frame_queue = frame_queue
            Callbackfunc = tisgrabber.TIS_GrabberDLL.FRAMEREADYCALLBACK(self.FrameReadyCallback)
            Camera.SetFrameReadyCallback(Callbackfunc, self.Userdata)
            
            if self.show:
                Camera.StartLive(1)
            else:
                Camera.StartLive(0)
                
            Camera.SetPropertySwitch("Strobe","Enable",1)
            
            while True:
                if frame_queue.empty():
                    time.sleep(1e-4)
                    continue
                frame = frame_queue.get(timeout = 0.1)
                
                if hasattr(self, 'video_writer') and self.digital_line is not None:
                    digital_output.WriteDigitalLines(1,True,1.0,DAQmxConstants.DAQmx_Val_GroupByChannel,numpy.array([1], dtype=numpy.uint8),None,None)
                if hasattr(self, 'video_writer'):
                    if len(frame.shape)==2:
                        frame=numpy.rollaxis(numpy.array([frame]*3),0,3).copy()
                    self.video_writer.writeFrame(frame)
                #Digital pulse indicates video save time
                if hasattr(self, 'video_writer') and self.digital_line is not None:
                    digital_output.WriteDigitalLines(1,True,1.0,DAQmxConstants.DAQmx_Val_GroupByChannel,numpy.array([0], dtype=numpy.uint8),None,None)
                if not self.queues['command'].empty():
                    cmd=self.queues['command'].get()
                    self.printl(cmd)
                    if cmd=='stop':
                        Camera.SetPropertySwitch("Strobe","Enable",0)
                        Camera.StopLive()
                        self.printl('Stop camera')
                        break
                    elif cmd[0]=='save':
                        self.video_writer=skvideo.io.FFmpegWriter(cmd[1], inputdict={'-r':fps}, outputdict={'-r':fps})
                    elif cmd=='stop_saving':
                        self.printl('Close video file')
                        self.video_writer.close()
                        del self.video_writer
                        self.queues['response'].put('save done')
                if self.queues['data'].empty() and frame is not None:#Send frame when queue empty (previous frame was taken
                    self.queues['data'].put(frame)
                    
            if hasattr(self, 'video_writer'):
                self.printl('Close video file')
                self.video_writer.close()
            if self.digital_line is not None:
                digital_output.ClearTask()
            self.printl('Leaving process')
        except:
            import traceback
            self.printl(traceback.format_exc())
            
class BlackFlySCamera(instrument.InstrumentProcess):
    def __init__(self,logfile,digital_line, frame_rate=30, auto_exposure_time = True, exposure_time_us=20000, filename=None, show=False, FrameStart_out_enabled = False, h_res=1440//4, v_res=1080//4):
        self.filename=filename
        self.frame_rate=frame_rate
        self.digital_line=digital_line
        self.exposure_time_us=exposure_time_us
        self.auto_exposure_time = auto_exposure_time
        self.FrameStart_out_enabled = FrameStart_out_enabled
        self.show=show
        self.h_res = h_res
        self.v_res = v_res
        self.queues={'command': multiprocessing.Queue(), 'response': multiprocessing.Queue(), 'data': multiprocessing.Queue()}
        instrument.InstrumentProcess.__init__(self,self.queues,logfile)
        self.control=multiprocessing.Queue()
        self.data=multiprocessing.Queue()
        print(self.logfile)
        print(1)
        
    def stop(self):
        self.queues['command'].put('stop')
        
    def read(self):
        if not self.queues['data'].empty():
            return self.queues['data'].get()
            
    def save(self, fn):
        self.queues['command'].put(['save', fn])

    def stop_saving(self):
        self.queues['command'].put('stop_saving')
        t0=time.time()
        while True:
            if not self.queues['response'].empty():
                msg=self.queues['response'].get()
                if msg=='save done':
                    break
                if time.time()-t0>60:
                    break
                time.sleep(0.5)

    def ttl_enable(self):
        self.queues['command'].put('ttl_enable')

    def ttl_disable(self):
        self.queues['command'].put('ttl_disable')
                
    def change_bufferhandling_mode(self, cam):
        sNodemap = cam.GetTLStreamNodeMap()
        
        # Change bufferhandling mode to NewestOnly
        node_bufferhandling_mode = PySpin.CEnumerationPtr(sNodemap.GetNode('StreamBufferHandlingMode'))
        if not PySpin.IsReadable(node_bufferhandling_mode) or not PySpin.IsWritable(node_bufferhandling_mode):
            raise ValueError(f'Unable to set stream buffer handling mode')
            #print('Unable to set stream buffer handling mode.. Aborting...')

        # Retrieve entry node from enumeration node
        node_newestonly = node_bufferhandling_mode.GetEntryByName('NewestOnly')
        if not PySpin.IsReadable(node_newestonly):
            raise ValueError(f'Unable to set stream buffer handling mode')

        # Retrieve integer value from entry node
        node_newestonly_mode = node_newestonly.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_bufferhandling_mode.SetIntValue(node_newestonly_mode)
        
    def set_acquisition_mode(self, cam):
        nodemap = cam.GetNodeMap()
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        if not PySpin.IsReadable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            raise ValueError(f'Unable to set acquisition mode to continuous (enum retrieval).')

        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        if not PySpin.IsReadable(node_acquisition_mode_continuous):
            raise ValueError(f'Unable to set acquisition mode to continuous (enum retrieval).')

        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print('Acquisition mode set to continuous...')

    def set_exposure_Auto_mode(self, cam, exposure_mode): #valid modes: Off, Continuous, Once
        cam.ExposureMode.SetValue(PySpin.ExposureMode_Timed)
        if(exposure_mode == "Off"):
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        elif(exposure_mode == "Once"):
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Once)
        elif(exposure_mode == "Continuous"):
            cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Continuous)
        else:
            print("invalid exposure_auto_mode!")
            return
        
        print('Exposure_mode_auto set to:', exposure_mode)

    def set_exposure_time_us(self, cam, exposure_time): #4-29999999us before set call set_exposure_mode with exposure_mode=Off)
        cam.ExposureTime.SetValue(exposure_time)
        print('Exposure_time set to:', exposure_time)

    def set_framerate(self, cam, fps):
        cam.AcquisitionFrameRateEnable.SetValue(True)
        cam.AcquisitionFrameRate.SetValue(fps)
        print('Set faremerate to:', fps)

    def disable_manual_trigger(self, cam):
        cam.TriggerMode.SetValue(PySpin.TriggerMode_On) #required to turn off then on to make it work. Why?
        cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

    def set_frame_start_gpio_state(self, cam, state):
        cam.LineSelector.SetValue(PySpin.LineSelector_Line2)
        if(state):
            cam.LineMode.SetValue(PySpin.LineMode_Output)
            cam.LineSource.SetValue(PySpin.LineSource_ExposureActive)  #may not be the right signal.
        else:
            cam.LineMode.SetValue(PySpin.LineMode_Input)
            cam.LineSource.SetValue(PySpin.LineSource_Off)

        print("set_frame_start_gpio_state", state)

    def set_resolution(self, cam, h_res, v_res):
        MAX_HRES = 1440
        MAX_VRES = 1080
        if(h_res <= MAX_HRES/2 and v_res <= MAX_VRES/2): #binning possible
            cam.BinningHorizontal.SetValue(2)
            cam.BinningVertical.SetValue(2)
            cam.Width.SetValue(h_res)
            cam.Height.SetValue(v_res)
            #cam.OffsetX.SetValue(((MAX_HRES/2)-h_res)/2)
           # cam.OffsetY.SetValue(((MAX_VRES/2)-v_res)/2)
            cam.BinningHorizontalMode.SetValue(PySpin.BinningHorizontalMode_Average) 
            cam.BinningVerticalMode.SetValue(PySpin.BinningVerticalMode_Average)
        else:
            cam.BinningHorizontal.SetValue(1)
            cam.BinningVertical.SetValue(1)
            cam.Width.SetValue(h_res)
            cam.Height.SetValue(v_res)
            #cam.OffsetX.SetValue(6)
           # cam.OffsetX.SetValue((MAX_HRES-h_res)/2)  # TODO doesn't work. Why?
           # cam.OffsetY.SetValue((MAX_VRES-v_res)/2)
            #print("get value", cam.OffsetX.GetValue())
            cam.BinningHorizontalMode.SetValue(PySpin.BinningHorizontalMode_Average) 
            cam.BinningVerticalMode.SetValue(PySpin.BinningVerticalMode_Average)
        print("resolution set to:", h_res, "x", v_res)
            



            
    def run(self):
        try:
            self.setup_logger()
            self.printl(f'pid: {os.getpid()}')
            self.running=False
            
            # Retrieve singleton reference to system object
            system = PySpin.System.GetInstance()

            # Get current library version
            version = system.GetLibraryVersion()
            print('Camera Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

            # Retrieve list of cameras from the system
            cam_list = system.GetCameras()
            num_cameras = cam_list.GetSize()
            print('Number of cameras detected(before reset): %d' % num_cameras)

            # Finish if there are no cameras
            if num_cameras == 0:
                # Clear camera list before releasing system
                cam_list.Clear()
                # Release system instance
                system.ReleaseInstance()
                raise ValueError(f'BlackFly S Camera not detected!!')

            cam = cam_list[0]
            cam.Init()
            print('Execute camera reset')
            cam.DeviceReset()
            cam.DeInit()
            del cam
            cam_list.Clear()
            time.sleep(1)

            cam_list = system.GetCameras()
            while cam_list.GetSize() == 0:
                time.sleep(1)
                print('Wait camera reset')
                cam_list = system.GetCameras()

            #cam_list = system.GetCameras()
            num_cameras = cam_list.GetSize()
            print('Number of cameras detected(after reset): %d' % num_cameras)

            # Finish if there are no cameras
            if num_cameras == 0:
                # Clear camera list before releasing system
                cam_list.Clear()
                # Release system instance
                system.ReleaseInstance()
                raise ValueError(f'BlackFly S Camera not detected!!')

            cam = cam_list[0]
            
            cam.Init()
            nodemap_tldevice = cam.GetTLDeviceNodeMap()
            
            self.change_bufferhandling_mode(cam)
            self.set_acquisition_mode(cam)
            self.disable_manual_trigger(cam)
            if(self.auto_exposure_time):
                self.set_exposure_Auto_mode(cam, "Continuous") #valid modes: Off, Continuous, Once
            else:
                self.set_exposure_Auto_mode(cam, "Off") #valid modes: Off, Continuous, Once
                self.set_exposure_time_us(cam, self.exposure_time_us)

            self.set_framerate(cam, self.frame_rate)

            self.set_resolution(cam, self.h_res, self.v_res)

            if(self.FrameStart_out_enabled):
                self.set_frame_start_gpio_state(cam, True)
            else:
                self.set_frame_start_gpio_state(cam, False)
            
            fps = str(self.frame_rate)
            print("video framerate:", fps)
            self.printl('Camera started')
            import skvideo.io
            if self.filename!=None:
                self.video_writer=skvideo.io.FFmpegWriter(self.filename, inputdict={'-r':fps}, outputdict={'-r':fps})
            self.timestamps=[]

            # Get the value of exposure time to set an appropriate timeout for GetNextImage
            timeout = 0
            if cam.ExposureTime.GetAccessMode() == PySpin.RW or cam.ExposureTime.GetAccessMode() == PySpin.RO:
                # The exposure time is retrieved in µs so it needs to be converted to ms to keep consistency with the unit being used in GetNextImage
                timeout = (int)(cam.ExposureTime.GetValue() / 1000 + 100)
            else:
                print ('Unable to get exposure time. Aborting...')
                raise ValueError(f'Unable to get exposure time. Aborting...')
            
            print("frame tiemout set to:", timeout)

            
            
            if self.show:
                #Camera.StartLive(1)
                cam.BeginAcquisition()
            else:
                #Camera.StartLive(0)
                cam.BeginAcquisition()
                
            while True:
                if not self.queues['command'].empty():
                    cmd=self.queues['command'].get()
                    self.printl(cmd)
                    if cmd=='stop':
                        self.set_frame_start_gpio_state(cam, False)
                        cam.EndAcquisition()
                        cam.DeInit()
                        self.printl('Stop camera')
                        break
                    elif cmd[0]=='save':
                        self.video_writer=skvideo.io.FFmpegWriter(cmd[1], inputdict={'-r':fps}, outputdict={'-r':fps})
                        self.filename_mp4=cmd[1]
                        self.timestamps=[]
                        self.set_frame_start_gpio_state(cam, True)
                    elif cmd=='stop_saving':
                        self.printl('Close video file')
                        self.set_frame_start_gpio_state(cam, False)
                        self.video_writer.close()
                        del self.video_writer
                        if hasattr(self, 'timestamps'):
                            numpy.savetxt(fileop.replace_extension(self.filename_mp4,'.txt'),numpy.array(self.timestamps),fmt='%10.4f')
                        self.queues['response'].put('save done')
                    elif cmd=='ttl_enable':
                        self.set_frame_start_gpio_state(cam, True)
                        self.queues['response'].put('done')
                    elif cmd=='ttl_disable':
                        self.set_frame_start_gpio_state(cam, False)
                        self.queues['response'].put('done')
            
                image_result = cam.GetNextImage(timeout)

                #  Ensure image completion
                if image_result.IsIncomplete():
                    print('Image incomplete with image status %d ...' % image_result.GetImageStatus())
                    continue
                else:                    
                    # Getting the image data as a numpy array
                    frame = image_result.GetNDArray()


                if hasattr(self, 'video_writer'):
                    if len(frame.shape)==2:
                        #frame=numpy.rollaxis(numpy.array([frame]*3),0,3).copy()
                        pass
                    self.video_writer.writeFrame(frame)
                    self.timestamps.append(time.time())
                    

                if self.queues['data'].empty() and frame is not None:#Send frame when queue empty (previous frame was taken
                    self.queues['data'].put(frame)
                    #print("image put ok")

                image_result.Release()  

            if hasattr(self, 'video_writer'):
                self.printl(f'Close video file {self.filename}')
                self.video_writer.close()
                
            cam.DeInit()
            del cam
            cam_list.Clear()
            system.ReleaseInstance()
            if hasattr(self, 'timestamps'):
                numpy.savetxt(fileop.replace_extension(self.filename,'.txt'),numpy.array(self.timestamps),fmt='%10.4f')
            
            self.printl('Leaving process')
        except:
            import traceback
            self.printl(traceback.format_exc())
        

        
class WebCamera(instrument.InstrumentProcess):
    def __init__(self,camera_id,logfile,digital_line,filename=None):
        self.filename=filename
        self.camera_id=camera_id
        self.digital_line=digital_line
        self.queues={'command': multiprocessing.Queue(), 'response': multiprocessing.Queue(), 'data': multiprocessing.Queue()}
        instrument.InstrumentProcess.__init__(self,self.queues,logfile)
        self.control=multiprocessing.Queue()
        self.data=multiprocessing.Queue()
        print(self.logfile)
        
    def stop(self):
        self.queues['command'].put('stop')
        
    def read(self):
        if not self.queues['data'].empty():
            return self.queues['data'].get()
            
    def run(self):
        try:
            self.setup_logger()
            self.printl(f'pid: {os.getpid()}')
            self.running=False
            if self.digital_line is not None:
                digital_output = PyDAQmx.Task()
                digital_output.CreateDOChan(self.digital_line,'do', DAQmxConstants.DAQmx_Val_ChanPerLine)
            fps='30'
            import skvideo.io
            if self.filename!=None:
                self.video_writer=skvideo.io.FFmpegWriter(self.filename, inputdict={'-r':fps}, outputdict={'-r':fps})
            frame_prev=None
            Camera = cv2.VideoCapture(self.camera_id)
            self.printl(Camera)
            while True:
                r, fr=Camera.read()
                if fr is None:
                    continue
                frame=fr.copy()
                if frame_prev is not None and numpy.array_equal(frame_prev, frame):#No new frame in buffer
                    continue
                frame_prev=frame
                if self.digital_line is not None:
                    digital_output.WriteDigitalLines(1,True,1.0,DAQmxConstants.DAQmx_Val_GroupByChannel,numpy.array([1], dtype=numpy.uint8),None,None)
                if hasattr(self, 'video_writer'):
                    if len(frame.shape)==2:
                        frame=numpy.rollaxis(numpy.array([fr]*3),0,3).copy()
                    self.video_writer.writeFrame(numpy.rot90(numpy.rot90(numpy.fliplr(numpy.flipud(frame)))))
                #Digital pulse indicates video save time
                if self.digital_line is not None:
                    digital_output.WriteDigitalLines(1,True,1.0,DAQmxConstants.DAQmx_Val_GroupByChannel,numpy.array([0], dtype=numpy.uint8),None,None)
                if not self.queues['command'].empty():
                    cmd=self.queues['command'].get()
                    self.printl(cmd)
                    if cmd=='stop':
                        Camera.release()
                        break
                if self.queues['data'].empty() and frame is not None:#Send frame when queue empty (previous frame was taken
                    self.queues['data'].put(frame)
                
                time.sleep(1e-3)
            if hasattr(self,  'video_writer'):
                self.video_writer.close()
            if self.digital_line is not None:
                digital_output.ClearTask()
        except:
            import traceback
            self.printl(traceback.format_exc())

        
class TestCamera(unittest.TestCase):
    def setUp(self):
        self.folder=r'f:\Scientific Camera Interfaces\SDK\Python Compact Scientific Camera Toolkit\dlls\64_lib'
        
    @unittest.skip('')
    def test_1_thorlabs_camera(self):        
        tc=ThorlabsCamera(self.folder)
        tc.set(exposure=100000)
        tc.start_()
        for i in range(20):
            frame=tc.get_frame()
        tc.stop()
        tc.close()
        del tc
        self.assertTrue(hasattr(frame, 'dtype'))
        
    @unittest.skip('')
    def test_2_adjust_parameters(self):
        tc=ThorlabsCamera(self.folder, nbit=16)
        tc.set(gain=1)
        tc.set(roi=(1000,1000,1500,1500))
        tc.start_()
        exp1=100000
        exp2=50000
        tc.set(exposure=exp1)
        for i in range(20):
            frame=tc.get_frame()
        self.assertAlmostEqual(exp1,tc.camera.frame_time_us,-3)
        tc.set(exposure=exp2)
        for i in range(20):
            frame=tc.get_frame()
        self.assertAlmostEqual(exp2,tc.camera.frame_time_us,-3)
        tc.set(gain=100)
        frame2=tc.get_frame()
        self.assertGreater(frame2.mean(),frame.mean())#Would pass once lens is mounted
        tc.stop()
        tc.close()
        del tc
        self.assertEqual(frame.shape,(504,504))
      
    @unittest.skip('')
    def test_3_thorlab_camera_process(self):
        logfile=r'f:\tmp\log_camera_{0}.txt'.format(time.time())
        tp=ThorlabsCameraProcess(self.folder,logfile,roi=(0,0,1004,1004))
        tp.start()
        exposure_time=30e-3
        tp.set(exposure=int(exposure_time*1e6))
        tp.set(gain=1)
#        tp.queues['command'].put(('set','exposure',int(exposure_time*1e6)))
#        tp.queues['command'].put(('set','gain',1))
        tp.start_()
        timestamps=[]
        t0=time.time()
        while True:
            frame=tp.read()
            if frame is not None:
                timestamps.append(time.time()-t0)
            if len(timestamps)>100:
                break
        tp.stop()
        tp.terminate() 
        self.assertTrue(hasattr(frame,'dtype'))
        numpy.testing.assert_almost_equal(numpy.diff(timestamps),exposure_time,3)
        self.assertTrue(os.path.exists(logfile))
        self.assertGreater(os.path.getsize(logfile),0)
    
    @unittest.skip('')      
    def test_4_imaging_source_camera(self):
        fn=r'c:\Data\a.mp4'
        cam=ISCamera('DMK 22BUC03 31710198',r'c:\Data\log\camlog.txt','Dev1/port0/line0', frame_rate=60, exposure=1/65, filename=fn)
        cam.start()
        for i in range(60):
            time.sleep(1.1)
        cam.stop()
        time.sleep(1)
        cam.terminate()
        
    @unittest.skip('') 
    def test_5_web_camera(self):
        cam=WebCamera(3,r'c:\Data\log\camlog.txt','Dev1/port0/line0', filename=None)
        cam.start()
        for i in range(60):
            time.sleep(0.1)
            fr=cam.read()
            if fr is not None:
                frame=fr
                from pylab import imshow, show
                imshow(frame)
                show()
                break
        cam.stop()
        time.sleep(1)
        cam.terminate()
        
    @unittest.skip('')       
    def test_6_strobe(self):
        fn=r'f:\a.mp4'
        from visexpman.engine.generic import fileop
        fileop.remove_if_exists(fn)
        import daq
        ai=daq.AnalogRead('Dev2/ai0:1',20,10000)
        cam=ISCamera('DMK 37BUX287 15120861',r'f:\camlog.txt',None, frame_rate=160, exposure=1/250, filename=fn)
        cam.start()
        for i in range(60):
            time.sleep(1.1)
        cam.stop()
        time.sleep(1)
        cam.terminate()
        time.sleep(30)
        data=ai.read()
        import skvideo.io
        from visexpman.engine.generic import signal
        videodata = skvideo.io.vread(fn)
        print(f'Recorded frames {videodata.shape[0]}, n pulses: {signal.trigger_indexes(data[0]).shape[0]/2}')
        print(f'D pulse: {signal.trigger_indexes(data[0]).shape[0]/2-videodata.shape[0]}')
        d=(signal.trigger_indexes(data[0])[-1]-signal.trigger_indexes(data[0])[0])/10e3
        print(f'Duration: {d},frame rate {videodata.shape[0]/d}')
        print(f'Pulse rate: {10e3/numpy.diff(signal.trigger_indexes(data[0])[::2]).mean()} Hz')
        from pylab import plot,show
        plot(data[0]);show()
        
#        import pdb
#       

    @unittest.skip('')       
    def test_7_strobe(self):
        fn=r'c:\Data\cam_test\a.mp4'
        from visexpman.engine.generic import fileop
        fileop.remove_if_exists(fn)
        import daq
        
        cam=ISCamera('DMK 37BUX287 15120861',r'c:\Data\cam_test\camlog.txt',None, frame_rate=60, exposure=1/250, filename=fn)
        cam.start()
        time.sleep(1.52) #ugy beallitani, hogy a mintavetelezes a hamis impulzusok után induljon
                        #biztos megoldas lenne a mintavetelezest a Camera.StartLive utan inditani, de hogy?
        ai=daq.AnalogRead('Dev5/ai0:1',30,10000) 
        for i in range(10):
            time.sleep(1.1)
        cam.stop()
        time.sleep(1)
        cam.terminate()
        time.sleep(2)
        data=ai.read()
        import skvideo.io
        from visexpman.engine.generic import signal
        videodata = skvideo.io.vread(fn)
        print(f'Recorded frames {videodata.shape[0]}, n pulses: {signal.trigger_indexes(data[0]).shape[0]/2}')
        print(f'D pulse: {signal.trigger_indexes(data[0]).shape[0]/2-videodata.shape[0]}')
        d=(signal.trigger_indexes(data[0])[-1]-signal.trigger_indexes(data[0])[0])/10e3
        print(f'Duration: {d},frame rate {videodata.shape[0]/d}')
        print(f'Pulse rate: {10e3/numpy.diff(signal.trigger_indexes(data[0])[::2]).mean()} Hz')
        from pylab import plot,show
        plot(data[0]);show()

    @unittest.skip('')   
    def test_7_BlackFlyS(self):
        cam=BlackFlySCamera(r'c:\data\cam_test\camlog.txt',None, frame_rate=30,auto_exposure_time = True, exposure_time_us=0, filename=None, show=False)
        cam.start()
        for i in range(5):
            time.sleep(0.3)
            fr=cam.read()
            if fr is not None:
                frame=fr
                from pylab import imshow, show
                #import pdb;pdb.set_trace()
                imshow(frame)
                show()
                #break
        #for i in range(3):
         #   time.sleep(1.1)
        cam.stop()
        time.sleep(1)
        cam.terminate()

    def test_8_BlackFlyS_video(self):
        fn=r'c:\data\cam_test\test.mp4'
        from visexpman.engine.generic import fileop
        fileop.remove_if_exists(fn)
        cam=BlackFlySCamera(r'c:\data\cam_test\camlog.txt',None, frame_rate=30, auto_exposure_time = False, exposure_time_us=20000, filename=fn, show=False, FrameStart_out_enabled = True, h_res=1000, v_res=1000)
        cam.start()
        for i in range(5):
            time.sleep(1)
        print('Stop saving')
        cam.stop_saving()
        time.sleep(3)
        cam.stop()
        time.sleep(1)
        cam.terminate()
            

if __name__ == '__main__':
    unittest.main()
