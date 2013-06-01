from matplotlib.pyplot import plot, show,figure,legend, savefig, subplot, title
from visexpman.engine.hardware_interface import scanner_control
from visexpA.engine.datahandlers import hdf5io
import Image
from visexpA.engine.dataprocessors.generic import normalize
import os
import os.path
from visexpman.engine.generic import utils
import time
import numpy
from visexpman.engine.hardware_interface import daq_instrument
from visexpman.engine.hardware_interface import camera_interface
from visexpman.engine.generic import configuration
from visexpman.engine.generic import file

class TestConfig(configuration.Config):
    def _create_application_parameters(self):
        self.F = [20.0, 40.0, 80.0, 100.0, 150.0, 200.0, 250.0, 400.0, 500.0, 700.0, 850.0, 1000.0, 1200.0, 1500.0, 1700.0, 1800.0, 1900.0, 2000.0, 2100.0, 2200.0, 2400.0, 2450.0, 2500.0, 2600.0, 2700.0, 2800.0, 2900.0, 3000.0]
        self.F = numpy.array(self.F*2).reshape((2, len(self.F))).flatten('F')
        self.F[::2] = 0
        self.F = self.F.tolist()
        self.F.append(0)
        self.A = [100/128.0*2]#100 um
        self.DURATION = 3.0
        self.CAMERA_FRAME_RATE = 30.0
        VIDEO_FORMAT = 'RGB24 (320x240)'
        self.PREPOST_DELAYS = 2.0
        DAQ_CONFIG = [
        {
        'ANALOG_CONFIG' : 'ao',
        'DAQ_TIMEOUT' : 2.0, 
        'AO_SAMPLE_RATE' : 400000,
        'AO_CHANNEL' : 'Dev1/ao0:1',
        'MAX_VOLTAGE' : 8.0,
        'MIN_VOLTAGE' : -8.0,
        'DURATION_OF_AI_READ' : 2.0,
        'AO_SAMPLING_MODE' : 'finite', 
        'ENABLE' : True
        },
        ]
        self._create_parameters_from_locals(locals())

class ScannerCalibration(camera_interface.ImagingSourceCamera, daq_instrument.AnalogIO):
    def __init__(self, config):
        camera_interface.ImagingSourceCamera.__init__(self, config)
        daq_instrument.AnalogIO.__init__(self, config)
#        self.generate_square_signal(self.A, self.duration)
        self.generate_signal(self.config.F, self.config.A, self.config.DURATION)
        
    def generate_signal(self, f, A, duration, two_channel = False):
        self.t = numpy.linspace(0, duration,  duration*self.ao_sample_rate, False)
        signal0 = numpy.zeros(self.t.size*len(f)*len(A))
        signal1 = numpy.zeros(self.t.size*len(f)*len(A))
        for i in range(len(A)):
            for j in range(len(f)):
                if f[j] == 0:
                    signal_i0 = numpy.zeros_like(self.t)
                    signal_i1 = numpy.zeros_like(self.t)
                else:
                    signal_i0 = 0.5*A[i]*numpy.sin(2*numpy.pi*self.t*f[j])
                    if two_channel:
                        signal_i1 = 0.5*A[i]*numpy.cos(2*numpy.pi*self.t*f[j])
                    else:
                        signal_i1 = numpy.zeros_like(self.t)
                index = len(f)*i+j
                signal0[signal_i0.size*index:signal_i0.size*(index+1)] = signal_i0
                signal1[signal_i1.size*index:signal_i1.size*(index+1)] = signal_i1
        self.waveform = numpy.array([signal0, signal1]).T
        from matplotlib.pyplot import plot, show
#        plot(self.waveform[:,0])
#        plot(self.waveform[:,1])
#        show()
        pass
        
    def generate_square_signal(self, A, duration):
        self.waveform = numpy.zeros(duration*self.ao_sample_rate)
        self.waveform[0.1*self.waveform.shape[0]:0.6*self.waveform.shape[0]] = A

    def run(self):
        camera_interface.ImagingSourceCamera.start(self)
        self.start_daq_activity()
        for i in range(int((len(self.config.A)*len(self.config.F)*self.config.DURATION+ self.config.PREPOST_DELAYS)*config.CAMERA_FRAME_RATE)):
            camera_interface.ImagingSourceCamera.save(self)
        
    def close(self):
        self.finish_daq_activity()
        self.waveform = numpy.zeros((100, 2))#Make sure that outputs are on 0V
        daq_instrument.AnalogIO.run(self)
        daq_instrument.AnalogIO.release_instrument(self)
        camera_interface.ImagingSourceCamera.stop(self)
        camera_interface.ImagingSourceCamera.close(self)
        self.process_data()
        
    def process_data(self):
        import tiffile
        tiffile.imsave(file.generate_filename('v:\\debug\\20130503\\camcalibs\\calib.tiff'), self.video, software = 'visexpman')
        p = file.generate_filename('v:\\debug\\20130503\\camcalibs\\calib.hdf5')
        hdf5io.save_item(p, 'video', self.video, filelocking = False)
        hdf5io.save_item(p, 'config', self.config.get_all_parameters(), filelocking = False)
        
def scanner_calib():
    config = TestConfig()
    sc = ScannerCalibration(config)
    sc.run()
    sc.close()
    print 'DONE'
    
def evaluate_calibdata():
    p = '/mnt/datafast/debug/20130503/calibs'
    p = '/mnt/rznb/data/20130503/calibs'
    p = '/home/rz/visexp/data/calibs'
    max_linearity_error = 10e-2
    a = []
    fi = []
    frq = []
    fs =os.listdir(p)
    fs.sort()
    fs = fs[:-2]
    fs.reverse()
    for f in fs:
        calibdata = hdf5io.read_item(os.path.join(p, f), 'calibdata', filelocking=False)
    #    pmt = calibdata['pmt']
    #    mask = utils.resample_array(calibdata['mask'], calibdata['parameters']['binning_factor'])
        res = scanner_control.scanner_bode_diagram(calibdata['pmt'], utils.resample_array(calibdata['mask'], calibdata['parameters']['binning_factor']), calibdata['parameters']['scanner_speed'], max_linearity_error)
        a.append(res['sigmas'][0])
        fi.append(res['means'][0])
        frq.append(res['frq'][0])
#        figure(len(frq))
#        t = numpy.linspace(0, 1, res['signals'][0].shape[0])
#        plot(t, res['signals'][0])
#        plot(t, scanner_control.gauss(t, res['gauss_amplitudes'][0], fi[-1], a[-1]))
#        title('{0} Hz, {1}, {2}'.format(frq[-1], fi[-1], a[-1]))
    #print frq
    phase = 2*utils.sinus_linear_range(max_linearity_error)*(fi-fi[-1])
    figure(len(frq)+1)
    plot(frq, a/a[-1])
    plot(frq, phase)
    
    import scipy.optimize
    p0 = [1.0/2500,0.0]
    frq = numpy.array(frq)
    coeff, var_matrix = scipy.optimize.curve_fit(linear, frq, phase, p0=p0)
#    plot(frq, p0[0]*frq+p0[1])
    plot(frq, coeff[0]*frq+coeff[1])
    legend(('gain', 'phase', 'fitted'))
    print coeff#[ 0.00043265 -0.02486131]
    
    #    show()
    
def linear(x, *p):
    A, B = p
    return A*x + B

    
def evaluate_videos():
    '''
    Strategy:
    First approach:
    1 .Find segment indexes in video
    2. Add frames within one segment
    3. Plot accumulated fragments to tiff sequence
    '''
    p = '/mnt/datafast/debug/20130503/camcalibs'
    i=0
    for pp in ['x', 'y']:
        i+=1
        for fn in os.listdir(os.path.join(p, pp)):
            if 'hdf5' in fn:
                evaluate_video(os.path.join(p, pp, fn),pp)
#    show()
    
def evaluate_video(p, axis):
    h=hdf5io.Hdf5io(p,filelocking=False)
    h.load('video')
    rvideo = h.video
    h.load('config')
    h.close()
    frqs = h.config['F'][1::2]
    mask = numpy.where(rvideo<2*rvideo.min(),False, True)
    masked_video = rvideo*mask
    sizes = []
    for fri in range(rvideo.shape[0]):
        frame = masked_video[fri,:,:]
        sizes.append(numpy.nonzero(frame)[0].shape[0])
    sizes = numpy.array(sizes)
    #calculate segment indexes
    diffs = abs(numpy.diff(sizes))
    indexes = numpy.nonzero(numpy.where(diffs > diffs.max()/3, True, False))[0]
    indexes = indexes[numpy.nonzero(numpy.where(numpy.diff(indexes) == 1, 0, 1))[0]]
    indexes = indexes[:-1]
    masked_video = normalize(masked_video,outtype=numpy.uint8)
    segments = numpy.split(masked_video, indexes)[1::2]
    mean_images = numpy.zeros((len(segments), masked_video.shape[1], masked_video.shape[2]))
    curves = numpy.zeros((len(segments), masked_video.shape[2]))
    for segment_i in range(len(segments)):
        segment_mean = segments[segment_i].mean(axis=0)
        curves[segment_i] = segment_mean.mean(axis=0)
#        dim_order = [0, 1]
#        from visexpA.engine.dataprocessors import signal
#        sigma = 2
#        positions = signal.regmax(segment_mean,dim_order, sigma = sigma)
#        for pos in positions:
#            segment_mean[pos['row'],pos['col']] = 255
        mean_images[segment_i, :, :] = segment_mean
    #Beam profile over frequencies
    Image.fromarray(normalize(curves,outtype=numpy.uint8)).show()
#    mean_images = normalize(mean_images,outtype=numpy.uint8)
    import tiffile
    tiffile.imsave(file.generate_filename('/mnt/rznb/1.tiff'), numpy.cast['uint8'](mean_images), software = 'visexpman')
    pass
    return
    
def delay_curve():
    #1. method
    folder1 = '/home/rz/visexp/data/calibs'
    fns = os.listdir(folder1)
    fns.sort()
    fns = fns[:-2]
    resolutions = []
    delays = []
    for fn in fns:
        calibdata = hdf5io.read_item(os.path.join(folder1, fn),  'calibdata',  filelocking=False)
        signal = calibdata['pmt']
        mask = utils.resample_array(calibdata['mask'], calibdata['parameters']['binning_factor'])
        scanner = utils.resample_array(calibdata['waveform'][:, 0], calibdata['parameters']['binning_factor'])
        #Calculate scan size and resolution
        npoints = numpy.diff(numpy.nonzero(numpy.diff(mask))[0][:2])[0]
        scan_range = ((scanner*mask).max() - (scanner*mask).min())*128/2
        resolutions.append(npoints/scan_range)
        #Extract peak delay
        indexes = numpy.nonzero(numpy.diff(mask))[0]
        indexes = indexes[:indexes.shape[0]/2]
        signal = numpy.array((numpy.split(signal, indexes)[1::2])).mean(axis=0)[:,0]
        p0 = [1., signal.argmax(), 1.]
        import scipy.optimize
        coeff, var_matrix = scipy.optimize.curve_fit(scanner_control.gauss, numpy.arange(signal.shape[0]), signal, p0=p0)
        delays.append(coeff[1]/signal.shape[0])
#        plot(signal)
#        plot(scanner_control.gauss(numpy.arange(signal.shape[0]),  *coeff))
        pass
    #2. method
    folder2 = '/home/rz/visexp/data/shift_size_calib'
    fns = os.listdir(folder2)
    fns.sort()
    fns = fns
    delays = {}
    figct = 0
    for fn in fns:
        rawdata = hdf5io.read_item(os.path.join(folder2,  fn),  'rawdata',  filelocking=False)
        scan_parameters = hdf5io.read_item(os.path.join(folder2,  fn),  'scan_parameters',  filelocking=False)
        resolution = 1/scan_parameters['resolution']
        scan_range = scan_parameters['scan_size']['col']
        curve = rawdata[:,:,0,0].mean(axis=0)
        
        p0 = [1., curve.argmax(), 1.]
        import scipy.optimize
        try:
            coeff, var_matrix = scipy.optimize.curve_fit(scanner_control.gauss, numpy.arange(curve.shape[0]), curve, p0=p0)
            delay = coeff[1]/curve.shape[0]
        except:
            delay = 0
        if not delays.has_key(scan_range):
            delays[scan_range] = []
        delays[scan_range].append([resolution, delay])
        figure(figct)
        plot(curve)
        plot(scanner_control.gauss(numpy.arange(curve.shape[0]),  *coeff))
        t = '{0} um {1} pixel um {2:0.3f}'.format(scan_range,  resolution, delay)
        title(t)
        figct += 1
        fn = os.path.join('/home/rz/Downloads/plots', t+'.png')
        savefig(fn)
        fig = numpy.asarray(Image.open(fn))
        pic = normalize(rawdata[:,:,0,0], numpy.uint8)
        ima = numpy.zeros((fig.shape[0]+pic.shape[0],  max(fig.shape[1],  pic.shape[1]), 3 ),  dtype = numpy.uint8)
        ima[0:fig.shape[0],  0:fig.shape[1],  :] = fig[:, :, 0:3]
        ima[fig.shape[0]: fig.shape[0] + pic.shape[0], 0:pic.shape[1],  1] = pic
        os.remove(fn)
        Image.fromarray(ima).save(fn)
        pass
    figure(figct)
    srs = delays.keys()
    srs.sort()
    for scan_range in srs:
        data = numpy.array(delays[scan_range])
        plot(data[:, 0],  data[:, 1],  '*-')
    fn = os.path.join('/home/rz/Downloads/plots/pos.png')
    legend(map(str, srs))
    savefig(fn)
    pass
    pass
    pass
    
if __name__ == "__main__":
    delay_curve()
#    evaluate_videos()
#    evaluate_calibdata()
#    show()
#    scanner_calib()
