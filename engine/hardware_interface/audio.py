import numpy, time, multiprocessing
import simpleaudio
import threading

class TonePlayer(multiprocessing.Process):
#class TonePlayer(threading.Thread):    
    def __init__(self,frq):
        self.cmd=multiprocessing.Queue()
        self.frq=frq
        self.tone_duration=1000
        self.balance=0.5
        multiprocessing.Process.__init__(self)
        #threading.Thread.__init__(self)
    
    def set_balance(self,left,right):
        if self.cmd.empty():
            self.cmd.put((left,right))
        
    def stop(self):
        self.cmd.put('terminate')
    
    def run(self):
        try:
            while True:
                #time.sleep(0.01)
                if not self.cmd.empty():
                    cmd=self.cmd.get()
                    if cmd=='terminate':
                        break
                    elif isinstance(cmd,tuple):
                        beep_waveform=(numpy.sin(numpy.arange(int(self.tone_duration*44100))*2*numpy.pi*self.frq*2/44100)*32767).astype(numpy.int16)
                        beep_waveform=numpy.repeat(beep_waveform, 2)
                        beep_waveform[::2]=(cmd[0]*beep_waveform[::2]).astype(numpy.int16)
                        beep_waveform[1::2]=(cmd[1]*beep_waveform[1::2]).astype(numpy.int16)
                        if hasattr(self,'audioplayer'):
                            self.audioplayer.stop()
                            #self.audioplayer.wait_done()
                        self.audioplayer=simpleaudio.play_buffer(beep_waveform, 2, 2, 44100)
        except:
            import traceback
            print(traceback.format_exc())
        print('Audio process terminated')

class TonePlayer2():
    def __init__(self,frq):
        self.frq=frq
        self.tone_duration=1000
        self.balance=0.5
        beep_waveform=(numpy.sin(numpy.arange(int(self.tone_duration*44100))*2*numpy.pi*self.frq*2/44100)*32767).astype(numpy.int16)
        beep_waveform=numpy.repeat(beep_waveform, 2)
        self.waveform=beep_waveform.copy()

    def start(self):
        pass

    def terminate(self):
        self.stop()


    def set_balance(self,left,right):
        beep_waveform=self.waveform.copy()
        beep_waveform[::2]=(left*beep_waveform[::2]).astype(numpy.int16)
        beep_waveform[1::2]=(right*beep_waveform[1::2]).astype(numpy.int16)
        if hasattr(self,'audioplayer'):
            self.audioplayer.stop()
            #self.audioplayer.wait_done()
        self.audioplayer=simpleaudio.play_buffer(beep_waveform, 2, 2, 44100)

    def stop(self):
        if hasattr(self,'audioplayer'):
            self.audioplayer.stop()


if __name__=='__main__':
    #ederwander
    import pyaudio 
    import numpy as np 
    import wave 

    chunk = 1024 
    FORMAT = pyaudio.paInt16 
    CHANNELS = 1 
    RATE = 8800 
    K=0 
    DISTORTION = 0.61

    p = pyaudio.PyAudio() 

    stream = p.open(format = FORMAT, 
                    channels = CHANNELS, 
                    rate = RATE, 
                    input = True, 
                    output = True, 
                    frames_per_buffer = chunk) 


  


    while(True): 

        data = stream.read(chunk) 
        data = np.fromstring(data, dtype=np.int16)  
        M = 2*DISTORTION/(1-DISTORTION);
        data = (1+M)*(data)/(1+K*abs(data));
        data = np.array(data, dtype='int16') 
        signal = wave.struct.pack("%dh"%(len(data)), *list(data))
        stream.write(signal) 

    stream.stop_stream() 
    stream.close() 
    p.terminate() 
            
