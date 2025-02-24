import numpy, time, multiprocessing
import simpleaudio

class TonePlayer(multiprocessing.Process):
    def __init__(self,frq):
        self.cmd=multiprocessing.Queue()
        self.frq=frq
        self.tone_duration=1000
        self.balance=0.5
        multiprocessing.Process.__init__(self)
    
    def set_balance(self,left,right):
        self.cmd.put((left,right))
        
    def stop(self):
        self.cmd.put('terminate')
    
    def run(self):
        try:
            while True:
                time.sleep(0.2)
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
                            self.audioplayer.wait_done()
                        self.audioplayer=simpleaudio.play_buffer(beep_waveform, 2, 2, 44100)
        except:
            import traceback
            print(traceback.format_exc())
