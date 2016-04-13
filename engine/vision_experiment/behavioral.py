import os,sys,time,threading,Queue,tempfile,random,shutil,multiprocessing,copy,logging
import numpy,scipy.io,visexpman, copy, traceback
import serial
from PIL import Image
import cv2
import PyQt4.Qt as Qt
import PyQt4.QtGui as QtGui
import PyQt4.QtCore as QtCore
import pyqtgraph,hdf5io,unittest
from visexpman.engine.generic import gui,utils,videofile,fileop,introspect
from visexpman.engine.hardware_interface import daq_instrument, digital_io
from visexpman.engine.vision_experiment import experiment_data, configuration
#TODO: protocol success rate summary
#TODO: valves will be controlled by arduino to ensure valve open time is precise

def object_parameters2dict(obj):
    return dict([(vn,getattr(obj, vn)) for vn in dir(obj) if vn.isupper()] )

class Protocol(object):
    def __init__(self,engine):
        self.engine=engine
        self.reset()
        
    def update(self):
        '''
        In subclass this method calculates if reward/punishment has to be given
        '''
        
    def stat(self):
        '''
        In a subclass this method calculates the actual success rate
        '''
    def reset(self):
        '''
        Resets state variables
        '''
        
        
def get_protocol_names():
    objects=[getattr(sys.modules[__name__], c) for c in dir(sys.modules[__name__])]
    pns=[o.__name__ for o in objects if 'Protocol' in introspect.class_ancestors(o)]
    return pns
    
class KeepStopReward(Protocol):
    '''
        self.PROTOCOL_KEEP_STOP_REWARD['reward period']=20#sec
        self.PROTOCOL_KEEP_STOP_REWARD['punishment time']=30#sec
        self.PROTOCOL_KEEP_STOP_REWARD['reward period increment']=0#sec
        self.PROTOCOL_KEEP_STOP_REWARD['max reward perdion']=3600.0#sec
        
                if not hasattr(self, 'keep_stop_time'):
            self.reward_period=self.config.PROTOCOL_KEEP_STOP_REWARD['reward period']
        speed=numpy.where(abs(self.checkdata[2])>self.config.MOVE_THRESHOLD,1,0)#speed mask 1s: above threshold
        if speed.sum()>0:
            if self.punishment:
                self.punishment_start_time=time.time()
                self.checkdata=numpy.copy(self.empty)    
            else:
                self.log('Start of water deprivation');
                self.punishment = True
                self.punishment_counter+=1
                self.punishment_start_time=time.time()
                self.checkdata=numpy.copy(self.empty)  
        else:
            if self.punishment:
                if time.time()-self.punishment_start_time>self.config.PROTOCOL_KEEP_STOP_REWARD['punishment time']:
                    self.punishment = False
                    self.log('End of water deprivation')
            else:
                if self.checkdata[0,-1]-self.checkdata[0,0]>self.reward_period:
                    speed=numpy.where(self.checkdata[2]>self.config.MOVE_THRESHOLD,1,0)#speed mask 1s: above threshold
                    t=self.checkdata[0]-self.checkdata[0,0]#Time is shifted to 0
                    #Calculate index for last stoptime duration
                    index=numpy.where(t>t.max()-self.reward_period)[0].min()
                    stop_speed=speed[index:]#...and the stop part
                    stop=stop_speed.sum()==0#All elements of the stop part shall be below threshold
                    if stop:
                        self.reward()#... give reward and...
                        self.checkdata=numpy.copy(self.empty)#Reset checkdata

    '''
    
class KeepRunningReward(Protocol):
    '''A reward is given if the animal was running for RUN_TIME '''
    RUN_TIME=5.0
    def reset(self):
        self.engine.run_time=self.RUN_TIME
        self.nrewards=0
    
    def update(self):
        indexes=[i for i in range(len(self.engine.events)) if not self.engine.events[i]['ack'] and self.engine.events[i]['type']=='run']
        self.last_update=time.time()
        if len(indexes)>0:
            for i in indexes:
                self.engine.events[i]['ack']=True
            self.engine.reward()
            self.nrewards+=1
        
    def stat(self):
        '''
        Success rate is the ratio of time while the animal was running
        '''
        if self.engine.speed_values.shape[0]>0:
            elapsed_time=self.last_update-self.engine.speed_values[0,0]
            success_rate=self.nrewards*self.RUN_TIME/elapsed_time
            success_rate = 1.0 if success_rate>=1 else success_rate
            return {'rewards':self.nrewards, 'Success Rate': success_rate}
        
class ForcedKeepRunningReward(KeepRunningReward):
    '''
    Animal gets a reward if it was running for RUN_TIME.
    If the animal is not moving for STOP_TIME it will be forced to move for RUN_FORCE_TIME seconds.
    '''
    RUN_TIME=5.0
    STOP_TIME=6.5
    RUN_FORCE_TIME = 3.0
    
    def reset(self):
        self.engine.run_time=self.RUN_TIME
        self.engine.stop_time=self.STOP_TIME
        self.nrewards=0
        self.nforcedruns=0
        
    def update(self):
        KeepRunningReward.update(self)
        indexes=[i for i in range(len(self.engine.events)) if not self.engine.events[i]['ack'] and self.engine.events[i]['type']=='stop']
        for i in indexes:
            self.engine.events[i]['ack']=True
        if len(indexes)>0:
            self.engine.airpuff()
            self.engine.forcerun(self.RUN_FORCE_TIME)
            self.nforcedruns+=1

    def stat(self):
        '''
        Number of rewards in session, number of forced runs
        '''
        stats=KeepRunningReward.stat(self)
        if stats==None:return
        stats['forcedruns']=self.nforcedruns
        return stats
                
        
class StopReward(Protocol):
    '''After running for RUN_TIME, the animal gets reward if stops for STOP_TIME
    '''
    RUN_TIME=10.0
    STOP_TIME=0.5
    
    def reset(self):
        self.engine.run_time=self.RUN_TIME
        self.engine.stop_time=self.STOP_TIME
        self.nrewards=0
        self.nruns=0
        self.run_complete=False
        
    def update(self):
        indexes=[i for i in range(len(self.engine.events)) if not self.engine.events[i]['ack'] and self.engine.events[i]['type'] in ['run', 'stop']]
        if len(indexes)>0:
            if self.engine.events[indexes[0]]['type']=='run':
                self.run_complete=True
                self.nruns+=1
            elif self.run_complete and self.engine.events[indexes[0]]['type']=='stop':
                self.run_complete=False
                self.engine.reward()
                self.nrewards+=1
            self.engine.events[indexes[0]]['ack']=True
        
    def stat(self):
        '''
        Success rate is the number of reward per the overall number of run events
        '''
        if self.nruns==0:
            success_rate=0
        else:
            success_rate=self.nrewards/float(self.nruns)
        return {'rewards':self.nrewards, 'runs': self.nruns, 'Success Rate': success_rate}
        


class StimStopReward(Protocol):
    '''
    If a run event occurs, turn on stimulus. Wait for stop event which should occur within DELAY_AFTER_RUN.
    If stop event does not happen, turn off stimulus and generate next randomized run time and wait for next run time
    If stop event takes place turn off stimulus, give reward and generate next randomized run time
    
    Randomized runtime :
    RUN_TME+a random number between 0 and RANDOM_TIME_RANGE in RANDOM_TIME_STEPs
    '''

    DELAY_AFTER_RUN=5.0
    RUN_TIME=5.0
    STOP_TIME=0.5
    RANDOM_TIME_RANGE=10.0
    RANDOM_TIME_STEP=0.5
    def reset(self):
        self.engine.run_time=self.generate_runtime()
        self.engine.stop_time=self.STOP_TIME
        self.nrewards=0
        self.nstimulus=0
        self.noreward=0
        self.run_complete=False
        
    def generate_runtime(self):
        nsteps=round(self.RANDOM_TIME_RANGE/self.RANDOM_TIME_STEP)
        new_runtime=self.RUN_TIME+round((random.random()*nsteps))*self.RANDOM_TIME_STEP
        logging.info('New runtime generated: {0} s'.format(new_runtime))
        return new_runtime
        
    def update(self):
        now=time.time()
        run_indexes=[i for i in range(len(self.engine.events)) if not self.engine.events[i]['ack'] and self.engine.events[i]['type'] =='run']
        if len(run_indexes)>0:
            self.run_complete=True
            self.run_complete_time=self.engine.events[run_indexes[0]]['t']
            self.engine.events[run_indexes[0]]['ack']=True
            self.engine.stimulate()
            self.nstimulus+=1
        elif self.run_complete:
            if now-self.run_complete_time>self.DELAY_AFTER_RUN:
                self.run_complete=False
                self.engine.run_time=self.generate_runtime()
                logging.info('No reward')
                self.noreward+=1
            else:
                stop_indexes=[i for i in range(len(self.engine.events)) if not self.engine.events[i]['ack'] and self.engine.events[i]['type'] =='stop' and self.engine.events[i]['t']>self.run_complete_time]
                if len(stop_indexes)>0:
                    for i in stop_indexes:
                        self.engine.events[i]['ack']=True
                    self.engine.reward()
                    self.nrewards+=1
                    self.run_complete=False
                    self.engine.run_time=self.generate_runtime()
        
    def stat(self):
        '''
        Success rate is nreward/nstimulus. nstimulus is the number when animal stopped and stimulus is presented. 
        nreward is the number when after stimulus the animal is stopped.
        '''
        if self.nstimulus==0:
            success_rate=0
        else:
            success_rate=self.nrewards/float(self.nstimulus)
        return {'rewards':self.nrewards, 'stimulus': self.nstimulus, 'No reward': self.noreward, 'Success Rate': success_rate}

class TreadmillSpeedReader(multiprocessing.Process):
    '''
    
    '''
    def __init__(self,queue, machine_config, emulate_speed=False):
        multiprocessing.Process.__init__(self)
        self.queue=queue
        self.speed_q=multiprocessing.Queue()
        self.machine_config=machine_config
        self.emulate_speed=emulate_speed
        
    def emulated_speed(self,t):
        if t<10:
            spd=5.0
        elif t>=10 and t<30:
            spd=20.0+t*1e-1
        elif t>=30:
            spd=2
        else:
            spd=10
        spd = 10*int(int(t/21)%2==0)
        spd+=numpy.random.random()-0.5
        return spd
        
    def run(self):
        self.last_run=time.time()
        self.start_time=time.time()
        logging.info('Speed reader started')
        while True:
            now=time.time()
            if now-self.last_run>self.machine_config.TREADMILL_SPEED_UPDATE_RATE:
                self.last_run=copy.deepcopy(now)
                if self.emulate_speed:
                    spd=self.emulated_speed(now-self.start_time)
                else:
                    pass
                self.speed_q.put([now,spd])
            
            if not self.queue.empty():
                msg=self.queue.get()
                if msg=='terminate':
                    break
            time.sleep(0.01)
        logging.info('Speed reader finished')

class CameraHandler(object):
    '''
    Reads image from camera, transmits via queue and saves frames to file
    '''
    def __init__(self,machine_config):
        self.machine_config=machine_config
        self.save_video=False
        self.camera=None
        self.tperiod=int(1.0/self.machine_config.CAMERA_FRAME_RATE)
        self.frame_counter=0
        self.last_runtime=time.time()
        if self.machine_config.ENABLE_CAMERA:
            try:
                self.camera = cv2.VideoCapture(0)#Initialize video capturing
                self.camera.set(3, self.machine_config.CAMERA_FRAME_WIDTH)#Set camera resolution
                self.camera.set(4, self.machine_config.CAMERA_FRAME_HEIGHT)
                logging.info('Camera initialized')
            except:
                logging.warning('no camera present')
            
    def start_video_recording(self,videofilename):
        if self.save_video and 0:
            numpy.save(self.videofilename.replace(os.path.splitext(self.videofilename)[1], '_frame_times.npy'),numpy.array(self.frame_times))
        self.video_saver= cv2.VideoWriter(videofilename,cv2.cv.CV_FOURCC(*'XVID'), 12,#self.machine_config.CAMERA_FRAME_RATE, 
                                                    (self.machine_config.CAMERA_FRAME_WIDTH,self.machine_config.CAMERA_FRAME_HEIGHT))
        self.save_video=True
        self.frame_counter=0
        self.frame_times=[]
        self.videofilename=videofilename
        logging.info('Recording video to {0} started'.format(videofilename))
        
    def stop_video_recording(self):
        self.save_video=False
        del self.video_saver
        logging.info('Recording video ended')
                
    def update_video_recorder(self):
        now=time.time()
        if now-self.last_runtime>=self.tperiod:
            now1=time.time()
            self.fps=1.0/(now1-self.last_runtime)
            self.last_runtime=now1
            frame = self.read_frame()
            self.to_gui.put({'update_main_image' :frame})
        
    def close_video_recorder(self):
        if hasattr(self.camera,'release'):
            self.camera.release()#Stop camera operation
                
    def read_frame(self):
        if self.camera is None:
            return
        ret, frame = self.camera.read()#Reading the raw frame from the camera
        if frame is None or not ret:
            return
        if self.save_video:
                self.video_saver.write(frame)
                self.frame_times.append(time.time())
        frame_color_corrected=numpy.zeros_like(frame)#For some reason we need to rearrange the color channels
        frame_color_corrected[:,:,0]=frame[:,:,2]
        frame_color_corrected[:,:,1]=frame[:,:,1]
        frame_color_corrected[:,:,2]=frame[:,:,0]
        self.frame_counter+=1
        return frame_color_corrected

class BehavioralEngine(threading.Thread,CameraHandler):
    def __init__(self,machine_config):
        self.machine_config=machine_config
        threading.Thread.__init__(self)
        CameraHandler.__init__(self,machine_config)
        self.from_gui = Queue.Queue()
        self.to_gui = Queue.Queue()
        self.context_filename = fileop.get_context_filename(self.machine_config,'npy')
        self.context_variables=['datafolder','parameters','current_animal']
        self.load_context()
        self.load_animal_file(switch2plot=True)
        self.speed_reader_q=multiprocessing.Queue()
        self.speed_reader=TreadmillSpeedReader(self.speed_reader_q,self.machine_config,emulate_speed=True)
        self.speed_reader.start()
        self.session_ongoing=False
        self.reset_data()
        self.enable_speed_update=True
        self.varnames=['speed_values','reward_values','airpuff_values','stimulus_values','forcerun_values', 'events','frame_times']
        
    def reset_data(self):
        self.speed_values=numpy.empty((0,2))
        self.reward_values=numpy.empty((0,2))
        self.airpuff_values=numpy.empty((0,2))
        self.stimulus_values=numpy.empty((0,2))
        self.forcerun_values=numpy.empty((0,2))
        self.events=[]
        logging.info('Data traces cleared')
        
    def load_context(self):
        if os.path.exists(self.context_filename):
            try:
                context_stream = numpy.load(self.context_filename)
            except:
                raise IOError('{0} context file may be corrupt. Delete it and start software again'.format(self.context_filename))
            context=utils.array2object(context_stream)
            for k,v in context.items():
                setattr(self,k,v)
        if not hasattr(self,'datafolder'):
            self.datafolder=self.machine_config.EXPERIMENT_DATA_PATH
            
    def save_context(self):
        context={}
        for vn in self.context_variables:
            if hasattr(self,vn):
                context[vn]=getattr(self,vn)
        context_stream=utils.object2array(context)
        numpy.save(self.context_filename,context_stream)
        
    def ask4confirmation(self,message):
        self.to_gui.put({'ask4confirmation':message})
        while True:
            if not self.from_gui.empty():
                break
            time.sleep(0.15)
        result=self.from_gui.get()
        logging.info('Ask for confirmation: {0}, {1}'.format(message, result))
        return result
        
    def notify(self,title,message):
        logging.info('{0}, {1}'.format(title, message))
        self.to_gui.put({'notify':{'title': title, 'msg':message}})
        
    def set_animal_id(self,current_animal):
        self.current_animal=current_animal
        self.load_animal_file(switch2plot=True)
        
    def add_animal(self,name):
        if self.session_ongoing:
            return
        foldername=os.path.join(self.datafolder,name)
        if os.path.exists(foldername):
            self.notify('Warning', '{0} folder already exists'.format(foldername))
            return
        os.mkdir(foldername)
        logging.info('{0} animal created'.format(name))
        self.set_animal_id(name)
        self.to_gui.put({'statusbar':''})
        
    def add_animal_weight(self,date,weight):
        if self.session_ongoing:
            return
        self.load_animal_file(date=date,weight=weight, switch2plot=True)
        
    def remove_last_animal_weight(self):
        if self.session_ongoing:
            return
        self.load_animal_file(remove_last=True, switch2plot=True)
        
    def load_animal_file(self, date=None,weight=None,remove_last=False, switch2plot=False):
        if not hasattr(self,'current_animal'):
            return
        self.current_animal_file=os.path.join(self.datafolder,self.current_animal,'animal_'+self.current_animal+'.hdf5')
        if not os.path.exists(os.path.dirname(self.current_animal_file)):
            return
        h=hdf5io.Hdf5io(self.current_animal_file)
        h.load('weight')
        save=False
        if not hasattr(h,'weight') or h.weight.shape[0]==0:
            h.weight=numpy.empty((0,2))
            save=True
        if remove_last:
            save=True
            if h.weight.shape[0]>0:
                h.weight=h.weight[:-1]
                if h.weight.shape[0]==0:
                    h.weight=numpy.empty((0,2))
        if date !=None and weight!=None:
            if h.weight.shape[0]>0 and date in h.weight[:,0]:
                logging.warning('This day is already added')
            else:
                h.weight=numpy.concatenate((h.weight,numpy.array([[date,weight]])))
                save=True
        if save:
            if h.weight.shape[0]>1:
                h.weight=h.weight[h.weight.argsort(axis=0)[:,0]]#Sort by timestamp
            h.save('weight')
        self.weight=h.weight
        h.close()
        if self.weight.shape[0]==0:
            self.weight=numpy.zeros((1,2))
        self.to_gui.put({'update_weight_history':self.weight.copy()})
        if switch2plot:
            self.to_gui.put({'switch2_animal_weight_plot':[]})
            
    def forcerun(self,duration):
        logging.info('Force run for {0} s'.format(duration))
        now=time.time()
        self.forcerun_values=numpy.concatenate((self.forcerun_values,numpy.array([[now, 1]])))
            
    def reward(self):
        logging.info('Reward')
        now=time.time()
        self.reward_values=numpy.concatenate((self.reward_values,numpy.array([[now, 1]])))
        
    def airpuff(self):
        logging.info('Airpuff')
        now=time.time()
        self.airpuff_values=numpy.concatenate((self.airpuff_values,numpy.array([[now, 1]])))
        
    def set_valve(self,channel,state):
        pass
        
    def stimulate(self):
        logging.info('Stimulus')
        now=time.time()
        self.stimulus_values=numpy.concatenate((self.stimulus_values,numpy.array([[now-1e-3, 0],[now, 1],[now+self.parameters['Stimulus Pulse Duration'], 1],[now+self.parameters['Stimulus Pulse Duration']+1e-3, 0]])))

    def set_speed_update(self, state):
        self.enable_speed_update=state
        #Reset data buffer if no recording is ongoing and file was opened
        if state and not self.session_ongoing:#and hasattr(self, 'datafile_opened') and time.time()-self.datafile_opened<30:
            self.reset_data()
        
    def update_speed_values(self):
        new_value=[]
        if not self.speed_reader.speed_q.empty() and self.speed_reader.speed_q.qsize()>2:
            while not self.speed_reader.speed_q.empty():
                new_value.append(self.speed_reader.speed_q.get())
        if len(new_value)>0:
            self.speed_values=numpy.concatenate((self.speed_values,numpy.array(new_value)))
            return True
        else: return False

    def update_plot(self):
        if not self.speed_values.shape[0]>0:
            return
        xs,ys=self.values2trace(self.speed_values)
        now=xs[-1]
        t0=xs[0]
        x=[xs]
        y=[ys]
        trace_names=['speed']
        if sum([self.parameters[k] for k in ['Reward Trace', 'Airpuff Trace', 'Stimulus Trace', 'Run Events Trace', 'Stop Events Trace', 'Force Run Trace']])>3:
            return
        if self.reward_values.shape[0]>0 and self.parameters['Reward Trace']:
            xr,yr=self.values2trace(self.reward_values, now)
            if xr.shape[0]>0:
                t0=min(t0, xr[0])
                x.append(xr)
                y.append(yr*ys.max())
                trace_names.append('reward')
        if self.airpuff_values.shape[0]>0 and self.parameters['Airpuff Trace']:
            xa,ya=self.values2trace(self.airpuff_values, now)
            if xa.shape[0]>0:
                t0=min(t0, xa[0])
                x.append(xa)
                y.append(ya*ys.max())
                trace_names.append('airpuff')
        if self.stimulus_values.shape[0]>0 and self.parameters['Stimulus Trace']:
            xst,yst=self.values2trace(self.stimulus_values, now)
            if xst.shape[0]>0:
                t0=min(t0, xst[0])
                x.append(xst)
                y.append(yst*ys.max())
                trace_names.append('stimulus')
        if self.forcerun_values.shape[0]>0 and self.parameters['Force Run Trace']:
            xf,yf=self.values2trace(self.forcerun_values, now)
            if xf.shape[0]>0:
                t0=min(t0, xf[0])
                x.append(xf)
                y.append(yf*ys.max())
                trace_names.append('forcerun')
        for event_type in ['run', 'stop']:
            if self.parameters['{0} Events Trace'.format(event_type.capitalize())]:
                xe=numpy.array([event['t'] for event in self.events if event['type']==event_type])
                if xe.shape[0]>0:
                    ye=numpy.ones_like(xe)*ys.max()*0.5
                    xe,ye=self.values2trace(numpy.array([xe,ye]).T, now)
                    if xe.shape[0]>0:
                        x.append(xe)
                        y.append(ye)
                        trace_names.append(event_type)
                        t0=min(t0,x[-1][0])
        
        t0=numpy.concatenate(x).min()
        for xi in x:
            xi-=t0
        self.to_gui.put({'update_events_plot':{'x':x, 'y':y, 'trace_names': trace_names}})

    def values2trace(self, values, now=None):
        x=values[:,0].copy()
        xl= x[-1] if now==None else now
        indexes=numpy.nonzero(numpy.where(x>xl-self.parameters['Save Period'],1,0))[0]
        x=x[indexes]
        y=values[indexes,1]
        return x,y
        
    def run_stop_event_detector(self):
        '''
        Stop event: animal motion is below threshold for a certain amount of time. (Also a run event should happen right before that)
        Run event: animal was running for a certain amount of time
        Event time is when the condition is met
        
        At stim stop protocol there should be some tolerance between the run and stop event times
        Event structure:
        name:
        acknowledged:
        time:
        previous event: name, time
        '''
        #consider speed values since the last event        
        t=self.speed_values[:,0]
        spd=self.speed_values[:,1]
        if len(self.events)>0:
            indexes=numpy.nonzero(numpy.where(t>self.events[-1]['t'],1,0))[0]
            t=t[indexes]
            spd=spd[indexes]
        if t.shape[0]==0:
            return
        ismoving=numpy.where(spd>self.parameters['Move Threshold'],True,False)
        if hasattr(self, 'stop_time'):
            stop_indexes=numpy.nonzero(numpy.where(t>t.max()-self.stop_time,1,0))[0]
        else:
            stop_indexes=numpy.empty(0)            
        if hasattr(self, 'run_time'):
            running_indexes=numpy.nonzero(numpy.where(t>t.max()-self.run_time,1,0))[0]
        else:
            running_indexes=numpy.empty(0)
        last_event_time=self.events[-1]['t'] if len(self.events)>0 else 0#checking time elapsed since last event ensures that one event is counted once
        if last_event_time<t[0]:#When new session is started, earlier events are ignored as last event time
            last_event_time=t[0]
        if stop_indexes.shape[0]>0 and not ismoving[stop_indexes].any() and t[-1] - last_event_time> self.stop_time:
            event={}
            event['type']='stop'
            event['ack']=False
            event['t']=t[-1]
            self.events.append(event)
        elif running_indexes.shape[0]>0 and ismoving[running_indexes].sum()>ismoving[running_indexes].shape[0]*1e-2*self.parameters['Run Threshold']\
            and t[-1] - last_event_time> self.run_time:
            event={}
            event['type']='run'
            event['ack']=False
            event['t']=t[-1]
            self.events.append(event)
            
    def start_session(self):
        if self.session_ongoing:
            return
        self.reset_data()
        rootfolder=os.path.join(self.datafolder, self.current_animal)
        animal_file=os.path.join(rootfolder, 'animal_' + self.current_animal+'.hdf5')
        if not os.path.exists(rootfolder):
            self.notify('Warning', 'Animal folder does not exists')
            return
        if not os.path.exists(animal_file):
            self.notify('Warning', 'Animal file does not exists')
            return
        today=utils.timestamp2ymd(time.time(),'')
        last_entry_timestamp=utils.timestamp2ymd(self.weight[-1,0],'')
        if last_entry_timestamp<today and not self.ask4confirmation('No animal weight was added today. Do you want to continue?'):
            return
        self.recording_folder=os.path.join(rootfolder, today)
        if not os.path.exists(self.recording_folder):
            os.mkdir(self.recording_folder)
        self.show_day_success_rate(self.recording_folder)
        self.current_protocol = self.parameters['Protocol']
        #TODO give warning if suggested protocol is different
        self.protocol=getattr(sys.modules[__name__], self.current_protocol)(self)
        self.to_gui.put({'update_protocol_description':'Current protocol: '+ self.current_protocol+'\r\n'+self.protocol.__doc__+'\r\n'+str(object_parameters2dict(self.protocol))})
        self.session_ongoing=True
        self.start_recording()
        self.to_gui.put({'set_recording_state': 'recording'})
        self.to_gui.put({'switch2_event_plot': ''})
        logging.info('Session started using {0} protocol'.format(self.current_protocol))
        self.enable_speed_update=True
        
    def run_protocol(self):
        if self.session_ongoing:
            self.protocol.update()
            if int(time.time()*10)%10==0:
                stat=self.protocol.stat()
                if hasattr(stat,'has_key'):
                    stat['Success Rate']='{0:0.0f} %'.format(100*stat['Success Rate'])
                    self.to_gui.put({'statusbar':stat})

    def start_recording(self):
        self.recording_started_state={}
        for vn in self.varnames:
            if not hasattr(self, vn):
                self.recording_started_state[vn]=0
            else:
                var=getattr(self, vn)
                if isinstance(var,list):
                    self.recording_started_state[vn]=len(var)
                elif hasattr(var,'shape'):
                    self.recording_started_state[vn]=var.shape[0]
        self.id=int(time.time())
        self.filename=os.path.join(self.recording_folder, 'data_{0}_{1}'.format(self.current_protocol.replace(' ', '_'), self.id))
        videofilename=self.filename+'.avi'
        self.filename+='.hdf5'
        self.to_gui.put({'set_events_title': os.path.basename(self.filename)})
        self.start_video_recording(videofilename)
        
    def finish_recording(self):
        self.stop_video_recording()
        logging.info('Recorded {0:.0f} s'.format(self.speed_values[-1,0]-self.speed_values[self.recording_started_state['speed_values'],0]))
        self.save2file()
        
    def periodic_save(self):
        if self.parameters['Enable Periodic Save'] and self.session_ongoing and self.speed_values.shape[0]>self.recording_started_state['speed_values'] and self.speed_values[-1,0]-self.speed_values[self.recording_started_state['speed_values'],0]>self.parameters['Save Period']:
            self.finish_recording()
            self.start_recording()
        
    def stop_session(self):
        if not self.session_ongoing:
            return
        self.finish_recording()
        self.session_ongoing=False
        self.to_gui.put({'set_recording_state': 'idle'})
        logging.info('Session ended')
        
    def save2file(self):
        self.datafile=hdf5io.Hdf5io(self.filename)
        self.datafile.stat=self.protocol.stat()
        for nn in ['machine_config', 'protocol']:
            var=getattr(self, nn)
            setattr(self.datafile, nn, object_parameters2dict(var))
            #setattr(self.datafile, nn, dict([(vn,getattr(var, vn)) for vn in dir(var) if vn.isupper()] ))
        for vn in self.varnames:
            values=copy.deepcopy(getattr(self, vn))[self.recording_started_state[vn]:]
            setattr(self.datafile, vn, values)
        self.datafile.protocol_name=self.protocol.__class__.__name__#This for sure the correct value
        nodes=['machine_config', 'protocol', 'protocol_name', 'stat']
        nodes.extend(self.varnames)
        self.datafile.save(nodes)
        self.datafile.close()
        logging.info('Data saved to {0}'.format(self.filename))
        self.show_day_success_rate(self.filename)
        
    def open_file(self, filename):
        if self.session_ongoing:
            self.notify('Warning', 'Stop recording before opening a file')
            return
        self.filename=filename
        self.datafile_opened=time.time()
        self.datafile=hdf5io.Hdf5io(self.filename)
        for vn in self.varnames:
            self.datafile.load(vn)
            setattr(self, vn, getattr(self.datafile, vn))
        stat=self.datafile.findvar('stat')
        self.datafile.close()
        self.enable_speed_update=False
        self.to_gui.put({'set_live_state': 0})
        self.to_gui.put({'set_events_title': os.path.basename(filename)})
        if hasattr(stat,'has_key'):
            stat['Success Rate']='{0:0.0f} %'.format(100*stat['Success Rate'])
            self.to_gui.put({'statusbar':stat})
            
    def show_animal_statistics(self):
        if self.session_ongoing: return
        logging.info('Generating plots, please wait')
        current_animal_folder=os.path.join(self.datafolder, self.current_animal)
        day_folders=[d for d in fileop.listdir_fullpath(current_animal_folder) if os.path.isdir(d)]
        day_folders.sort()
        self.animal_stat={}
        for d in day_folders:
            dayd=[self.read_file_summary(f) for f in fileop.listdir_fullpath(d) if os.path.splitext(f)[1]=='.hdf5']
            protocols=list(set([di['protocol'] for di in dayd if di.has_key('protocol')]))
            day_item={}
            for protocol in protocols:
                day_item[protocol]=numpy.array([[di['t'], di['Success Rate']] for di in dayd if di.has_key('t') and di.has_key('Success Rate') and di['protocol']==protocol]).T
            self.animal_stat[os.path.basename(d)]=day_item
        #group days
        max_plots_per_page=6
        npages=int(numpy.ceil(len(self.animal_stat.keys())/float(max_plots_per_page)))
        days=self.animal_stat.keys()
        days.sort()
        pages=[days[pi*max_plots_per_page:(pi+1)*max_plots_per_page] for pi in range(npages)]
        self.animal_stat_per_page=[]
        for page in pages:
            self.animal_stat_per_page.append(dict([(d, self.animal_stat[d]) for d in page]))
        self.to_gui.put({'show_animal_statistics':[self.animal_stat_per_page, self.current_animal]})
        
            
    def show_animal_success_rate(self):
        current_animal_folder=os.path.join(self.datafolder, self.current_animal)
        x=[]
        y=[]
        day_folders=[d for d in fileop.listdir_fullpath(current_animal_folder) if os.path.isdir(d)]
        day_folders.sort()
        top_protocols=[]
        for day_folder in day_folders:
            success_rate,top_protocol=self.day_summary(day_folder)
            top_protocols.append(top_protocol)
            y.append(success_rate)
            x.append(utils.datestring2timestamp(os.path.basename(day_folder),format="%Y%m%d"))
        x=numpy.array(x)
        if x.shape[0]==0: return
        x-=x.max()
        x/=86400
        self.animal_success_rate=numpy.array([x,y])
        y=numpy.array(y)
        self.to_gui.put({'update_success_rate_plot':{'x':[x], 'y':[y*100], 'trace_names': ['success_rate']}})
        self.to_gui.put({'set_success_rate_title': self.current_animal})
        logging.info('Top protocols per day\r\nDay\tTop protocol\r\n' + '\r\n'.join(['{0}\t{1}'.format(x[i], top_protocols[i]) for i in range(len(top_protocols))]))
        
        
    def day_summary(self, folder):
        summary = [self.read_file_summary(f) for f in [os.path.join(folder,fn) for fn in os.listdir(folder) if os.path.splitext(fn)[1]=='.hdf5']]
        try:
            all_recording_time = sum([s['duration'] for s in summary])
            weighted_success_rates=[s['duration']/all_recording_time*s['Success Rate'] for s in summary]
            success_rate=sum(weighted_success_rates)
            #Most frequent protocol:
            protocols=[s['protocol'] for s in summary]
            protocol_names=list(set(protocols))
            top_protocol = protocol_names[numpy.array([len([p for p in protocols if pn in p]) for pn in protocol_names]).argmax()]
        except:
            success_rate=0
            top_protocol=''
        return success_rate,top_protocol
            
    def show_day_success_rate(self,folder):
        '''
        
        
        '''
        if os.path.isdir(folder):
            self.summary = [self.read_file_summary(f) for f in [os.path.join(folder,fn) for fn in os.listdir(folder) if os.path.splitext(fn)[1]=='.hdf5']]
        elif hasattr(self, 'summary'):
            self.summary.append(self.read_file_summary(folder))
        else:
            return
        #Sort
        ts=[item['t'] for item in self.summary if item.has_key('t')]
        ts.sort()
        sorted=[]
        for ti in ts:
            sorted.append([item for item in self.summary if item.has_key('t') and ti==item['t']][0])
        self.summary=sorted
        xi=[]
        yi=[]
        for si in self.summary:
            xi.append(si['t'])
            yi.append(si['Success Rate'])
        xp=[]
        xp=numpy.array([self.summary[i]['t'] for i in range(len(self.summary)) if i>0 and self.summary[i]['protocol']!=self.summary[i-1]['protocol']])
        protocol_order=[self.summary[i]['protocol'] for i in range(len(self.summary)) if i>0 and self.summary[i]['protocol']!=self.summary[i-1]['protocol']]
        if len(self.summary)>0:
            protocol_order.insert(0,self.summary[0]['protocol'])
        if len(xi)>0:
            if xp.shape[0]>0:
                if xp.shape[0]%2==1:
                    xp=numpy.append(xp, xi[-1])
                xp-=xi[0]
            xi-=xi[0]
            x=[numpy.array(xi)]
            y=[numpy.array(yi)*100]
            trace_names=['success_rate']
            self.to_gui.put({'update_success_rate_plot':{'x':x, 'y':y, 'trace_names': trace_names, 'vertical_lines': xp}})
            self.to_gui.put({'set_success_rate_title': os.path.basename(folder)})
            logging.info('Protocol order: {0}'.format(', '.join(protocol_order)))
            
    def read_file_summary(self,filename):
        '''
        Read time of recording, duration, protocol, success rate and other stat parameters
        '''
        h=hdf5io.Hdf5io(filename)
        for vn in ['protocol_name', 'speed_values', 'stat']:
            h.load(vn)
            if not hasattr(h, vn):
                h.close()
                return {}
        duration=h.speed_values[-1,0]-h.speed_values[0,0]
        t=h.speed_values[0,0]
        data_item ={'t':t, 'duration':duration, 'protocol': h.protocol_name}
        data_item.update(h.stat)
        h.close()
        return data_item
        
    def run(self):
        logging.info('Engine started')
        while True:
            try:
                self.last_run = time.time()#helps determining whether the engine still runs
                if not self.from_gui.empty():
                    msg = self.from_gui.get()
                    if msg == 'terminate':
                        break
                    if msg.has_key('function'):#Functions are simply forwarded
                        #Format: {'function: function name, 'args': [], 'kwargs': {}}
                        logging.info(msg)
                        getattr(self, msg['function'])(*msg['args'])
                self.update_video_recorder()
                if hasattr(self, 'parameters'):#At startup parameters may not be immediately available
                    #logging.info('alive')
                    if self.enable_speed_update:
                        self.run_stop_event_detector()
                        self.update_speed_values()
                    self.update_plot()
                    self.periodic_save()
                    self.run_protocol()
            except:
                logging.error(traceback.format_exc())
                self.save_context()
            time.sleep(40e-3)
        self.close()
        
    def close(self):
        if self.session_ongoing:
            self.stop_session()
        self.close_video_recorder()
        self.speed_reader_q.put('terminate')
        self.speed_reader.join()
        self.save_context()
        logging.info('Engine terminated')
        
class Behavioral(gui.SimpleAppWindow):
    '''
    command line parameters expected:
    '''
    def __init__(self):
        #Figure out machine config
        self.machine_config = utils.fetch_classes('visexpman.users.common', classname = sys.argv[1], required_ancestors = visexpman.engine.vision_experiment.configuration.BehavioralConfig,direct = False)[0][1]()
        self.logfile=fileop.get_log_filename(self.machine_config)
        logging.basicConfig(filename= self.logfile,
                    format='%(asctime)s %(levelname)s\t%(message)s',
                    level=logging.INFO)
        self.engine=BehavioralEngine(self.machine_config)
        self.engine.start()
        self.to_engine=self.engine.from_gui
        self.from_engine=self.engine.to_gui
        self.maximized=False
        gui.SimpleAppWindow.__init__(self)
        
    def init_gui(self):
        self.setWindowTitle('Behavioral Experiment Control')
        self.setWindowIcon(gui.get_icon('behav'))
        gui.set_win_icon()
        self.setGeometry(self.machine_config.SCREEN_OFFSET[0],self.machine_config.SCREEN_OFFSET[1],self.machine_config.SCREEN_SIZE[0],self.machine_config.SCREEN_SIZE[1])
        self.debugw.setFixedHeight(self.machine_config.BOTTOM_WIDGET_HEIGHT)
        self.debugw.setMaximumWidth(650)
        self.cw=CWidget(self)#Creating the central widget which contains the image, the plot and the control widgets
        self.setCentralWidget(self.cw)#Setting it as a central widget
        self.params_config=[
                            {'name': 'General', 'type': 'group', 'expanded' : True, 'children': [
                                {'name': 'Protocol', 'type': 'list', 'values': get_protocol_names(),'value':''},
                                {'name': 'Save Period', 'type': 'float', 'value': 100.0,'siPrefix': True, 'suffix': 's'},
                                {'name': 'Enable Periodic Save', 'type': 'bool', 'value': True},
                                {'name': 'Move Threshold', 'type': 'float', 'value': 1,'suffix': 'm/s'},
                                {'name': 'Run Threshold', 'type': 'float', 'value': 50.0, 'suffix': '%'},
                                {'name': 'Laser Intensity', 'type': 'float', 'value': 1.0,'siPrefix': True, 'suffix': 'V'},
                                {'name': 'Stimulus Pulse Duration', 'type': 'float', 'value': 1.0,'siPrefix': True, 'suffix': 's'},
                                {'name': 'Led Stim Voltage', 'type': 'float', 'value': 1.0,'siPrefix': True, 'suffix': 'V'},
                            ]},
                            {'name': 'Show...', 'type': 'group', 'expanded' : True, 'children': [
                                {'name': 'Run Events Trace', 'type': 'bool', 'value': True},
                                {'name': 'Stop Events Trace', 'type': 'bool', 'value': True},
                                {'name': 'Reward Trace', 'type': 'bool', 'value': False},
                                {'name': 'Airpuff Trace', 'type': 'bool', 'value': False},
                                {'name': 'Stimulus Trace', 'type': 'bool', 'value': False},
                                {'name': 'Force Run Trace', 'type': 'bool', 'value': False},
                                ]},
                            {'name': 'Advanced', 'type': 'group', 'expanded' : True, 'children': [
                                {'name': 'Water Open Time', 'type': 'float', 'value': 10e-3,'siPrefix': True, 'suffix': 's'},
                                {'name': 'Air Puff Duration', 'type': 'float', 'value': 10e-3,'siPrefix': True, 'suffix': 's'},
                                ]},
                    ]
        if hasattr(self.engine, 'parameters'):
            for k,v in self.engine.parameters.items():
                for p in self.params_config:
                    for pi in p['children']:
                        if pi['name']==k:
                            pi['value']=v
                            break
        self.paramw = gui.ParameterTable(self, self.params_config)
        self.paramw .setFixedWidth(400)
        self.paramw.params.sigTreeStateChanged.connect(self.parameter_changed)
        self.parameter_changed()
        self.add_dockwidget(self.paramw, 'Parameters', QtCore.Qt.RightDockWidgetArea, QtCore.Qt.RightDockWidgetArea)
        
        self.plotnames=['events', 'animal_weight', 'success_rate']
        self.plots=gui.TabbedPlots(self,self.plotnames)
        self.plots.animal_weight.plot.setLabels(left='weight [g]')
        self.plots.events.plot.setLabels(left='speed [m/s]', bottom='times [s]')
        self.plots.success_rate.setToolTip('''Displays success rate of days or a specific day depending on what is selected in Files tab.
        A day summary is shown if a specific recording day is selected. If animal is selected, a summary for each day is plotted
        ''')
        self.plots.success_rate.plot.setLabels(left='%')
        self.plots.tab.setMinimumWidth(700)
        self.plots.tab.setFixedHeight(self.machine_config.BOTTOM_WIDGET_HEIGHT)
        for pn in self.plotnames:
            getattr(self.plots, pn).setMinimumWidth(self.plots.tab.width()-50)
        self.add_dockwidget(self.plots, 'Plots', QtCore.Qt.BottomDockWidgetArea, QtCore.Qt.BottomDockWidgetArea)
        
        toolbar_buttons = ['record', 'stop', 'select_data_folder','add_animal', 'add_animal_weight', 'remove_last_animal_weight', 'show_animal_statistics', 'exit']
        self.toolbar = gui.ToolBar(self, toolbar_buttons)
        self.addToolBar(self.toolbar)
        self.statusbar=self.statusBar()
        self.update_statusbar()
        
        self.cq_timer=QtCore.QTimer()
        self.cq_timer.start(20)#ms
        self.connect(self.cq_timer, QtCore.SIGNAL('timeout()'), self.check_queue)
        
    def parameter_changed(self):
        self.engine.parameters=self.paramw.get_parameter_tree(return_dict=True)
        
    def check_queue(self):
        if not self.from_engine.empty():
            if self.from_engine.qsize()>5:
                while not self.from_engine.empty():
                    self.process_msg(self.from_engine.get(), skip_main_image_display=True)
            else:
                self.process_msg(self.from_engine.get())
            
    def process_msg(self,msg,skip_main_image_display=False):
        if msg.has_key('notify'):
            self.notify(msg['notify']['title'], msg['notify']['msg'])
        elif msg.has_key('statusbar'):
            self.update_statusbar(msg=msg['statusbar'])
        elif msg.has_key('update_weight_history'):
            x=msg['update_weight_history'][:,0]
            x/=86400
            x-=x[-1]
            utils.timestamp2ymd(self.engine.weight[-1,0])
            self.plots.animal_weight.update_curve(x,msg['update_weight_history'][:,1],plotparams={'symbol' : 'o', 'symbolSize': 8, 'symbolBrush' : (0, 0, 0)})
            self.plots.animal_weight.plot.setLabels(bottom='days, {0} = {1}, 0 = {2}'.format(int(numpy.round(x[0])), utils.timestamp2ymd(self.engine.weight[0,0]), utils.timestamp2ymd(self.engine.weight[-1,0])))
        elif msg.has_key('switch2_animal_weight_plot'):
            self.plots.tab.setCurrentIndex(1)
        elif msg.has_key('update_main_image'):
            if not skip_main_image_display:
                self.cw.images.main.set_image(numpy.rot90(msg['update_main_image'],3),alpha=1.0)
        elif msg.has_key('update_events_plot'):
            plotparams=[]
            for tn in msg['update_events_plot']['trace_names']:
                if tn =='speed':
                    plotparams.append({'name': tn, 'pen':(0,0,0)})
                elif tn=='reward':
                    plotparams.append({'name': tn, 'pen':None, 'symbol':'t', 'symbolSize':12, 'symbolBrush': (0,255,0,150)})
                elif tn=='airpuff':
                    plotparams.append({'name': tn, 'pen':None, 'symbol':'t', 'symbolSize':12, 'symbolBrush': (255,0,0,150)})
                elif tn=='stimulus':
                    plotparams.append({'name': tn, 'pen':(0,0,255)})
                elif tn=='run':
                    plotparams.append({'name': tn, 'pen':None, 'symbol':'o', 'symbolSize':10, 'symbolBrush': (128,128,128,150)})
                elif tn=='stop':
                    plotparams.append({'name': tn, 'pen':None, 'symbol':'o', 'symbolSize':10, 'symbolBrush': (0,128,0,150)})
                elif tn=='forcerun':
                    plotparams.append({'name': tn, 'pen':None, 'symbol':'t', 'symbolSize':12, 'symbolBrush': (128,0,0,150)})
            self.plots.events.update_curves(msg['update_events_plot']['x'], msg['update_events_plot']['y'], plotparams=plotparams)
        elif msg.has_key('ask4confirmation'):
            reply = QtGui.QMessageBox.question(self, 'Confirm following action', msg['ask4confirmation'], QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
            self.to_engine.put(reply == QtGui.QMessageBox.Yes)
        elif msg.has_key('set_recording_state'):
            self.cw.set_state(msg['set_recording_state'])
        elif msg.has_key('switch2_event_plot'):
            self.plots.tab.setCurrentIndex(0)
        elif msg.has_key('set_live_state'):
            self.cw.live_speed.input.setCheckState(msg['set_live_state'])
        elif msg.has_key('set_events_title'):
            self.plots.events.plot.setTitle(msg['set_events_title'])
        elif msg.has_key('update_protocol_description'):
            self.cw.state.setToolTip(msg['update_protocol_description'])
        elif msg.has_key('update_success_rate_plot'):
            plotparams=[]
            for tn in msg['update_success_rate_plot']['trace_names']:
                if tn =='success_rate':
                    plotparams.append({'name': tn, 'pen':(0,0,0), 'symbol':'o', 'symbolSize':6, 'symbolBrush': (0,0,0)})
            self.plots.success_rate.update_curves(msg['update_success_rate_plot']['x'], msg['update_success_rate_plot']['y'], plotparams=plotparams)
            if msg['update_success_rate_plot'].has_key('vertical_lines') and msg['update_success_rate_plot']['vertical_lines'].shape[0]>0:
                self.plots.success_rate.add_linear_region(msg['update_success_rate_plot']['vertical_lines'],color=(0,0,0,20))
            else:
                self.plots.success_rate.add_linear_region([])
        elif msg.has_key('set_success_rate_title'):
            self.plots.success_rate.plot.setTitle(msg['set_success_rate_title'])
        elif msg.has_key('show_animal_statistics'):
            if hasattr(self, 'asp'):
                del self.asp
            self.asp = AnimalStatisticsPlots(self, *msg['show_animal_statistics'])
        
    def update_statusbar(self,msg=''):
        '''
        General text display
        '''
        ca=self.engine.current_animal if hasattr(self.engine, 'current_animal') else ''
        txt='Data folder: {0}, Current animal: {1} {2}'. format(self.engine.datafolder,ca,msg)
        self.statusbar.showMessage(txt)
        self.setWindowTitle('Behavioral\t {0}'.format(ca))
        
    def closeEvent(self, e):
        e.accept()
        self.exit_action()
        
    def record_action(self):
        self.to_engine.put({'function': 'start_session','args':[]})
    
    def stop_action(self):
        self.to_engine.put({'function': 'stop_session','args':[]})
        
    def select_data_folder_action(self):
        self.engine.datafolder = self.ask4foldername('Select Data Folder', self.engine.datafolder)
        self.cw.filebrowserw.set_root(self.engine.datafolder)
        self.update_statusbar()
        
    def add_animal_action(self):
        text, ok = QtGui.QInputDialog.getText(self, 'Add Animal', 
            'Enter animal name:')
        if ok:
            self.to_engine.put({'function': 'add_animal','args':[str(text)]})
            
    def add_animal_weight_action(self):
        self.aw=AddAnimalWeightDialog(self.to_engine)
        self.aw.show()
      
    def remove_last_animal_weight_action(self):
        if self.ask4confirmation('Do you want to remove last weight entry?'):
            self.to_engine.put({'function': 'remove_last_animal_weight','args':[]})
            
    def show_animal_statistics_action(self):
        self.to_engine.put({'function': 'show_animal_statistics','args':[]})
        
    def exit_action(self):
        if hasattr(self,'asp'):
            self.asp.close()
        self.to_engine.put('terminate')
        self.engine.join()
        self.close()
        
class CWidget(QtGui.QWidget):
    '''
    The central widget of the user interface which contains the image, the plot and the various controls for starting experiment or adjusting parameters
    '''
    def __init__(self,parent):
        QtGui.QWidget.__init__(self,parent)
        self.setFixedHeight(340)
        self.imagenames=['main', 'eye']
        self.images=gui.TabbedImages(self,self.imagenames)
        ar=float(parent.machine_config.CAMERA_FRAME_WIDTH)/parent.machine_config.CAMERA_FRAME_HEIGHT
        self.images.main.setFixedWidth(280*ar)
        self.images.main.setFixedHeight(280)
        self.main_tab = self.images.tab
        self.filebrowserw=FileBrowserW(self)
        self.main_tab.addTab(self.filebrowserw, 'Files')
        self.lowleveldebugw=LowLevelDebugW(self)
        self.main_tab.addTab(self.lowleveldebugw, 'Advanced')
        self.main_tab.setCurrentIndex(0)
        self.main_tab.setTabPosition(self.main_tab.South)
        self.main_tab.setMinimumHeight(290)
        self.main_tab.setMinimumWidth(700)
        self.live_speed=gui.LabeledCheckBox(self,'Live Speed')
        self.live_speed.input.setCheckState(2)
        self.connect(self.live_speed.input, QtCore.SIGNAL('stateChanged(int)'),  self.live_event_udpate_clicked)
        self.state=QtGui.QPushButton('Idle', parent=self)
        self.state.setMinimumWidth(100)
        self.state.setEnabled(False)
        self.set_state('idle')
        self.l = QtGui.QGridLayout()
        self.l.addWidget(self.main_tab, 0, 0, 2, 4)
        self.l.addWidget(self.state, 2, 0, 1, 1)
        self.l.addWidget(self.live_speed, 2, 1, 1, 1)
        self.setLayout(self.l)
        self.l.setColumnStretch(3,20)
        self.l.setRowStretch(3,20)
        
    def set_state(self,state):
        self.state.setText(state)
        if state=='recording':
            self.state.setStyleSheet('QPushButton {background-color: red; color: black;}')
        elif state=='idle':
            self.state.setStyleSheet('QPushButton {background-color: gray; color: black;}')
        
        
    def live_event_udpate_clicked(self,state):
        self.parent().to_engine.put({'function':'set_speed_update','args':[state==2]})
        
class FileBrowserW(gui.FileTree):
    def __init__(self,parent):
        gui.FileTree.__init__(self,parent, parent.parent().engine.datafolder, ['mat','hdf5', 'avi'])
        self.doubleClicked.connect(self.open_file)
        self.clicked.connect(self.file_selected)
        
    def file_selected(self,index):
        self.selected_filename = gui.index2filename(index)
        datafolder=self.parent.parent().engine.datafolder
        if os.path.isdir(self.selected_filename) and os.path.dirname(self.selected_filename).lower()==datafolder.lower():
            #Animal folder selected
            self.parent.parent().engine.current_animal=os.path.basename(self.selected_filename)
            logging.info('Animal selected: {0}'.format(self.parent.parent().engine.current_animal))
            self.parent.parent().update_statusbar()
            self.parent.parent().to_engine.put({'function':'load_animal_file','args':[]})
            self.parent.parent().to_engine.put({'function':'show_animal_success_rate','args':[]})
        elif os.path.isdir(self.selected_filename) and hasattr(self.parent.parent().engine, 'current_animal') and os.path.dirname(self.selected_filename)==os.path.join(self.parent.parent().engine.datafolder, self.parent.parent().engine.current_animal):
            self.parent.parent().to_engine.put({'function':'show_day_success_rate','args':[self.selected_filename]})
            
    def open_file(self,index):
        self.double_clicked_filename = gui.index2filename(index)
        if os.path.splitext(self.double_clicked_filename)[1]=='.hdf5' and os.path.basename(self.double_clicked_filename)[:4]=='data':
            self.parent.parent().to_engine.put({'function':'open_file','args':[self.double_clicked_filename]})
            
class LowLevelDebugW(QtGui.QWidget):
    '''
    
    '''
    def __init__(self,parent):
        QtGui.QWidget.__init__(self,parent)
        self.parent=parent
        valve_names=['water', 'air']
        self.valves={}
        for vn in valve_names:
            self.valves[vn]=gui.LabeledCheckBox(self,'Open {0} Valve'.format(vn.capitalize()))
            self.valves[vn].setToolTip('When checked, the valve is open')
        self.connect(self.valves['water'].input, QtCore.SIGNAL('stateChanged(int)'),  self.water_valve_clicked)
        self.connect(self.valves['air'].input, QtCore.SIGNAL('stateChanged(int)'),  self.air_valve_clicked)
        self.airpuff=QtGui.QPushButton('Air Puff', parent=self)
        self.connect(self.airpuff, QtCore.SIGNAL('clicked()'), self.airpuff_clicked)
        self.reward=QtGui.QPushButton('Reward', parent=self)
        self.connect(self.reward, QtCore.SIGNAL('clicked()'), self.reward_clicked)
        self.stimulate=QtGui.QPushButton('Stimulate', parent=self)
        self.connect(self.stimulate, QtCore.SIGNAL('clicked()'), self.stimulate_clicked)
        self.forcerun=QtGui.QPushButton('Force Run', parent=self)
        self.connect(self.forcerun, QtCore.SIGNAL('clicked()'), self.forcerun_clicked)
        self.l = QtGui.QGridLayout()
        [self.l.addWidget(self.valves.values()[i], 0, i, 1, 1) for i in range(len(self.valves.values()))]
        self.l.addWidget(self.reward, 1, 0, 1, 1)
        self.l.addWidget(self.airpuff, 1, 1, 1, 1)
        self.l.addWidget(self.stimulate, 2, 0, 1, 1)
        self.l.addWidget(self.forcerun, 2, 1, 1, 1)
        self.setLayout(self.l)
        self.l.setColumnStretch(2,20)
        self.l.setRowStretch(3,20)
        
    def forcerun_clicked(self):
        self.parent.parent().to_engine.put({'function':'forcerun','args':[1]})
        
    def airpuff_clicked(self):
        self.parent.parent().to_engine.put({'function':'airpuff','args':[]})
        
    def reward_clicked(self):
        self.parent.parent().to_engine.put({'function':'reward','args':[]})
    
    def stimulate_clicked(self):
        self.parent.parent().to_engine.put({'function':'stimulate','args':[]})
    
    def air_valve_clicked(self,state):
        self.parent.parent().to_engine.put({'function':'set_valve','args':['air', state==2]})
        
    def water_valve_clicked(self,state):
        self.parent.parent().to_engine.put({'function':'set_valve','args':['water', state==2]})
        
class AddAnimalWeightDialog(QtGui.QWidget):
    '''
    Dialog window for date and animal weight
    '''
    def __init__(self,q):
        self.q=q
        QtGui.QWidget.__init__(self)
        self.move(200,200)
        date_format = QtCore.QString('yyyy-MM-dd')
        self.date = QtGui.QDateTimeEdit(self)
        self.date.setDisplayFormat(date_format)
        now = time.localtime()
        self.date.setDateTime(QtCore.QDateTime(QtCore.QDate(now.tm_year, now.tm_mon, now.tm_mday), QtCore.QTime(now.tm_hour, now.tm_min)))
        self.date.setFixedWidth(150)
        self.weight_input=QtGui.QLineEdit(self)
        self.weight_input.setFixedWidth(100)
        self.weight_input_l = QtGui.QLabel('Animal weight [g]', self)
        self.weight_input_l.setFixedWidth(120)
        self.ok=QtGui.QPushButton('OK' ,parent=self)
        self.connect(self.ok, QtCore.SIGNAL('clicked()'), self.ok_clicked)
        self.cancel=QtGui.QPushButton('Cancel' ,parent=self)
        self.connect(self.cancel, QtCore.SIGNAL('clicked()'), self.cancel_clicked)
        
        self.l = QtGui.QGridLayout()
        self.l.addWidget(self.date, 0, 0, 1, 1)
        self.l.addWidget(self.weight_input_l, 0, 1, 1, 1)
        self.l.addWidget(self.weight_input, 0, 2, 1, 1)
        self.l.addWidget(self.ok, 1, 0, 1, 1)
        self.l.addWidget(self.cancel, 1, 1, 1, 1)
        self.l.setColumnStretch(3,20)
        self.l.setRowStretch(0,1)
        self.l.setRowStretch(1,1)
        self.setLayout(self.l)
        
    def ok_clicked(self):
        weight=float(str(self.weight_input.text()))
        date=self.date.date()
        datets=time.mktime(time.struct_time((date.year(),date.month(),date.day(),0,0,0,0,0,-1)))
        self.q.put({'function': 'add_animal_weight','args':[datets,weight]})
        self.close()
        
    def cancel_clicked(self):
        self.close()
        
class AnimalStatisticsPlots(QtGui.QTabWidget):
    def __init__(self, parent, data, animal_name):
        QtGui.QTabWidget.__init__(self)
        self.setWindowIcon(gui.get_icon('behav'))
        gui.set_win_icon()
        self.machine_config=parent.machine_config
        self.setGeometry(self.machine_config.SCREEN_OFFSET[0],self.machine_config.SCREEN_OFFSET[1],parent.machine_config.SCREEN_SIZE[0],parent.machine_config.SCREEN_SIZE[1])
        self.setWindowTitle('Summary of '+animal_name)
        self.setTabPosition(self.North)
        self.pages=[]
        for i in range(len(data)):
            self.pages.append(PlotPage(data[i],self))
            self.addTab(self.pages[-1],str(i))
        self.show()
        
class PlotPage(QtGui.QWidget):
    '''
    Dialog window for date and animal weight
    '''
    def __init__(self,data,parent):
        QtGui.QWidget.__init__(self)
        self.tp=[]
        self.l = QtGui.QGridLayout()
        days=data.keys()
        days.sort()
        ct=0
        for d in days:
            self.tp.append(gui.TabbedPlots(self, data[d].keys()))
            for pn in data[d].keys():
                ref=getattr(self.tp[-1],pn)
                ref.plot.setTitle(d)
                t=data[d][pn][0]
                t=numpy.array(map(utils.timestamp2secondsofday,t.tolist()))/3600.0
                minutes=(t-numpy.floor(t))*0.6
                t-=t-numpy.floor(t)
                t+=minutes
                ref.update_curve(t,data[d][pn][1]*100,plotparams={'symbol':'o', 'symbolSize':6, 'symbolBrush': (0,0,0)})
                ref.plot.setLabels(left='success rate [%]', bottom='time [hour.minute]')
            self.tp[-1].tab.setFixedWidth(parent.machine_config.SCREEN_SIZE[0]/3-50)
            self.tp[-1].tab.setFixedHeight(parent.machine_config.SCREEN_SIZE[1]/2-50)
            self.l.addWidget(self.tp[-1], ct/3, ct%3, 1, 1)
            ct+=1
        self.setLayout(self.l)
        

class TestBehavEngine(unittest.TestCase):
    
    def setUp(self):
        self.machine_config = utils.fetch_classes('visexpman.users.common', classname = 'BehavioralSetup', required_ancestors = visexpman.engine.vision_experiment.configuration.BehavioralConfig,direct = False)[0][1]()
        if not hasattr(self, 'log_initialized'):
            self.logfile=fileop.generate_filename('/tmp/behav_engine_test.txt')
            logging.basicConfig(filename= self.logfile,
                        format='%(asctime)s %(levelname)s\t%(message)s',
                        level=logging.INFO)
            self.log_initialized=True
        self.context_filename = fileop.get_context_filename(self.machine_config,'npy')
        if os.path.exists(self.context_filename):
            os.remove(self.context_filename)
        
    def test_01_add_remove_weight(self):
        self.animal_name='ta001'
        self.engine=BehavioralEngine(self.machine_config)
        af=os.path.join(self.engine.datafolder,self.animal_name)
        if os.path.exists(af):
            shutil.rmtree(af)
        self.engine.add_animal(self.animal_name)
        for i in range(10):
            self.engine.add_animal_weight(time.time()+i*86400,i)
        for i in range(11):
            self.engine.remove_last_animal_weight()
        for i in range(5):
            self.engine.add_animal_weight(time.time()+i*86400,i)
        self.assertEqual(5,self.engine.weight.shape[0])
        self.engine.close()
        
    def test_02_video_recorder(self):
        frame_folder=fileop.generate_foldername('/tmp/frames')
        capture_duration=3
        videofilename1='/tmp/{0}.avi'.format(int(time.time()))
        self.engine=BehavioralEngine(self.machine_config)
        t0=time.time()
        while True:
            self.engine.update_video_recorder()
            if time.time()-t0>capture_duration:
                break
        self.engine.start_video_recording(videofilename1)
        t0=time.time()
        while True:
            self.engine.update_video_recorder()
            if time.time()-t0>capture_duration:
                break
        videofilename2='/tmp/{0}.avi'.format(int(time.time()))
        self.engine.start_video_recording(videofilename2)
        self.assertEqual(os.path.exists(videofilename1), True)
        t0=time.time()
        while True:
            self.engine.update_video_recorder()
            if time.time()-t0>capture_duration:
                break
        self.engine.stop_video_recording()
        self.engine.close()
        self.assertEqual(os.path.exists(videofilename2), True)
        
    def test_03_speed_emulator(self):
        q=multiprocessing.Queue()
        rectime=3
        tsr=TreadmillSpeedReader(q,self.machine_config,emulate_speed=True)
        tsr.start()
        time.sleep(rectime)
        q.put('terminate')
        tsr.join()
        samples=[]
        while not tsr.speed_q.empty():
            samples.append(tsr.speed_q.get())
        self.assertEqual(numpy.array(samples).shape[0], rectime/self.machine_config.TREADMILL_SPEED_UPDATE_RATE-1)
        
    def test_04_animal_summary(self):
        self.engine=BehavioralEngine(self.machine_config)
        self.engine.session_ongoing=False
        self.engine.current_animal='eger1'
        self.engine.datafolder='/tmp/animal'
        self.engine.show_animal_statistics()
        self.engine.animal_stat_per_page
        self.engine.close()
        

if __name__ == '__main__':
    if len(sys.argv)>1:
        gui = Behavioral()
    else:
        unittest.main()
