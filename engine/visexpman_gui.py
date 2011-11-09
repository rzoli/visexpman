import time
import socket
import sys
import PyQt4.Qt as Qt
import PyQt4.QtGui as QtGui
import PyQt4.QtCore as QtCore
import visexpman.engine.generic.utils as utils
import visexpman.engine.generic.configuration as configuration
import visexpman.engine.hardware_interface.network_interface as network_interface
import Queue
import os.path

class Gui(Qt.QMainWindow):
    def __init__(self, config, command_queue, response_queue):
        self.config = config
        self.command_queue = command_queue
        self.response_queue = response_queue
        self.default_parameters()
        #=== Init GUI ===
        Qt.QMainWindow.__init__(self)
        self.setWindowTitle('Vision Experiment Manager GUI')
        self.resize(800, 600)
        self.move(100,  100)
        self.create_user_interface()
        self.show()
        
    def create_user_interface(self):        
        panel_size = utils.cr((180, 40))
        date_format = QtCore.QString('dd-mm-yyyy')        
        mouse_birth_date = {'size' : panel_size,  'position' : utils.cr((0,  100))}
        gcamp_injection_date = {'size' : panel_size,  'position' : utils.cr((2.1*panel_size['col'],  100))}
        ear_punch_l = {'size' : panel_size,  'position' : utils.cr((0,  150))}
        ear_punch_r = {'size' : panel_size,  'position' : utils.cr((2.1*panel_size['col'],  150))}
        ear_punch_items = QtCore.QStringList(['0',  '1',  '2'])       
        anesthesia_protocol = {'size' : panel_size,  'position' : utils.cr((0,  200))}
        anesthesia_protocol_items = QtCore.QStringList(['isoflCP 1.0', 'isoflCP 0.5', 'isoflCP 1.5'])
        mouse_strain = {'size' : panel_size,  'position' : utils.cr((2.1*panel_size['col'],  200))}
        mouse_strain_items = QtCore.QStringList(['bl6', 'chat', 'chatdtr'])
        generate_id = {'size' : panel_size,  'position' : utils.cr((0,  250))}
        load_previous_experiment = {'size' : panel_size,  'position' : utils.cr((2.1*panel_size['col'],  250))}
        id = {'size' : utils.cr((640, 40)),  'position' : utils.cr((0,  300))}
        
        acquire_camera_image = {'size' : panel_size,  'position' : utils.cr((0,  350))}
        acquire_z_stack = {'size' : panel_size,  'position' : utils.cr((panel_size['col'],  350))}
        two_photon_recording = {'size' : panel_size,  'position' : utils.cr((2*panel_size['col'],  350))}
        log = {'size' : utils.cr((640, 40)),  'position' : utils.cr((0,  400))}
        update = {'size' : panel_size,  'position' : utils.cr((0,  450))}
        
        #=== Generating experiment id ===

        self.generate_id_button = QtGui.QPushButton('Generate ID',  self)
        self.generate_id_button.resize(generate_id['size']['col'],  generate_id['size']['row'])
        self.generate_id_button.move(generate_id['position']['col'],  generate_id['position']['row'])
        self.connect(self.generate_id_button, QtCore.SIGNAL('clicked()'),  self.generate_id)
        
        self.load_previous_experiment_button = QtGui.QPushButton('Load previous experiment',  self)
        self.load_previous_experiment_button.resize(load_previous_experiment['size']['col'],  load_previous_experiment['size']['row'])
        self.load_previous_experiment_button.move(load_previous_experiment['position']['col'],  load_previous_experiment['position']['row'])
        self.connect(self.load_previous_experiment_button, QtCore.SIGNAL('clicked()'),  self.load_previous_experiment)
        
        self.mouse_birth_date = QtGui.QDateEdit(self)
        self.mouse_birth_date.setDisplayFormat(date_format)
        self.mouse_birth_date.resize(mouse_birth_date['size']['col'],  mouse_birth_date['size']['row'])
        self.mouse_birth_date.move(mouse_birth_date['position']['col'] + mouse_birth_date['size']['col'],  mouse_birth_date['position']['row'])
        self.mouse_birth_date_label = QtGui.QLabel('Mouse birth date',  self)
        self.mouse_birth_date_label.resize(mouse_birth_date['size']['col'],  mouse_birth_date['size']['row'])
        self.mouse_birth_date_label.move(mouse_birth_date['position']['col'],  mouse_birth_date['position']['row'])
        
        self.gcamp_injection_date = QtGui.QDateEdit(self)
        self.gcamp_injection_date.setDisplayFormat(date_format)
        self.gcamp_injection_date.resize(gcamp_injection_date['size']['col'],  gcamp_injection_date['size']['row'])
        self.gcamp_injection_date.move(gcamp_injection_date['position']['col'] + gcamp_injection_date['size']['col'],  gcamp_injection_date['position']['row'])
        self.gcamp_injection_date_label = QtGui.QLabel('GCAMP injection date',  self)
        self.gcamp_injection_date_label.resize(gcamp_injection_date['size']['col'],  gcamp_injection_date['size']['row'])
        self.gcamp_injection_date_label.move(gcamp_injection_date['position']['col'],  gcamp_injection_date['position']['row'])
        
        self.ear_punch_l = QtGui.QComboBox(self)
        self.ear_punch_l.resize(ear_punch_l['size']['col'],  ear_punch_l['size']['row'])
        self.ear_punch_l.move(ear_punch_l['position']['col'] + ear_punch_l['size']['col'],  ear_punch_l['position']['row'])
        self.ear_punch_l_label = QtGui.QLabel('Ear punch L',  self)
        self.ear_punch_l_label.resize(ear_punch_l['size']['col'],  ear_punch_l['size']['row'])
        self.ear_punch_l_label.move(ear_punch_l['position']['col'],  ear_punch_l['position']['row'])
        self.ear_punch_l.addItems(ear_punch_items)
        
        self.ear_punch_r = QtGui.QComboBox(self)
        self.ear_punch_r.resize(ear_punch_r['size']['col'],  ear_punch_r['size']['row'])
        self.ear_punch_r.move(ear_punch_r['position']['col'] + ear_punch_r['size']['col'],  ear_punch_r['position']['row'])
        self.ear_punch_r_label = QtGui.QLabel('Ear punch R',  self)
        self.ear_punch_r_label.resize(ear_punch_r['size']['col'],  ear_punch_r['size']['row'])
        self.ear_punch_r_label.move(ear_punch_r['position']['col'],  ear_punch_r['position']['row'])
        self.ear_punch_r.addItems(ear_punch_items)
        
        self.anesthesia_protocol = QtGui.QComboBox(self)
        self.anesthesia_protocol.resize(anesthesia_protocol['size']['col'],  anesthesia_protocol['size']['row'])
        self.anesthesia_protocol.move(anesthesia_protocol['position']['col'] + anesthesia_protocol['size']['col'],  anesthesia_protocol['position']['row'])
        self.anesthesia_protocol_label = QtGui.QLabel('Anesthesia protocol',  self)
        self.anesthesia_protocol_label.resize(anesthesia_protocol['size']['col'],  anesthesia_protocol['size']['row'])
        self.anesthesia_protocol_label.move(anesthesia_protocol['position']['col'],  anesthesia_protocol['position']['row'])
        self.anesthesia_protocol.addItems(anesthesia_protocol_items)
                
        self.mouse_strain = QtGui.QComboBox(self)
        self.mouse_strain.resize(mouse_strain['size']['col'],  mouse_strain['size']['row'])
        self.mouse_strain.move(mouse_strain['position']['col'] + mouse_strain['size']['col'],  mouse_strain['position']['row'])
        self.mouse_strain_label = QtGui.QLabel('Mouse strain',  self)
        self.mouse_strain_label.resize(mouse_strain['size']['col'],  mouse_strain['size']['row'])
        self.mouse_strain_label.move(mouse_strain['position']['col'],  mouse_strain['position']['row'])
        self.mouse_strain.addItems(mouse_strain_items)
        
        self.id = QtGui.QLabel('',  self)
        self.id.resize(id['size']['col'],  id['size']['row'])
        self.id.move(id['position']['col'],  id['position']['row'])
        
        #=== Experiment commands ===        
        
        self.acquire_camera_image_button = QtGui.QPushButton('Acquire camera image',  self)
        self.acquire_camera_image_button.resize(acquire_camera_image['size']['col'],  acquire_camera_image['size']['row'])
        self.acquire_camera_image_button.move(acquire_camera_image['position']['col'],  acquire_camera_image['position']['row'])
        self.connect(self.acquire_camera_image_button, QtCore.SIGNAL('clicked()'),  self.acquire_camera_image)
        
        self.acquire_z_stack_button = QtGui.QPushButton('Acquire z stack',  self)
        self.acquire_z_stack_button.resize(acquire_z_stack['size']['col'],  acquire_z_stack['size']['row'])
        self.acquire_z_stack_button.move(acquire_z_stack['position']['col'],  acquire_z_stack['position']['row'])
        self.connect(self.acquire_z_stack_button, QtCore.SIGNAL('clicked()'),  self.acquire_z_stack)
        
        self.two_photon_recording_button = QtGui.QPushButton('Two photon record',  self)
        self.two_photon_recording_button.resize(two_photon_recording['size']['col'],  two_photon_recording['size']['row'])
        self.two_photon_recording_button.move(two_photon_recording['position']['col'],  two_photon_recording['position']['row'])
        self.connect(self.two_photon_recording_button, QtCore.SIGNAL('clicked()'),  self.two_photon_recording)
        
        #=== Others ===
        self.log = QtGui.QLabel('',  self)
        self.log.resize(log['size']['col'],  log['size']['row'])
        self.log.move(log['position']['col'],  log['position']['row'])
        self.log.setText('')
        
        self.update_button = QtGui.QPushButton('Update',  self)
        self.update_button.resize(update['size']['col'],  update['size']['row'])
        self.update_button.move(update['position']['col'],  update['position']['row'])
        self.connect(self.update_button, QtCore.SIGNAL('clicked()'),  self.update)
        
    def default_parameters(self):
        self.acquire_camera_image_parameters = os.path.join(self.config.working_path, 'acquire_camera_image_parameters.m')
        self.acquire_z_stack_parameters = os.path.join(self.config.working_path, 'acquire_z_stack_parameters.m')
        self.two_photon_parameters = os.path.join(self.config.working_path, 'two_photon_parameters.m')
        
#        Mat file handling
#        data = scipy.io.loadmat('test.mat')
#        data = {}
#        data['x'] = x
#        scipy.io.savemat('test.mat',data)
        
    def generate_id(self):
        mouse_birth_date = self.mouse_birth_date.date()
        mouse_birth_date = '{0}{1}20{2}'.format(mouse_birth_date.day(),  mouse_birth_date.month(),  mouse_birth_date.year())
        gcamp_injection_date = self.gcamp_injection_date.date()
        gcamp_injection_date = '{0}{1}20{2}'.format(gcamp_injection_date.day(),  gcamp_injection_date.month(),  gcamp_injection_date.year())        
        
        #undefined variables
        stagex = 'tbd'
        stagey = 'tbd'
        stagez = 'tbd'
        i = 'tbd'
        data_type = 'tbd'
        experiment_class_name = 'tbd'
        experiment_config_name = 'tbd'
        #[mouse strain](b[birth date] i[injection date] [stagex] [stagey] [zpos])-r[i]-[data type]-[stim class name]-[stimcfgname]-[anesthesia]-[earpunch]
        id_text = '{0}(b{1}i{2}{3}{4}{5})-r{6}-{7}-{8}-{9}-{10}-{11}{12}' .format(
                                                                                   self.mouse_strain.currentText(),  
                                                                                   mouse_birth_date, 
                                                                                   gcamp_injection_date, 
                                                                                   stagex, stagey, stagez, 
                                                                                   i, data_type, 
                                                                                   experiment_class_name, experiment_config_name, 
                                                                                   self.anesthesia_protocol.currentText(), 
                                                                                   self.ear_punch_l.currentText(), self.ear_punch_r.currentText(), 
                                                                                   )
        self.id.setText(id_text)
        
    def load_previous_experiment(self):
        pass
        
    def acquire_camera_image(self):
        self.command_queue.put('SOCacquire_camera_imageEOC{0}EOP' .format(self.acquire_camera_image_parameters))        
        
    def acquire_z_stack(self):
        self.command_queue.put('SOCacquire_z_stackEOC{0}EOP' .format(self.acquire_z_stack_parameters))
        
    def two_photon_recording(self):
        self.command_queue.put('SOCtwo_photon_recordingEOC{0}EOP'.format(self.two_photon_parameters))
        
    def update(self):
        message = self.log.text()
        while not response_queue.empty():
            response = response_queue.get()
            print response
            message += response + ' '
        message = message[-100:]
        self.log.setText(message)        
        
    def closeEvent(self, e):
        e.accept()
        self.command_queue.put('SOCclose_connectionEOCEOP')
        time.sleep(1.0) #Enough time to close connection with MES
        sys.exit(0)

class GuiConfig(configuration.Config):
    def _create_application_parameters(self):
        if len(sys.argv) > 1:
            port = int(sys.argv[1])
        else:
            port = 10000
            
        if len(sys.argv) > 2:
            self.working_path =  sys.argv[2]
        else:
            self.working_path = ''
        MES = {'ip': '',  'port' : port,  'receive buffer' : 256}
        VISEXPMAN = {'ip': '',  'port' : 10001}
        VISEXPA = {'ip': '',  'port' : 10002}        
        self._create_parameters_from_locals(locals())
        
if __name__ == '__main__':
    config = GuiConfig()    
    command_queue = Queue.Queue()
    response_queue = Queue.Queue()
    server = network_interface.MesServer(config, command_queue, response_queue)
    server.start()
    app = Qt.QApplication(sys.argv)
    gui = Gui(config, command_queue, response_queue)
    app.exec_()
