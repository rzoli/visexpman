'''
generic.gui module has generic gui widgets like labeled widgets. It also contains some gui helper function
'''

import PyQt4.Qt as Qt
import PyQt4.QtGui as QtGui
import PyQt4.QtCore as QtCore

from visexpman.engine.generic import utils
from visexpman.engine.generic import stringop

class GroupBox(QtGui.QGroupBox):
    def __init__(self, parent, name):
        QtGui.QGroupBox.__init__(self, name, parent)
        self.create_widgets()
        self.create_layout()
        
    def create_widgets(self):
        pass
        
    def create_layout(self):
        pass

class LabeledInput(QtGui.QWidget):
    '''
    Default value in input field:
        self.input.setText(TEXT)
    '''
    def __init__(self, parent, label):
        QtGui.QWidget.__init__(self, parent)
        self.label = label
        self.create_widgets()
        self.create_layout()

    def create_widgets(self):
        self.labelw = QtGui.QLabel(self.label, self)
        self.input = QtGui.QLineEdit(self)

    def create_layout(self):
        self.layout = QtGui.QGridLayout()
        self.layout.addWidget(self.labelw, 0, 0)
        self.layout.addWidget(self.input, 0, 1)
        self.setLayout(self.layout)

class LabeledCheckBox(QtGui.QWidget):
    '''
    Default value in input field:
        self.input.setText(TEXT)
    '''
    def __init__(self, parent, label):
        QtGui.QWidget.__init__(self, parent)
        self.label = label
        self.create_widgets()
        self.create_layout()

    def create_widgets(self):
        self.labelw = QtGui.QLabel(self.label, self)
        self.input = QtGui.QCheckBox(self)

    def create_layout(self):
        self.layout = QtGui.QGridLayout()
        self.layout.addWidget(self.labelw, 0, 0)
        self.layout.addWidget(self.input, 0, 1)
        self.setLayout(self.layout)

class LabeledComboBox(QtGui.QWidget):
    '''
    Default value in input field:
        self.input.setText(TEXT)
    '''
    def __init__(self, parent, label,items=None):
        QtGui.QWidget.__init__(self, parent)
        self.label = label
        self.create_widgets()
        self.create_layout()
        if items is not None:
            self.input.addItems(QtCore.QStringList(items))
            

    def create_widgets(self):
        self.labelw = QtGui.QLabel(self.label, self)
        self.input = QtGui.QComboBox(self)

    def create_layout(self):
        self.layout = QtGui.QGridLayout()
        self.layout.addWidget(self.labelw, 0, 0)
        self.layout.addWidget(self.input, 0, 1)
        self.setLayout(self.layout)
        
class PushButtonWithParameter(QtGui.QWidget):
    '''
    Default value in input field:
        self.input.setText(TEXT)
    '''
    def __init__(self, parent, buttonname, parametername):
        QtGui.QWidget.__init__(self, parent)
        self.parametername = parametername
        self.buttonname = buttonname
        self.create_widgets()
        self.create_layout()

    def create_widgets(self):
        self.input = LabeledInput(self, self.parametername)
        self.button = QtGui.QPushButton(self.buttonname, self)

    def create_layout(self):
        self.layout = QtGui.QGridLayout()
        self.layout.addWidget(self.input, 0, 1, 1, 2)
        self.layout.addWidget(self.button, 0, 0)
        self.setLayout(self.layout)
        
class ParameterTable(QtGui.QTableWidget):
    '''
    A special QTable with two columns: first holds the parameter names, the second holds the corresponding parameter values
    '''
    def __init__(self, parent):
        QtGui.QTableWidget.__init__(self, parent)
        self.setColumnCount(2)
        self.setHorizontalHeaderLabels(QtCore.QStringList(['Parameter name', 'value']))
        self.verticalHeader().setDefaultSectionSize(20)
        
    def set_values(self, parameters, parname_order=None):
        '''
        Sets the content of the table.
        parameters: dictionary: keys: parameter names, values: parameter values.
        '''
        self.parameters = parameters
        if parameters.has_key('self.editable') and parameters['self.editable'] == 'False':
            lock=True
        else:
            lock=False
        if parameters.has_key('self.editable'):
            del parameters['self.editable']
        if parameters.has_key('self.editable'):
            nrows = len(parameters)-1
        else:
            nrows = len(parameters)
        self.setRowCount(nrows)
        self.setVerticalHeaderLabels(QtCore.QStringList(nrows*['']))
        for row in range(nrows):
            if parname_order is None:
                parname = str(parameters.keys()[row])
            else:
                parname = parname_order[row]
            item = QtGui.QTableWidgetItem(parname)
            item.setFlags(QtCore.Qt.ItemIsSelectable| QtCore.Qt.ItemIsEnabled)
            self.setItem(row, 0, item)#Set parameter name
            #Setting value of table element depends on the widget type
            if self.cellWidget(row,1) is None:
                item=QtGui.QTableWidgetItem(str(parameters[parname]))
                if lock:
                    item.setFlags(QtCore.Qt.ItemIsSelectable|QtCore.Qt.ItemIsEnabled)
                self.setItem(row, 1, item)
            elif hasattr(self.cellWidget(row,1), 'date'):
                d, m, y=map(int, str(parameters[parname]).split('-'))
                self.cellWidget(row,1).setDate(QtCore.QDate(y, m, d))
            elif hasattr(self.cellWidget(row,1), 'currentText'):
                #Find out index
                items = [str(self.cellWidget(row,1).itemText(i)) for i in range(self.cellWidget(row,1).count())]
                if str(parameters[parname]) in items:
                    index = items.index(str(parameters[parname]))
                    self.cellWidget(row,1).setCurrentIndex(index)
                elif self.cellWidget(row,1).isEditable():
                    self.cellWidget(row,1).setEditText(str(parameters[parname]))

    def get_values(self):
        '''
        Return values of table in a dictionary format
        '''
        current_values = {}
        for row in range(self.rowCount()):
            parname = str(self.item(row,0).text())
            if hasattr(self.item(row,1), 'text') and self.cellWidget(row,1) is None:
                current_values[parname] = str(self.item(row,1).text())
            elif hasattr(self.cellWidget(row,1), 'date'):
                date = self.cellWidget(row,1).date()
                current_values[parname] = '{0}-{1}-{2}'.format(date.day(), date.month(), date.year())
            elif hasattr(self.cellWidget(row,1), 'currentText'):
                current_values[parname] = str(self.cellWidget(row,1).currentText())
            elif self.item(row,1) is None:
                current_values[parname] = ''
            else:
                raise NotImplementedError('Reader for this type of widget is not implemented {0}. Parameter name: {1}'.format(self.item(row,1), parname))
        return current_values
        
def update_combo_box_list(self, widget, new_list,  selected_item = None):
    current_value = widget.currentText()
    try:
        if current_value in new_list:
            current_index = new_list.index(current_value)
        else:
            current_index = 0
    except:
        current_index = 0
        self.printc((current_value, new_list))
        self.printc(traceback.format_exc())
    items_list = QtCore.QStringList(new_list)
    widget.blockSignals(True)
    widget.clear()
    widget.addItems(QtCore.QStringList(new_list))
    widget.blockSignals(False)
    if selected_item != None and selected_item in new_list:
        widget.setCurrentIndex(new_list.index(selected_item))
    else:
        widget.setCurrentIndex(current_index)

def load_experiment_config_names(config, widget):
    '''
    Loads all experiment config names and adds them to a dropdown widget
    OBSOLETE
    '''
    if hasattr(config, 'user'):
        import visexpman
        experiment_config_list = utils.fetch_classes('visexpman.users.' + config.user,  required_ancestors = visexpman.engine.vision_experiment.experiment.ExperimentConfig, direct = False)
        experiment_config_names = []
        for experiment_config in experiment_config_list:
            experiment_config_names.append(experiment_config[1].__name__)
        experiment_config_names.sort()
        widget.addItems(QtCore.QStringList(experiment_config_names))
        try:
            if hasattr(config, 'EXPERIMENT_CONFIG'):
                widget.setCurrentIndex(experiment_config_names.index(config.EXPERIMENT_CONFIG))
        except ValueError:
            pass
    return experiment_config_list
    
class WidgetControl(object):
    def __init__(self, poller, config, widget):
        self.config = config
        self.poller = poller
        self.widget = widget
        self.printc = self.poller.printc
    
def connect_and_map_signal(self, widget, mapped_signal_parameter, widget_signal_name = 'clicked'):
    self.signal_mapper.setMapping(widget, QtCore.QString(mapped_signal_parameter))
    getattr(getattr(widget, widget_signal_name), 'connect')(self.signal_mapper.map)
