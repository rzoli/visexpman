from visexpman.engine import jobhandler
import sys
import unittest,os



class TestRestore(unittest.TestCase):
    def test_01(self):
        import shutil
        wf='/mnt/databig/debug/3'
        #if os.path.exists(wf):
            #shutil.rmtree('/mnt/databig/debug/6')
        #shutil.copytree('/mnt/mdrive/invivo/rc/default_user/20160121/151', wf)
        jobhandler.offline(wf,'/mnt/databig/debug/1')


if __name__=='__main__':
    if len(sys.argv)==1:
        unittest.main()
    else:
        input_folder=sys.argv[1]
        output_folder=sys.argv[2]
        jobhandler.offline(input_folder,output_folder)

        
