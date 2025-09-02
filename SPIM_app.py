'''
SPIM App
*******************
@authors: Emma Martinelli, Andrea Bassi. Politecnico di Milano

'''
import sys
from os.path import dirname
sys.path.append(dirname(dirname(__file__)))

from ScopeFoundry import BaseMicroscopeApp

class SPIM_app(BaseMicroscopeApp):
    name = 'SPIM_App'
    
    def setup(self):
        
        #Add hardware components

        print("Adding Camera Hardware Components")
        from NAC_ScopeFoundry_update.NAC_hw import NeoAndorHW
        self.add_hardware(NeoAndorHW(self))

        print("Adding Shutter Hardware Components")
        from Shutter_ScopeFoundry.shutter_hw import ShutterHW
        self.add_hardware(ShutterHW(self))

        print("Adding Translator Hardware Components")
        from PI_ScopeFoundry_update.PI_CG_hardware import PI_CG_HW
        self.add_hardware(PI_CG_HW(self, serial='0115500028'))
         
        # Add measurement components
        print("Create Measurement objects")
        from NAC_ScopeFoundry_update.NAC_measure import NeoAndorMeasure
        self.add_measurement(NeoAndorMeasure(self))
        print("Create Measurement objects")
        from SPIM_measure import SpimMeasure
        self.add_measurement(SpimMeasure(self))
        # measurement without the trigger
        from SPIM_measure_base import SpimMeasureBase
        self.add_measurement(SpimMeasureBase(self))

if __name__ == '__main__':
    import sys
    import os

    app = SPIM_app(sys.argv)

    # current file dir and select settings file:
    # path = os.path.dirname(os.path.realpath(__file__))
    # new_path = os.path.join(path, 'Settings', 'Settings.ini')
    # print(new_path)
    #
    # app.settings_load_ini(new_path)
    # # connect all the hardwares
    # for hc_name, hc in app.hardware.items():
    #     hc.settings['connected'] = True

    sys.exit(app.exec_())