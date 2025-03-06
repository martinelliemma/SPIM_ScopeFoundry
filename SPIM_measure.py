'''
SPIM Measurement
*******************
@authors: Emma Martinelli, Andrea Bassi. Politecnico di Milano

'''
from ScopeFoundry import Measurement
from ScopeFoundry.helper_funcs import sibling_path, load_qt_ui_file
from ScopeFoundry import h5_io
import pyqtgraph as pg
import numpy as np
import os, time


class SpimMeasure(Measurement):
    name = "SPIM_measure"

    def setup(self):
        self.ui_filename = sibling_path(__file__, "spim.ui")
        self.ui = load_qt_ui_file(self.ui_filename)

        self.settings.New('save_h5', dtype=bool, initial=False)

        self.settings.New('Z_step', dtype=int, unit='um', initial=10)
        self.settings.New('Z_series', dtype=int, initial=10)
        self.settings.New('Time_series', dtype=int, initial=1)

        # how often we want to update the display
        self.settings.New('refresh_period', dtype=float, unit='s', spinbox_decimals=3, initial=0.05, vmin=0)

        #RISOLUZIONE: dimensione del pixel della camera e sella profondità in z
        self.settings.New('xsampling', dtype=float, unit='um', initial=0.65)
        self.settings.New('ysampling', dtype=float, unit='um', initial=0.65)
        self.settings.New('zsampling', dtype=float, unit='um', initial=10)

        self.auto_range = self.settings.New('auto_range', dtype=bool, initial=True)
        self.settings.New('auto_levels', dtype=bool, initial=True)
        self.settings.New('level_min', dtype=int, initial=60)
        self.settings.New('level_max', dtype=int, initial=4000)

        '''AGGIUNGI UN BOTTONE:
        self.add_operation('name',op_func)
        example: self.add_operation('measure',self.measure)'''

        self.image_gen = self.app.hardware['NeoAndorHW']
        self.stage = self.app.hardware['PI_HW']

    def setup_figure(self):
        """
        Runs once during App initialization, after setup()
        This is the place to make all graphical interface initializations,
        build plots, etc.
        """
        # connect ui widgets to measurement/hardware settings or functions
        self.ui.start_pushButton.clicked.connect(self.start)
        self.ui.interrupt_pushButton.clicked.connect(self.interrupt)
        self.settings.save_h5.connect_to_widget(self.ui.save_h5_checkBox)
        self.settings.auto_levels.connect_to_widget(self.ui.autoLevels_checkbox)
        self.settings.auto_range.connect_to_widget(self.ui.autoRange_checkbox)
        self.settings.level_min.connect_to_widget(self.ui.min_doubleSpinBox)
        self.settings.level_max.connect_to_widget(self.ui.max_doubleSpinBox)

        # Set up pyqtgraph graph_layout in the UI
        self.imv = pg.ImageView()
        self.ui.imageLayout.addWidget(self.imv)
        colors = [(0, 0, 0),
                  (45, 5, 61),
                  (84, 42, 55),
                  (150, 87, 60),
                  (208, 171, 141),
                  (255, 255, 255)
                  ]
        cmap = pg.ColorMap(pos=np.linspace(0.0, 1.0, 6), color=colors)
        self.imv.setColorMap(cmap)

    def update_display(self):
        """
        Displays (plots) the numpy array self.buffer.
        This function runs repeatedly and automatically during the measurement run.
        its update frequency is defined by self.display_update_period
        """
        self.image_gen.read_from_hardware()
        self.stage.read_from_hardware()

        self.display_update_period = self.settings['refresh_period']

        # self.settings['progress'] = (self.frame_index + 1) * 100 / self.length
        self.settings['progress'] = self.frame_index * 100 / self.length

        if hasattr(self, 'img'):
            # show the image
            self.imv.setImage(self.img.T,
                              autoLevels=self.settings['auto_levels'],
                              autoRange=self.auto_range.val,
                              levelMode='mono'
                              )

            if self.settings['auto_levels']:
                lmin, lmax = self.imv.getHistogramWidget().getLevels()
                self.settings['level_min'] = lmin
                self.settings['level_max'] = lmax
            else:
                self.imv.setLevels(min=self.settings['level_min'],
                                   max=self.settings['level_max'])

    def measure_ext(self):
        self.stage.read_from_hardware()
        self.image_gen.read_from_hardware()

        self.stage.motor.go_home()
        self.stage.motor.wait_on_target()

        print('start: ', self.stage.motor.get_position())

        step_length = self.settings['Z_step'] * 0.001
        self.length_saving = space_frame = self.settings['Z_series']
        time_frame = self.settings['Time_series']

        if not self.speed_check(step_length):
            return

        self.length = num_frame = space_frame * time_frame

        self.create_h5_file_ext(time_frame)

        self.image_gen.camera.acquisition_setup(num_frame)
        self.image_gen.camera.acquisition_start()

        self.frame_index = 0
        t = time.perf_counter()
        for time_idx in range(time_frame):
            if time_idx % 2 == 0:
                t_end = step_length*space_frame+0.5
                self.stage.motor.trigger(step_length, t_end)
            else:
                t_end = -0.5
                self.stage.motor.trigger(-step_length, t_end)

            for frame_idx in range(space_frame):
                t1 = time.perf_counter()
                self.frame_index += 1
                if time_idx % 2 == 0:
                    self.frame_index_saving = frame_idx
                else:
                    self.frame_index_saving = space_frame - frame_idx - 1

                print(self.frame_index_saving)

                self.image_gen.camera.image_wait()
                self.img = self.image_gen.camera.image_read()

                t_acqui = time.perf_counter() - t1
                t_a_1 = time.perf_counter()
                print('acquisition time: ', t_acqui)

                # access the group and save the image
                t_group = self.h5_group[f't{time_idx}']
                image_h5 = t_group['c0/image']
                image_h5[self.frame_index_saving, :, :] = self.img
                self.h5file.flush()

                print('saving time: ', time.perf_counter() - t_a_1)

                if self.interrupt_measurement_called:
                    break
        self.image_gen.camera.acquisition_stop()

        # make sure to close the data file
        self.h5file.close()
        self.settings['save_h5'] = False
        print(time.perf_counter()-t)


    def run(self):

        try:
            #start the camera
            self.frame_index = -1

            self.image_gen.read_from_hardware()
            self.length = self.image_gen.frame_num.val

            self.image_gen.camera.acquisition_clear()
            self.image_gen.camera.acquisition_setup(self.image_gen.frame_num.val)
            self.image_gen.camera.acquisition_start()

            # continuously get the last frame and put it in self.image, in order to
            # show it via self.update_display()

            while not self.interrupt_measurement_called:

                # If measurement is called, stop the acquisition, call self.measure
                # and get out of run()
                if self.settings['save_h5']:
                    # measure is triggered by save_h5 button
                    self.image_gen.camera.acquisition_stop()
                    self.image_gen.settings['trigger'] = 'ext'
                    self.measure_ext()
                    break

                self.image_gen.camera.image_wait()
                self.img = self.image_gen.camera.image_read()

                if self.interrupt_measurement_called:
                    break

        finally:
            self.image_gen.camera.acquisition_stop()
            self.image_gen.settings['trigger'] = 'int'
            if self.settings['save_h5'] and hasattr(self, 'h5file'):
                # make sure to close the data file
                self.h5file.close()
                self.settings['save_h5'] = False

    def create_saving_directory(self):

        if not os.path.isdir(self.app.settings['save_dir']):
            os.makedirs(self.app.settings['save_dir'])

    def create_h5_file_ext(self, t_frame):
        self.create_saving_directory()
        # file name creation
        timestamp = time.strftime("%y%m%d_%H%M%S", time.localtime())
        sample = self.app.settings['sample']
        # sample_name = f'{timestamp}_{self.name}_{sample}.h5'
        if sample == '':
            sample_name = '_'.join([timestamp, self.name])
        else:
            sample_name = '_'.join([timestamp, self.name, sample])
        fname = os.path.join(self.app.settings['save_dir'], sample_name + '.h5')

        self.h5file = h5_io.h5_base_file(app=self.app, measurement=self, fname=fname)
        self.h5_group = h5_io.h5_create_measurement_group(measurement=self, h5group=self.h5file)

        for t_idx in range (t_frame):
            # Group creation for each t_index
            t_group = self.h5_group.create_group(f't{t_idx}')

            img_size = (2160, 2560)
            dtype = 'uint16'

            length = self.length_saving

            self.image_h5 = t_group.create_dataset(name='c0/image',
                                                   shape=[length, img_size[0], img_size[1]],
                                                   dtype=dtype)
            self.image_h5.attrs['element_size_um'] = [self.settings['zsampling'], self.settings['ysampling'],
                                                      self.settings['xsampling']]

            self.positions_h5 = t_group.create_dataset(name='c0/position_mm',
                                                       shape=[length],
                                                       dtype=np.float32)

    def speed_check(self, step):
        import warnings

        self.image_gen.read_from_hardware()
        self.stage.read_from_hardware()

        time_stage = step * self.length_saving /  self.stage.velocity.value
        print(self.stage.velocity.value)

        time_camera = self.image_gen.exposure_time.value * 10
        print(self.image_gen.exposure_time.value)

        if (time_camera > time_stage) :
            warnings.warn('!! WARNING !!', UserWarning)
            warnings.warn('not enough time for acquiring', UserWarning)
            return False

        return True



